"""High-level operator commands, the **E-STOP** and the **safe-shutdown guarantee**.

Three operator actions, three separate jobs — they are deliberately NOT nested
(2026-09-04). Before that, E-STOP published ``default``, which dropped out of
override, and "E-STOP + Stop Override" additionally killed the launch, so an
operator could not cut the motors without also giving the servo mapping back and
could not give the mapping back without ending the mission:

===========================  =========================  ===========================
Action                       Publishes                  Terminates the launch?
===========================  =========================  ===========================
E-STOP                       ``stop``                   **no**
E-STOP + Stop Override       ``stop`` then ``default``  **no**
Stop Mission / App Exit      ``default``                yes, after confirmation
===========================  =========================  ===========================

``stop`` is what an emergency stop should be: robot-side it zeroes the thrust,
disables the motor gate, disarms and **latches** until an explicit ``enable``, all
without leaving override. Leaving override is a *transfer* of authority — the RC
channels go back to whatever else is transmitting — so it is the operator's
separate, deliberate second action, not part of the panic button.

Its acknowledgement is ``/blueboat/controller_ready`` going ``False``, which
``robot_interface`` publishes the moment it latches.

Every path that terminates ROS2 nodes — Stop Mission and Application Exit —
funnels through one sequence, :meth:`CommandCenter.safe_shutdown`, which
guarantees that::

    ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: default"

has actually been transmitted and received before anything is killed.

How the guarantee is implemented (layered, strongest evidence first)
--------------------------------------------------------------------
1. **Graph verification (delivery precondition).** Before publishing, the
   sequence queries ``publisher.get_subscription_count()`` — the ROS2 graph
   count of DDS subscriptions *matched* to our RELIABLE writer. A count > 0
   proves ``robot_interface``'s subscription has completed QoS discovery
   with this publisher: under DDS reliable QoS, a sample handed to the
   writer will then be delivered (with retransmission) as long as the writer
   stays alive. A count of 0 is reported to the operator (nothing on the
   robot side could receive the command) and the sequence still publishes
   and re-checks, in case discovery completes within the window.
2. **Publication.** ``publish()`` is synchronous into the DDS writer: when
   it returns, the sample is queued in the reliable send path. This is the
   "publication succeeded" check — an exception here aborts nothing silently;
   it is surfaced and the sequence falls back to step 4's grace delay.
3. **End-to-end application acknowledgement.** The strongest confirmation
   available in this stack: ``param_set`` echoes the applied mode on
   ``/blueboat/param_mode``. Receiving ``'default'`` back proves the command
   traversed publisher → robot_interface → param_set and was *acted on*, not
   merely delivered. The sequence waits for this echo up to
   ``estop_confirm_timeout_s``; at half the timeout it republishes once
   (idempotent — 'default' is a mode, not an increment).

   ``param_set`` now HEARTBEATS that topic at 1 Hz so a late station sees the
   mode without waiting for a transition. A heartbeat is not evidence that this
   command was acted on, so the sequence snapshots the last known mode before
   publishing and requires a *transition* into ``'default'``. The one case it
   accepts without a transition is a boat that was already in ``'default'`` when
   the operator asked — the state being demanded already holds, and that is
   reported as its own confirmation level rather than dressed up as an echo.
4. **Reliable-writer flush fallback.** If no echo arrives (e.g. ``param_set``
   not running), the matched-subscription evidence from step 1 plus a
   ``estop_flush_delay_s`` grace period — during which the process, node and
   writer are deliberately kept alive so the reliable protocol can complete
   its handshake/retransmissions — is accepted as transmission confirmation,
   and the operator is told which level of confirmation was obtained.
5. **Only after 3 or 4 completes** does node termination begin
   (:meth:`LaunchManager.stop`, itself SIGINT-first). The ordering is
   enforced structurally: termination lives in :meth:`_finish`, reachable
   only from the confirmation slots. Nothing in the application calls
   ``LaunchManager.stop`` directly except this class and the launch
   manager's own idle finalisation.

On completion the sequence emits ``SignalBus.shutdown_sequence_finished`` so
asynchronous callers (Application Exit) can proceed.
"""

from __future__ import annotations

import logging
from enum import Enum, auto

from PySide6.QtCore import QObject, QTimer

from mcs.config.settings import AppConfig
from mcs.core.signals import SignalBus
from mcs.ros.launch_manager import LaunchManager
from mcs.ros.ros_manager import RosManager

_LOG = logging.getLogger(__name__)


class _Phase(Enum):
    IDLE = auto()
    WAIT_CONFIRM = auto()
    FLUSHING = auto()


class CommandCenter(QObject):
    """Publishes operator commands; owns the safe-shutdown sequence."""

    def __init__(
        self,
        cfg: AppConfig,
        bus: SignalBus,
        ros: RosManager,
        launcher: LaunchManager,
    ) -> None:
        super().__init__()
        self._cfg = cfg
        self._bus = bus
        self._ros = ros
        self._launcher = launcher
        self._phase = _Phase.IDLE
        self._terminate_after = False
        self._republished = False
        self._simulation = False
        self._mode_toggle_next = "default"  # state of the Default/Override button
        # Last mode seen on /blueboat/param_mode. param_set heartbeats that topic,
        # so a repeat is not evidence of anything; the sequence needs a
        # transition, and this is what a transition is measured against.
        self._last_param_mode: str | None = None
        self._mode_before: str | None = None   # snapshot taken per sequence
        self._confirm_timer = QTimer(self)
        self._confirm_timer.setSingleShot(True)
        self._confirm_timer.timeout.connect(self._on_confirm_timeout)
        self._retry_timer = QTimer(self)
        self._retry_timer.setSingleShot(True)
        self._retry_timer.timeout.connect(self._on_retry)
        bus.param_mode_received.connect(self._on_param_mode)

        # E-STOP acknowledgement, kept in its own tiny state machine rather than
        # in _phase: the panic button must never be blocked by, and must never
        # block, an in-flight 'default' sequence.
        self._estop_pending = False
        self._estop_timer = QTimer(self)
        self._estop_timer.setSingleShot(True)
        self._estop_timer.timeout.connect(self._on_estop_timeout)
        bus.controller_ready_received.connect(self._on_controller_ready)

    # ------------------------------------------------------------- commands
    def _node(self):
        node = self._ros.node
        if node is None:
            self._bus.ros_log.emit("Cannot publish: ROS layer not running.")
        return node

    def publish_manual_target(self, x: float, y: float) -> None:
        node = self._node()
        if node:
            node.publish_manual_target(x, y)

    def resume_original_mission(self) -> None:
        """Publish the [0.0, 0.0] manual target that hands control back."""
        node = self._node()
        if node:
            node.publish_manual_target(0.0, 0.0)

    def publish_mode_toggle(self) -> str:
        """'Publish Default/Override Control Mode' button: alternates the two.

        Returns the command that was just published (for button relabelling).
        """
        node = self._node()
        if node is None:
            return self._mode_toggle_next
        cmd = self._mode_toggle_next
        node.publish_input_str(cmd)
        self._mode_toggle_next = "override" if cmd == "default" else "default"
        return cmd

    @property
    def next_mode_command(self) -> str:
        return self._mode_toggle_next

    def request_mission_path(self, total_time: float = 120.0, dt: float = 0.5) -> None:
        node = self._node()
        if node:
            node.request_mission_path(total_time, dt)

    def cancel_mission_path_request(self) -> None:
        """Drop any pending/in-flight /path_request. Silent when ROS is down —
        this is idle-path cleanup and must never complain during shutdown."""
        node = self._ros.node
        if node is not None:
            node.cancel_mission_path_request()

    def arm_sim_gps(self, model) -> None:
        """Point the bridge's simulated-GPS timer at a SimGpsModel."""
        node = self._node()
        if node:
            node.set_sim_gps(model)

    def disarm_sim_gps(self) -> None:
        """Stop the simulated GPS feed. Silent when ROS is down — this is
        idle-path cleanup and must never complain during shutdown."""
        node = self._ros.node
        if node is not None:
            node.set_sim_gps(None)

    def send_sea_state(self, payload: str) -> bool:
        """Forward a sea-state JSON command to the simulator (sim runs)."""
        node = self._ros.node
        if node is None:
            return False
        node.publish_sea_state_command(payload)
        return True

    def set_simulation_mode(self, simulation: bool) -> None:
        """Adapt the safe-shutdown sequence to the running graph.

        ``Sim_launch.py`` starts neither ``robot_interface`` nor ``param_set``:
        nothing subscribes to ``/blueboat/input_str`` and no ``param_mode``
        echo can ever arrive. In that mode the sequence still publishes the
        command (harmless, and correct should a hybrid graph ever exist) but
        skips the acknowledgement wait — waiting the full timeout for an echo
        that structurally cannot come would only delay the operator without
        adding any safety, since there is no physical robot to protect.
        """
        self._simulation = simulation

    # ----------------------------------------------------------- emergency
    def emergency_stop(self) -> None:
        """E-STOP button: cut the motors and nothing else.

        Publishes ``stop``. Does NOT change the parameter mode and does NOT
        terminate the launch — both are separate operator actions. Safe to
        press repeatedly and safe to press while a 'default' sequence is in
        flight; it shares no state with it.
        """
        _LOG.warning("EMERGENCY STOP triggered")
        node = self._node()
        if node is None:
            self._bus.ros_log.emit("E-STOP: ROS layer unavailable — nothing published.")
            return
        try:
            node.publish_input_str("stop")
        except Exception as exc:  # noqa: BLE001
            _LOG.error("publish('stop') raised: %s", exc)
            self._bus.ros_log.emit(f"E-STOP: publish failed ({exc}).")
            return

        if self._simulation:
            # Sim_launch.py starts neither robot_interface nor param_set, and
            # simulation_interface latches controller_ready True forever, so no
            # acknowledgement can structurally arrive.
            self._bus.estop_state_changed.emit("estop-sim")
            return

        self._estop_pending = True
        self._bus.estop_state_changed.emit("estop")
        self._estop_timer.start(int(self._cfg.estop_confirm_timeout_s * 1000))

    def stop_override(self) -> None:
        """E-STOP + Stop Override: cut the motors, THEN hand the mapping back.

        The order is the point. ``stop`` first, so thrust is already zero and
        the gate latched before the servo functions leave RC passthrough; only
        then ``default``, through the full confirmation sequence. Still does not
        terminate anything — that is Stop Mission.
        """
        _LOG.warning("E-STOP + Stop Override triggered")
        self.emergency_stop()
        self.safe_shutdown(terminate_nodes=False, reason="Stop Override")

    def _on_controller_ready(self, _t: float, ready: bool) -> None:
        """robot_interface withdraws readiness the instant it latches the stop."""
        if self._estop_pending and not ready:
            self._estop_timer.stop()
            self._estop_pending = False
            _LOG.info("E-STOP confirmed by controller_ready=False")
            self._bus.estop_state_changed.emit("estop-confirmed")

    def _on_estop_timeout(self) -> None:
        if not self._estop_pending:
            return
        self._estop_pending = False
        self._bus.estop_state_changed.emit("estop-timeout")
        self._bus.ros_log.emit(
            "E-STOP: 'stop' was published on a reliable writer but no "
            "controller_ready=False came back. Check that robot_interface is "
            "running and current; use Stop Mission if the boat is still moving.")

    # ------------------------------------------------------- shutdown paths

    def safe_stop_mission(self) -> None:
        """Stop Mission button: guarantee 'default' before graceful teardown."""
        _LOG.info("Stop Mission requested — running safe-shutdown sequence")
        self.safe_shutdown(terminate_nodes=True, reason="Stop Mission")

    def safe_app_exit(self) -> None:
        """Application Exit with a mission running: same guarantee."""
        _LOG.info("Application exit requested — running safe-shutdown sequence")
        self.safe_shutdown(terminate_nodes=True, reason="Application Exit")

    # ------------------------------------------------------------- sequence
    def safe_shutdown(self, terminate_nodes: bool, reason: str) -> None:
        """Publish → verify → confirm → (only then) terminate. See module doc."""
        if self._phase is not _Phase.IDLE:
            # A stronger request supersedes a weaker one in flight: pressing
            # Stop Mission while a Stop Override sequence is confirming must
            # still terminate. The publish already happened and the ordering
            # guarantee is unaffected — only the post-confirmation action is
            # upgraded, never downgraded.
            if terminate_nodes and not self._terminate_after:
                self._terminate_after = True
                _LOG.info("%s during %s — upgraded to terminate on confirmation",
                          reason, self._phase)
                self._bus.ros_log.emit(
                    f"{reason}: a shutdown sequence is already confirming; it "
                    "will terminate the launch when it completes.")
            else:
                _LOG.info("safe_shutdown re-entered during %s — ignored", self._phase)
                self._bus.ros_log.emit(
                    f"{reason}: a shutdown sequence is already in progress.")
            return
        node = self._ros.node
        if node is None:
            # Degraded mode: nothing can be published, but nothing robot-side
            # is reachable either. Tear the launch down if asked and finish.
            self._bus.ros_log.emit(
                f"{reason}: ROS layer unavailable — skipping publish, "
                "stopping launch process only.")
            if terminate_nodes:
                self._launcher.stop()
            self._bus.shutdown_sequence_finished.emit()
            return

        self._terminate_after = terminate_nodes
        self._republished = False
        self._phase = _Phase.WAIT_CONFIRM
        # Snapshot BEFORE publishing: the echo that confirms this command must
        # be a transition into 'default', not param_set's 1 Hz heartbeat.
        self._mode_before = self._last_param_mode
        # Keep the Default/Override toggle truthful. Set before the state emit,
        # because the toolbar refreshes that button's label on this very signal
        # and would otherwise read the stale value for up to a full timeout.
        self._mode_toggle_next = "override"
        self._bus.estop_state_changed.emit("publishing")

        # (1) Graph verification: matched reliable subscription = delivery
        #     precondition satisfied (DDS will deliver/retransmit).
        try:
            matched = node.input_str_subscriber_count()
        except Exception as exc:  # noqa: BLE001 - graph query never fatal here
            matched = -1
            _LOG.warning("Graph query failed: %s", exc)
        if matched == 0 and not self._simulation:
            self._bus.ros_log.emit(
                f"{reason}: WARNING — no subscriber matched on "
                f"{self._cfg.topics.input_str}; publishing anyway and holding "
                "the writer alive for late discovery.")
        elif matched == 0:
            _LOG.info("%s: no input_str subscriber (expected in simulation)", reason)
        elif matched < 0:
            # The graph query itself failed; -1 is a sentinel, not a count.
            _LOG.warning("%s: subscriber count unavailable — publishing anyway", reason)
        else:
            _LOG.info("%s: %s matched subscriber(s) on input_str", reason, matched)

        # (2) Publication — synchronous into the reliable DDS writer.
        try:
            node.publish_input_str("default")
        except Exception as exc:  # noqa: BLE001
            _LOG.error("publish('default') raised: %s — retrying once", exc)
            QTimer.singleShot(100, lambda: self._safe_republish())

        if self._simulation:
            # Simulation graph: no robot_interface/param_set → no subscriber,
            # no echo. Skip straight to the flush-then-terminate step; the
            # ordering guarantee (publish before any kill) is preserved.
            self._phase = _Phase.FLUSHING
            self._bus.estop_state_changed.emit("sim")
            _LOG.info("%s: simulation graph — ack skipped, flushing", reason)
            QTimer.singleShot(
                int(self._cfg.estop_flush_delay_s * 1000), self._finish)
            return

        # (3a) The boat was already in 'default' when the operator asked. The
        #      state being demanded holds; say so as its own confirmation level
        #      rather than waiting for a transition that will never come (and
        #      rather than letting the heartbeat impersonate an echo).
        if self._mode_before == "default":
            self._phase = _Phase.FLUSHING
            self._bus.estop_state_changed.emit("already-default")
            _LOG.info("%s: boat already in 'default' — no transition to wait for",
                      reason)
            QTimer.singleShot(
                int(self._cfg.estop_flush_delay_s * 1000), self._finish)
            return

        # (3) Wait for the end-to-end param_mode echo; republish once at T/2.
        timeout_ms = int(self._cfg.estop_confirm_timeout_s * 1000)
        self._confirm_timer.start(timeout_ms)
        self._retry_timer.start(timeout_ms // 2)

    def _safe_republish(self) -> None:
        node = self._ros.node
        if node is not None and self._phase is _Phase.WAIT_CONFIRM:
            try:
                node.publish_input_str("default")
            except Exception as exc:  # noqa: BLE001
                _LOG.error("republish('default') raised: %s", exc)

    def _on_retry(self) -> None:
        if self._phase is _Phase.WAIT_CONFIRM and not self._republished:
            self._republished = True
            _LOG.info("No param_mode echo yet — republishing 'default' once")
            self._safe_republish()

    def _on_param_mode(self, _t: float, mode: str) -> None:
        self._last_param_mode = mode
        if (self._phase is _Phase.WAIT_CONFIRM and mode == "default"
                and self._mode_before != "default"):
            # (3) succeeded: full-chain, acted-upon confirmation.
            self._confirm_timer.stop()
            self._retry_timer.stop()
            _LOG.info("safe-shutdown: 'default' confirmed by param_mode echo")
            self._bus.estop_state_changed.emit("confirmed")
            self._finish()

    def _on_confirm_timeout(self) -> None:
        if self._phase is not _Phase.WAIT_CONFIRM:
            return
        # (4) Fallback: no application echo. The command was handed to a
        # reliable writer with (normally) a matched subscription; keep the
        # writer alive for the flush delay so DDS can complete delivery,
        # then — and only then — allow termination.
        self._phase = _Phase.FLUSHING
        self._bus.estop_state_changed.emit("timeout")
        self._bus.ros_log.emit(
            "safe-shutdown: no param_mode echo — 'default' was published on a "
            "reliable writer; flushing before any node termination.")
        QTimer.singleShot(int(self._cfg.estop_flush_delay_s * 1000), self._finish)

    def _finish(self) -> None:
        """(5) Termination — structurally reachable only after 3 or 4."""
        self._phase = _Phase.IDLE
        terminate = self._terminate_after
        self._terminate_after = False
        if terminate:
            self._launcher.stop()
        # Return the toolbar's status label to blank. Without this the last
        # phase text ("no echo — check chain") stuck for the rest of the
        # session, long after the condition it described was over.
        self._bus.estop_state_changed.emit("idle")
        self._bus.shutdown_sequence_finished.emit()
