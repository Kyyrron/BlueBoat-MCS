"""Run sessions: a real-robot launch that stays connected between runs.

Since 2026-10-09 every real-robot launch from the station is a **session**
(``BlueBoat_launch.py session_mode:=True``). The boat connects and goes into
override, but ``master_control`` idles -- manual target and arrow keys drive
it, nothing follows a path or the pinger, nothing is logged -- until the
operator presses START RUN. A run ends with STOP RUN (its CSV, ``.npy`` and
report are written by the control stack); the session ends with Turn Off
Connection (the v1.0 safe shutdown, unchanged).

This object owns the station's half of that protocol::

    station  --/blueboat/run_command-->  master_control   (configure/start/stop)
    station  <--/blueboat/run_state----  master_control   (ack + 1 Hz heartbeat)
    station  <--/blueboat/run_log------  robot_interface  (the run's CSV)

and the one file ``path_generation`` watches in a session
(``LaunchConfig.session_trajectory_file``): each run's path is written there
just before the start command -- a GPS-anchored mission deployed onto its
real-world position (``io_yaml.deploy_mission``), any other mission placed at
the boat (``io_yaml.place_mission``: first sample on the boat, first heading
along its heading). Written before ``start`` goes out, so master_control's
first window request after the start already reads the new file; nothing
waits, which is what makes the boat move the moment the button is pressed.

Commands carry a ``seq``; ``run_state`` echoes the last one handled with
``ok`` / ``detail``. ``configure`` is idempotent and re-sent until
acknowledged (a first publish can race DDS discovery); ``start`` / ``stop``
are sent once and reported if unacknowledged -- master_control's own state,
heartbeated, stays the truth about whether a run is active.

No widgets here: the RUN box (``gui/run_panel.py``) and the toolbar read
:meth:`status` and :meth:`start_blockers` on the shared 10 Hz tick.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from pathlib import Path

from PySide6.QtCore import QObject, QTimer, Signal

from mcs.config.settings import AppConfig
from mcs.core.signals import SignalBus
from mcs.models.store import DataStore, NextMission, now_mono
from mcs.ros.command_center import CommandCenter
from mcs.ros.launch_manager import LaunchParameters, sanitize_note

_LOG = logging.getLogger(__name__)

_CONFIGURE_RETRIES = 5


@dataclass
class RunStatus:
    """What the RUN box shows (one snapshot per tick)."""

    connected: bool
    controller: str          # e.g. "PID — loaded", "MPC — loading…"
    controller_color: str    # "ok" | "warn" | "err" | "dim"
    state: str               # e.g. "Run 3 — 02:31", "Connected — no run"
    state_color: str
    log: str                 # current / last CSV name
    log_color: str
    bag: str                 # the run's ros2bag (v1.1): recording / saved / off
    bag_color: str
    running: bool            # the button reads STOP RUN
    pending: str             # "start" | "stop" | ""


class RunSession(QObject):
    """Station side of a real-robot run session."""

    #: (message, timeout_ms) for the status bar
    message = Signal(str, int)
    #: trajectory string ("from_yaml:<session file>") to preview on the map
    preview_requested = Signal(str)
    #: the map preview no longer describes the next run
    preview_cleared = Signal()
    #: arm the deferred GPS deployment of this source into the session file
    deploy_requested = Signal(str)
    #: arrow control must end without a hold point (a run is starting)
    arrows_release_requested = Signal()
    #: the manual target was cleared by master_control (start / stop)
    manual_target_cleared = Signal()

    def __init__(self, cfg: AppConfig, bus: SignalBus, store: DataStore,
                 commands: CommandCenter) -> None:
        super().__init__()
        self._cfg = cfg
        self._bus = bus
        self._store = store
        self._commands = commands
        self._seq = 0
        self._configure_seq: int | None = None
        self._configure_acked = False
        self._configure_tries = 0
        self._pending_seq: int | None = None
        self._deployed_from = ""        # source currently deployed in the file
        self._last_detail = ""          # last refusal / error worth showing

        self._ack_timer = QTimer(self)
        self._ack_timer.setSingleShot(True)
        self._ack_timer.timeout.connect(self._on_ack_timeout)
        self._configure_timer = QTimer(self)
        self._configure_timer.setSingleShot(True)
        self._configure_timer.timeout.connect(self._on_configure_timeout)
        self._log_timer = QTimer(self)
        self._log_timer.setSingleShot(True)
        self._log_timer.timeout.connect(self._on_log_timeout)

        bus.run_state_received.connect(self._on_run_state)
        bus.run_log_received.connect(self._on_run_log)
        commands.set_before_terminate(self._stop_before_terminate)

    # ================================================================ lifecycle
    @property
    def session_file(self) -> Path:
        return Path(self._cfg.launch.session_trajectory_file).expanduser()

    def begin(self, params: LaunchParameters) -> None:
        """A real-robot launch just started: a fresh session, no run."""
        ss = self._store.session
        ss.active = True
        ss.motors_enabled = bool(params.enable_motors)
        source = params.gps_anchored_source or params.session_source
        ss.next = NextMission(
            source=source,
            name=Path(source).stem if source else "",
            anchored=bool(params.gps_anchored_source),
            use_pinger=bool(params.use_pinger),
            controller_type=params.controller_type or "PID",
            note=params.note,
        )
        self._reset_run_state()
        self._configure_seq = None
        self._configure_acked = False
        self._configure_tries = 0
        self._deployed_from = ""
        self._last_detail = ""
        self._store.mission.following = False
        # Never let a previous session's path be served (or drawn by the GCS)
        # before this session decides what its first run executes. The GPS
        # deployment watcher unlinks it too for an anchored first mission.
        self.session_file.unlink(missing_ok=True)

    def end(self) -> None:
        """The launch is gone (idle): forget the session."""
        ss = self._store.session
        ss.active = False
        self._reset_run_state()
        self._store.mission.following = True
        for timer in (self._ack_timer, self._configure_timer, self._log_timer):
            timer.stop()

    def _reset_run_state(self) -> None:
        ss = self._store.session
        ss.run_active = False
        ss.run_pending = ""
        ss.run = 0
        ss.state = {}
        ss.state_t = None
        ss.log = {}
        ss.log_t = None
        ss.arrow_active = False
        ss.arrow_cmd = (0.0, 0.0)
        self._pending_seq = None

    def mark_deployed(self, source: str) -> None:
        """MainWindow's GPS watcher wrote *source* into the session file."""
        self._deployed_from = str(source)

    # ================================================================ commands
    def _next_seq(self) -> int:
        self._seq += 1
        return self._seq

    def _send_configure(self) -> None:
        nxt = self._store.session.next
        seq = self._next_seq()
        self._configure_seq = seq
        self._configure_acked = False
        self._configure_tries += 1
        payload = {
            "cmd": "configure",
            "controller_type": nxt.controller_type,
            "use_pinger": bool(nxt.use_pinger),
            "note": sanitize_note(nxt.note),
            "mission": "pinger" if nxt.use_pinger else nxt.name,
            "seq": seq,
        }
        self._commands.send_run_command(payload)
        self._configure_timer.start(int(self._cfg.run.ack_timeout_s * 1000))

    def update_mission(self, nxt: NextMission) -> None:
        """Update Mission confirmed: what the next run executes."""
        ss = self._store.session
        if not ss.active or ss.run_active or ss.run_pending:
            self.message.emit("Update Mission is only possible between runs.", 6000)
            return
        ss.next = nxt
        self._configure_tries = 0
        self._last_detail = ""
        self._send_configure()

        # The map preview must describe the next run, never the last one.
        self.preview_cleared.emit()
        self._deployed_from = ""
        self.session_file.unlink(missing_ok=True)
        if not nxt.use_pinger and nxt.source and nxt.anchored:
            self.deploy_requested.emit(nxt.source)
        what = ("the USBL pinger" if nxt.use_pinger
                else f"'{nxt.name}'" + (" (GPS)" if nxt.anchored
                                        else " (placed at the boat at START RUN)"))
        self.message.emit(f"Next run: {what} with {nxt.controller_type}.", 8000)

    # ---------------------------------------------------------------- gating
    def start_blockers(self) -> list[str]:
        """Why START RUN is not possible now ([] = it is)."""
        s = self._store
        ss = s.session
        if not ss.active:
            return ["no run session"]
        if not ss.state_fresh(self._cfg.run.state_fresh_s):
            return ["master_control not reporting yet"]
        if ss.run_pending:
            return [f"waiting for the {ss.run_pending} acknowledgement"]
        state = ss.state.get("state")
        if state == "running" or ss.run_active:
            return ["a run is active"]
        if state == "loading":
            return [f"{ss.state.get('controller_type', '')} loading…"]
        if state == "error":
            return [f"controller error: {ss.state.get('detail', '')}"]
        nxt = ss.next
        reasons = []
        if not self._config_applied():
            reasons.append("mission settings not applied yet")
        if self._commands.estop_latched:
            reasons.append("E-STOP latched — Release E-STOP first")
        elif not s.robot.controller_ready:
            reasons.append("robot not ready (override not confirmed)")
        if not s.robot.has_odom:
            reasons.append("no odometry")
        if not nxt.use_pinger:
            if not nxt.source:
                reasons.append("no path chosen — Update Mission")
            elif nxt.anchored and not (s.geo.is_valid
                                       or self._deployed_from == nxt.source):
                reasons.append("waiting for GPS to anchor the path")
        return reasons

    def _config_applied(self) -> bool:
        st = self._store.session.state
        nxt = self._store.session.next
        return (self._configure_acked
                and st.get("controller_type") == nxt.controller_type
                and bool(st.get("use_pinger")) == bool(nxt.use_pinger)
                and st.get("note", "") == sanitize_note(nxt.note))

    def can_update(self) -> bool:
        ss = self._store.session
        return (ss.active and ss.state_fresh(self._cfg.run.state_fresh_s)
                and not ss.run_active and not ss.run_pending
                and ss.state.get("state") != "loading")

    # ---------------------------------------------------------------- start
    def start_run(self) -> bool:
        """START RUN: write the run's path, then start -- no waiting."""
        blockers = self.start_blockers()
        if blockers:
            self.message.emit("START RUN not possible: " + "; ".join(blockers), 8000)
            return False
        if self._commands.run_command_subscriber_count() == 0:
            self.message.emit("START RUN not sent: master_control is not "
                              "subscribed to the run commands.", 8000)
            return False

        s = self._store
        nxt = s.session.next
        dst = self.session_file
        if not nxt.use_pinger:
            from mcs.designer import io_yaml  # lazy: PyYAML machinery
            src = Path(nxt.source)
            try:
                if nxt.anchored:
                    geo = s.geo
                    if geo.is_valid and geo.fit is not None:
                        # Re-deployed with the CURRENT fit: a long session must
                        # not run on the anchor of its first minute.
                        io_yaml.deploy_mission(src, geo.fit, dst)
                    elif self._deployed_from == nxt.source and dst.exists():
                        self.message.emit(
                            "GPS fit not valid right now — running the path as "
                            "deployed earlier in this session.", 8000)
                    else:
                        raise ValueError("no GPS anchor to deploy the path with")
                else:
                    r = s.robot
                    io_yaml.place_mission(src, r.x, r.y, r.yaw, dst)
            except Exception as exc:  # noqa: BLE001 - surfaced, run not started
                _LOG.error("START RUN: path not written: %s", exc)
                self.message.emit(f"START RUN aborted — path not written: {exc}", 10000)
                return False
            self._deployed_from = nxt.source

        # A run starts following now: arrow control ends without a hold point,
        # and master_control drops any manual target itself.
        self.arrows_release_requested.emit()
        seq = self._next_seq()
        if not self._commands.send_run_command({"cmd": "start", "seq": seq}):
            return False
        ss = s.session
        ss.run_pending = "start"
        self._pending_seq = seq
        s.mission.manual_target = None
        s.mission.following = True
        s.reset_experiment()           # the display now describes this run
        self.manual_target_cleared.emit()
        self._ack_timer.start(int(self._cfg.run.ack_timeout_s * 1000))
        self._log_timer.start(int(self._cfg.run.log_confirm_timeout_s * 1000))
        if not nxt.use_pinger:
            self.preview_requested.emit(f"from_yaml:{dst}")
        _LOG.info("START RUN sent (seq %d): %s", seq, nxt.summary())
        return True

    # ---------------------------------------------------------------- stop
    def stop_run(self) -> bool:
        """STOP RUN: end the run; the boat stops and stays in override."""
        ss = self._store.session
        if not (ss.run_active or ss.run_pending == "start"):
            return False
        seq = self._next_seq()
        if not self._commands.send_run_command({"cmd": "stop", "seq": seq}):
            return False
        ss.run_active = False
        ss.run_pending = "stop"
        self._pending_seq = seq
        self._store.mission.following = False
        self._store.mission.manual_target = None
        self._store.mission.stopped_t = now_mono()
        self.manual_target_cleared.emit()
        self._log_timer.stop()
        self._ack_timer.start(int(self._cfg.run.ack_timeout_s * 1000))
        if not ss.next.use_pinger and not ss.next.anchored:
            # A placed path belongs to the run that just ended: the next run
            # is placed afresh wherever the boat then is.
            self.preview_cleared.emit()
            self._deployed_from = ""
            self.session_file.unlink(missing_ok=True)
        _LOG.info("STOP RUN sent (seq %d)", seq)
        return True

    def _stop_before_terminate(self) -> None:
        """Turn Off Connection / app exit: a run still active is stopped first,
        so its log is closed and filed by the control stack itself."""
        ss = self._store.session
        if ss.active and (ss.run_active or ss.run_pending == "start"):
            _LOG.info("Turn Off Connection with a run active — stopping it first")
            self.stop_run()

    # ================================================================ feedback
    def _on_run_state(self, t: float, payload: dict) -> None:
        ss = self._store.session
        if not ss.active:
            return
        first = ss.state_t is None
        ss.state = payload
        ss.state_t = t
        seq = payload.get("seq")
        ok = bool(payload.get("ok", True))
        detail = str(payload.get("detail", ""))

        if first or (self._configure_seq is None):
            self._send_configure()       # the session's first mission settings
        elif seq is None and self._configure_acked:
            # master_control came back without our settings (it restarted):
            # its heartbeat no longer echoes any command. Send them again.
            self._configure_tries = 0
            self._send_configure()

        if self._configure_seq is not None and seq == self._configure_seq \
                and not self._configure_acked:
            self._configure_acked = True
            self._configure_timer.stop()
            if not ok:
                self._last_detail = detail
                self.message.emit(f"Mission settings refused: {detail}", 10000)

        if self._pending_seq is not None and seq == self._pending_seq:
            pending, self._pending_seq = ss.run_pending, None
            ss.run_pending = ""
            self._ack_timer.stop()
            if pending == "start":
                if ok and payload.get("state") == "running":
                    ss.run_active = True
                    ss.run = int(payload.get("run") or 0)
                    self.message.emit(f"Run {ss.run} started.", 5000)
                else:
                    ss.run_active = False
                    self._store.mission.following = False
                    self._log_timer.stop()
                    self._last_detail = detail
                    self.message.emit(f"START RUN refused: {detail}", 10000)
            elif pending == "stop":
                ss.run_active = False
                self.message.emit(f"Run {ss.run} stopped — log written.", 6000)
        elif not ss.run_pending:
            # No command in flight: master_control's own state is the truth.
            running = payload.get("state") == "running"
            if running != ss.run_active:
                ss.run_active = running
                self._store.mission.following = running
                if not running and self._store.mission.stopped_t is None:
                    self._store.mission.stopped_t = now_mono()
            if running:
                ss.run = int(payload.get("run") or ss.run)

        # The loaded controller is what the display reports.
        m = self._store.mission
        m.controller_type = str(payload.get("controller_type") or m.controller_type)
        m.use_pinger = bool(payload.get("use_pinger", m.use_pinger))

    def _on_run_log(self, t: float, payload: dict) -> None:
        ss = self._store.session
        if not ss.active:
            return
        ss.log = payload
        ss.log_t = t
        if payload.get("logging") and payload.get("run") == ss.run:
            self._log_timer.stop()

    def _on_ack_timeout(self) -> None:
        ss = self._store.session
        pending = ss.run_pending
        if not pending:
            return
        ss.run_pending = ""
        self._pending_seq = None
        if pending == "start":
            # Not acknowledged: master_control's heartbeat decides from here.
            ss.run_active = False
            self._store.mission.following = False
        self.message.emit(
            f"{pending.upper()} RUN not acknowledged by master_control within "
            f"{self._cfg.run.ack_timeout_s:.0f} s — check the launch console.", 10000)
        self._bus.ros_log.emit(f"Run session: '{pending}' not acknowledged.")

    def _on_configure_timeout(self) -> None:
        if self._configure_acked or not self._store.session.active:
            return
        if self._configure_tries < _CONFIGURE_RETRIES:
            self._send_configure()
        else:
            self.message.emit("master_control does not acknowledge the mission "
                              "settings — check the launch console.", 10000)

    def _on_log_timeout(self) -> None:
        ss = self._store.session
        if ss.run_active and not (ss.log.get("logging") and ss.log.get("run") == ss.run):
            self._last_detail = "position log not confirmed by robot_interface"
            self.message.emit("WARNING: robot_interface has not confirmed this "
                              "run's position log — field data may be missing.",
                              15000)
            self._bus.ros_log.emit("Run session: position log NOT confirmed.")
        elif (ss.run_active and ss.log.get("bag_enabled")
              and not ss.log.get("bag_recording")):
            self.message.emit("WARNING: this run's ros2bag is not recording — "
                              "the CSV is, but the run will not be replayable.",
                              15000)
            self._bus.ros_log.emit("Run session: run bag NOT recording.")

    # ================================================================ display
    def status(self) -> RunStatus:
        s = self._store
        ss = s.session
        fresh = ss.state_fresh(self._cfg.run.state_fresh_s)
        st = ss.state
        nxt = ss.next

        if not fresh:
            controller, c_color = "waiting for master_control…", "warn"
        elif st.get("state") == "loading":
            controller, c_color = f"{st.get('controller_type', '')} — loading…", "warn"
        elif st.get("state") == "error":
            controller, c_color = (f"error — {st.get('detail', '')}", "err")
        elif not self._config_applied():
            controller, c_color = (f"{nxt.controller_type} — applying…", "warn")
        else:
            controller, c_color = f"{st.get('controller_type', '')} — loaded", "ok"

        elapsed = s.mission.elapsed_s()
        if ss.run_pending == "start":
            state, s_color = f"Starting run {ss.run + 1}…", "warn"
        elif ss.run_pending == "stop":
            state, s_color = f"Stopping run {ss.run}…", "warn"
        elif ss.run_active:
            state, s_color = f"Run {ss.run} — {_fmt(elapsed)}", "ok"
        elif ss.run > 0:
            state = f"Connected — last run {ss.run} ({_fmt(elapsed)})"
            s_color = "dim"
        elif fresh:
            state, s_color = "Connected — no run yet", "dim"
        else:
            state, s_color = "Connecting…", "warn"
        if self._commands.estop_latched:
            state, s_color = state + " · E-STOP latched", "err"

        log = ss.log
        csv = Path(str(log.get("csv") or "")).name
        if log.get("logging"):
            log_text, l_color = csv, "ok"
        elif ss.run_active and ss.run_pending == "":
            log_text, l_color = "not confirmed", "err"
        elif csv:
            log_text, l_color = f"saved: {csv}", "dim"
        else:
            log_text, l_color = "—", "dim"

        # The run's ros2bag (robot_interface, v1.1). A control stack older
        # than v1.1 sends no bag fields at all.
        bag = Path(str(log.get("bag") or "")).name
        if "bag_enabled" not in log:
            bag_text, b_color = "—", "dim"
        elif not log.get("bag_enabled"):
            bag_text, b_color = "off (record_bag:=false)", "dim"
        elif log.get("bag_recording"):
            bag_text, b_color = "recording", "ok"
        elif ss.run_active and ss.run_pending == "":
            bag_text, b_color = "not recording", "err"
        elif bag:
            bag_text, b_color = f"saved: {bag}", "dim"
        else:
            bag_text, b_color = "armed", "dim"

        return RunStatus(connected=fresh, controller=controller,
                         controller_color=c_color, state=state, state_color=s_color,
                         log=log_text, log_color=l_color,
                         bag=bag_text, bag_color=b_color,
                         running=ss.run_active or ss.run_pending == "start",
                         pending=ss.run_pending)


def _fmt(seconds: float | None) -> str:
    if seconds is None:
        return "0:00"
    seconds = int(max(0.0, seconds))
    h, rem = divmod(seconds, 3600)
    m, s = divmod(rem, 60)
    return f"{h:d}:{m:02d}:{s:02d}" if h else f"{m:d}:{s:02d}"
