"""Bottom mission-control toolbar.

Buttons (left → right):

* **Launch Mission** — opens the configuration dialog, then starts
  ``ros2 launch`` through the :class:`~mcs.ros.launch_manager.LaunchManager`.
* **Stop Mission** — graceful SIGINT-first shutdown; the station stays open.
  The ONLY button here that ends the mission.
* **E-STOP** — publishes ``stop``: robot-side that zeroes the thrust, closes
  the motor gate, disarms and latches. Leaves the parameter mode alone and
  leaves every node running.  Never disabled while ROS is up.
* **E-STOP + Stop Override** — the same ``stop``, then ``default`` through the
  confirmation sequence, handing the servo mapping back. Also leaves the nodes
  running: it used to terminate them, which made "give the mapping back" and
  "end the mission" impossible to ask for separately.
* **Publish Default/Override Control Mode** — alternates the two commands.
* **Manual Target** / **Continue Original Mission** — Manual Target arms the
  next map click and publishes nothing by itself; only Continue Original
  Mission publishes the ``[0.0, 0.0]`` resume sentinel (CLAUDE.md N2).
* **Measure** — toggles the distance tool.
* A one-line launch console + launch state LED.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core.signals import SignalBus
from mcs.gui import theme
from mcs.gui.dialogs.launch_dialog import LaunchDialog
from mcs.gui.widgets import StatusLed
from mcs.ros.command_center import CommandCenter
from mcs.ros.launch_manager import LaunchManager


class BottomToolbar(QWidget):
    """Mission control strip along the bottom of the main window."""

    manual_target_mode_changed = Signal(bool)
    measure_mode_changed = Signal(bool)
    continue_mission_clicked = Signal()
    create_pattern_clicked = Signal()
    mission_launched = Signal(object)   # LaunchParameters
    mission_stopped = Signal()

    def __init__(self, cfg: AppConfig, bus: SignalBus, launcher: LaunchManager,
                 commands: CommandCenter, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self._bus = bus
        self._launcher = launcher
        self._commands = commands

        outer = QVBoxLayout(self)
        self.setSizePolicy(
            QSizePolicy.Policy.Expanding,
            QSizePolicy.Policy.Fixed,
        )
        self.setMinimumWidth(0)
        outer.setContentsMargins(8, 4, 8, 6)
        outer.setSpacing(3)

        row = QHBoxLayout()
        row.setSizeConstraint(QHBoxLayout.SizeConstraint.SetNoConstraint)
        row.setSpacing(8)
        outer.addLayout(row)

        self._launch_led = StatusLed(12)
        row.addWidget(self._launch_led)

        self.launch_button = QPushButton("Launch Mission")
        self.launch_button.setObjectName("launchButton")
        self.launch_button.clicked.connect(self._on_launch)
        row.addWidget(self.launch_button)

        self.stop_button = QPushButton("Stop Mission")
        self.stop_button.setEnabled(False)
        self.stop_button.setToolTip(
            "Publishes 'default' on /blueboat/input_str, waits for confirmed "
            "transmission, then gracefully stops every launched node.")
        self.stop_button.clicked.connect(self._on_stop)
        row.addWidget(self.stop_button)

        row.addSpacing(14)

        # Two direct emergency buttons (no confirmation popup — an emergency
        # action must be one click). NEITHER terminates the launch; that is
        # Stop Mission's job alone. They differ only in whether the servo
        # mapping is handed back afterwards.
        self.estop_kill_button = QPushButton("E-STOP + Stop Override")
        self.estop_kill_button.setObjectName("estopButton")
        self.estop_kill_button.setToolTip(
            "Publish 'stop' (thrust zeroed, motors disabled, disarmed, "
            "latched), then 'default' to hand the servo mapping back to "
            "QGC/RC — confirmed by the param_mode echo. Nodes keep running; "
            "use Stop Mission to end the mission. One click, no dialog.")
        self.estop_kill_button.clicked.connect(self._commands.stop_override)
        row.addWidget(self.estop_kill_button)

        self.estop_button = QPushButton("E-STOP")
        self.estop_button.setObjectName("estopButton")
        self.estop_button.setToolTip(
            "Publish 'stop' on /blueboat/input_str: thrust zeroed, motors "
            "disabled, disarmed, and latched until an explicit 'enable'. "
            "Stays in override and leaves every node running. "
            "One click, no confirmation dialog.")
        self.estop_button.clicked.connect(self._commands.emergency_stop)
        row.addWidget(self.estop_button)

        self.mode_button = QPushButton()
        self.mode_button.clicked.connect(self._on_mode_toggle)
        row.addWidget(self.mode_button)
        self._refresh_mode_button()

        row.addSpacing(14)

        # Manual target: one-shot arming. Checking the button arms the next
        # map click as a target; the click publishes and auto-disarms, so the
        # map immediately returns to normal interaction (pan / inspect /
        # measure) while the boat drives to the target. 'Continue Original
        # Mission' appears while a target is active and ONLY publishes the
        # [0.0, 0.0] resume message.
        self.manual_button = QPushButton("Manual Target")
        self.manual_button.setCheckable(True)
        self.manual_button.setToolTip(
            "Arm: the next map click is published as a manual target, then "
            "the map returns to normal interaction. Press again to arm a "
            "replacement target; press while armed to cancel arming "
            "(publishes nothing).")
        self.manual_button.toggled.connect(self.manual_target_mode_changed.emit)
        row.addWidget(self.manual_button)

        self.continue_button = QPushButton("Continue Original Mission")
        self.continue_button.setToolTip(
            "Publish the [0.0, 0.0] manual target: master_control resumes "
            "the original mission. This button does nothing else.")
        self.continue_button.setVisible(False)
        # Reserve the button's footprint while hidden: its appearance after
        # a manual target used to reflow the toolbar and nudge the window /
        # panel geometry ("the window becomes weird").
        policy = self.continue_button.sizePolicy()
        policy.setRetainSizeWhenHidden(True)
        self.continue_button.setSizePolicy(policy)
        self.continue_button.clicked.connect(self.continue_mission_clicked.emit)
        row.addWidget(self.continue_button)

        self.measure_button = QPushButton("Measure")
        self.measure_button.setCheckable(True)
        self.measure_button.toggled.connect(self.measure_mode_changed.emit)
        row.addWidget(self.measure_button)

        row.addSpacing(14)

        self.designer_button = QPushButton("Create Survey Pattern")
        self.designer_button.setToolTip(
            "Open the Survey Pattern Designer: create, edit and manage "
            "trajectories. Saved missions appear in Launch Mission → "
            "custom paths.")
        self.designer_button.clicked.connect(self.create_pattern_clicked.emit)
        self.designer_button.setSizePolicy(
            QSizePolicy.Policy.Maximum,
            QSizePolicy.Policy.Fixed,
        )
        row.addWidget(self.designer_button)

        row.addStretch(1)

        self._estop_label = QLabel("")
        self._estop_label = QLabel("")
        self._estop_label.setSizePolicy(
            QSizePolicy.Policy.Ignored,
            QSizePolicy.Policy.Fixed,
        )
        self._estop_label.setMinimumWidth(0)
        self._estop_label.setStyleSheet(f"color: {theme.WARN}; font-weight: bold;")
        row.addWidget(self._estop_label)

        self._console = QLabel("")
        self._console.setSizePolicy(
            QSizePolicy.Policy.Ignored,
            QSizePolicy.Policy.Fixed,
        )
        self._console.setStyleSheet(
            f"color: {theme.TEXT_DIM}; font-family: 'DejaVu Sans Mono', monospace;"
            "font-size: 10px;")
        self._console.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse)
        outer.addWidget(self._console)

        for b in (
            self.launch_button,
            self.stop_button,
            self.estop_button,
            self.estop_kill_button,
            self.mode_button,
            self.manual_button,
            self.continue_button,
            self.measure_button,
            self.designer_button,
        ):
            b.setSizePolicy(
                QSizePolicy.Policy.Maximum,
                QSizePolicy.Policy.Fixed,
            )

        bus.launch_state_changed.connect(self._on_launch_state)
        bus.launch_output.connect(self._on_launch_output)
        bus.estop_state_changed.connect(self._on_estop_state)
        # The safe-shutdown sequence publishes 'default' outside the toggle
        # button's own bookkeeping; CommandCenter resyncs its state, and this
        # refresh keeps the label consistent with it.
        bus.estop_state_changed.connect(self._on_estop_state_mode_sync)

    # ================================================================ actions
    def _on_launch(self) -> None:
        dialog = LaunchDialog(self._cfg, self._launcher.last_parameters, self)
        if dialog.exec() != dialog.DialogCode.Accepted:
            return
        params = dialog.parameters()
        if params.enable_motors:
            confirm = QMessageBox.warning(
                self, "Motors enabled",
                "Motors are ENABLED for this launch — the boat will actually "
                "move.\n\nProceed?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No)
            if confirm != QMessageBox.StandardButton.Yes:
                return
        # Simulated GPS-anchored mission: offer the personalized Gazebo
        # worlds (BlueBoat-SSS-Sim folders) whose limits contain the path.
        # Kept out of LaunchDialog.parameters(), which must stay
        # non-interactive for the headless smoke test. In world mode the
        # simulator's mavros shim owns the GPS feed (the station's SimGps
        # stays disarmed — see MainWindow._on_mission_launched) and
        # full_mission_launch.py declares no spawn_yaw.
        if params.simulation and params.gps_simulated \
                and params.gps_anchored_source:
            from pathlib import Path

            from mcs.core.worlds import eligible_worlds  # lazy: yaml/numpy
            worlds = eligible_worlds(Path(self._cfg.launch.worlds_root),
                                     Path(params.gps_anchored_source))
            if worlds:
                from mcs.gui.dialogs.world_dialog import WorldChoiceDialog
                wdlg = WorldChoiceDialog(worlds, self)
                if wdlg.exec() != wdlg.DialogCode.Accepted:
                    return                        # Cancel aborts the launch
                if wdlg.selected_world_dir:
                    params.world_dir = wdlg.selected_world_dir
                    params.spawn_yaw_rad = None
        # Every Gazebo launch (empty world or personalized world) then
        # chooses the sea state the boat will feel. Cancel aborts.
        if params.simulation:
            from mcs.core.sea import SeaCatalog, find_presets_file
            from mcs.gui.dialogs.sea_state_dialog import SeaStateDialog
            catalog = SeaCatalog.load(
                find_presets_file(self._cfg.sea.presets_file))
            last = self._launcher.last_parameters
            sdlg = SeaStateDialog(catalog, self._cfg.sea,
                                  last.sea if last is not None else None,
                                  parent=self)
            if sdlg.exec() != sdlg.DialogCode.Accepted:
                return
            params.sea = sdlg.choice()
        if self._launcher.start(params):
            self.mission_launched.emit(params)

    def _on_stop(self) -> None:
        # Stop Mission is also a node-termination path, so it runs the same
        # guarantee as E-STOP: publish 'default', confirm transmission, and
        # only then terminate (CommandCenter.safe_shutdown — never a direct
        # launcher.stop()).
        
        self._commands.safe_stop_mission()
        self.mission_stopped.emit()

    def _on_mode_toggle(self) -> None:
        self._commands.publish_mode_toggle()
        self._refresh_mode_button()

    def _refresh_mode_button(self) -> None:
        nxt = self._commands.next_mode_command
        self.mode_button.setText(f"Publish {nxt.capitalize()} Control Mode")
        self.mode_button.setToolTip(
            f"Publishes String('{nxt}') on {self._cfg.topics.input_str}; "
            "the button then alternates to the other command.")

    def set_manual_target_active(self, active: bool) -> None:
        """Show 'Continue Original Mission' while a manual target is active."""
        self.continue_button.setVisible(active)

    # ================================================================ feedback
    def _on_launch_state(self, state: str) -> None:
        led = {"idle": "never", "starting": "warn",
               "running": "ok", "stopping": "warn"}.get(state, "warn")
        self._launch_led.set_status(led)
        self.launch_button.setEnabled(state == "idle")
        self.stop_button.setEnabled(state in ("starting", "running"))

    def _on_launch_output(self, line: str) -> None:
        self._console.setText(line[-160:])

    def _on_estop_state_mode_sync(self, state: str) -> None:
        """Resync the Default/Override label after a sequence that moved the mode.

        Only the 'default' sequence does. An E-STOP publishes 'stop', which is
        not a parameter mode at all, so its states must leave the label alone.
        """
        if state.startswith("estop"):
            return
        self._refresh_mode_button()

    def _on_estop_state(self, state: str) -> None:
        text = {"estop": "E-STOP: 'stop' published, awaiting robot ack…",
                "estop-confirmed": "E-STOP: confirmed — thrust cut and latched",
                "estop-timeout": "E-STOP: published, no ack (check robot_interface)",
                "estop-sim": "E-STOP: simulation — no robot to acknowledge",
                "publishing": "safe-shutdown: publishing 'default'…",
                "confirmed": "safe-shutdown: confirmed by param_mode echo",
                "already-default": "safe-shutdown: boat was already in 'default'",
                "timeout": "safe-shutdown: published, flushed (no echo — check chain)",
                "sim": "safe-shutdown: simulation — ack skipped",
                "idle": ""}.get(state, "")
        self._estop_label.setText(text)
