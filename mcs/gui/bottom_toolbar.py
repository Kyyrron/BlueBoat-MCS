"""Bottom mission-control toolbar.

Buttons (left → right):

* **Launch Mission** — opens the configuration dialog, then starts
  ``ros2 launch`` through the :class:`~mcs.ros.launch_manager.LaunchManager`.
* **Stop Mission** — graceful SIGINT-first shutdown; the station stays open.
* **EMERGENCY STOP** — sequenced: publish ``default`` on
  ``/blueboat/input_str``, wait for confirmation, then (optionally)
  terminate nodes.  Never disabled while ROS is up.
* **Publish Default/Override Control Mode** — alternates the two commands.
* **Manual Target** / **Continue Original Mission** — toggles the map's
  manual-target mode; deactivation publishes ``[0.0, 0.0]``.
* **Measure** — toggles the distance tool.
* A one-line launch console + launch state LED.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QHBoxLayout, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget, QSizePolicy,
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
        # action must be one click). Both run the same guaranteed sequence
        # (publish 'default' → confirm transmission → …); they differ only in
        # whether the launched nodes are terminated afterwards.
        self.estop_kill_button = QPushButton("E-STOP + Stop Override")
        self.estop_kill_button.setObjectName("estopButton")
        self.estop_kill_button.setToolTip(
            "Publish 'default' on /blueboat/input_str, confirm transmission, "
            "THEN terminate every launched node (stops whatever is driving "
            "the motors). One click, no confirmation dialog.")
        self.estop_kill_button.clicked.connect(
            lambda: self._commands.emergency_stop(terminate_nodes=True))
        row.addWidget(self.estop_kill_button)

        self.estop_button = QPushButton("E-STOP")
        self.estop_button.setObjectName("estopButton")
        self.estop_button.setToolTip(
            "Publish 'default' on /blueboat/input_str and confirm "
            "transmission. Nodes keep running. One click, no confirmation "
            "dialog.")
        self.estop_button.clicked.connect(
            lambda: self._commands.emergency_stop(terminate_nodes=False))
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
        bus.estop_state_changed.connect(lambda _s: self._refresh_mode_button())

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
               "running": "ok", "stopping": "warn"}[state]
        self._launch_led.set_status(led)
        self.launch_button.setEnabled(state == "idle")
        self.stop_button.setEnabled(state in ("starting", "running"))

    def _on_launch_output(self, line: str) -> None:
        self._console.setText(line[-160:])

    def _on_estop_state(self, state: str) -> None:
        text = {"publishing": "safe-shutdown: publishing 'default'…",
                "confirmed": "safe-shutdown: confirmed by param_mode echo",
                "timeout": "safe-shutdown: published, flushed (no echo — check chain)",
                "sim": "safe-shutdown: simulation — ack skipped",
                "idle": ""}.get(state, "")
        self._estop_label.setText(text)
