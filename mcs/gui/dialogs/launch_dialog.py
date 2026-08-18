"""Mission launch configuration dialog.

Two mission types, matching the two launch files of the stack:

* **Real robot** — ``BlueBoat_launch.py`` with ``enable_motors``, ``note``,
  ``controller_type`` (may be empty: robot interface only), ``trajectory``
  (disabled when ``use_pinger`` is set, since the launch file then skips
  ``path_generation``) and ``use_pinger``.
* **Gazebo simulation** — ``Sim_launch.py`` with ``robot_file``,
  ``trajectory`` and ``controller_type`` only. The simulation launch always
  starts ``master_control``, so an empty controller is not offered; motors /
  note / pinger do not exist in that graph and their fields are hidden to
  keep the dialog coherent with what will actually run.

The dialog returns a :class:`~mcs.ros.launch_manager.LaunchParameters`;
``to_cli`` then emits exactly the arguments the chosen launch file declares.
"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtWidgets import (
    QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFormLayout, QLabel,
    QLineEdit, QVBoxLayout,
)

from mcs.config.settings import AppConfig
from mcs.designer import io_yaml
from mcs.gui import theme
from mcs.ros.launch_manager import LaunchParameters

_REAL = "Real robot"
_SIM = "Gazebo simulation"


def _list_custom_paths(cfg: AppConfig) -> list[tuple[str, Path]]:
    """Saved designer missions: (name, runtime yaml path), sorted."""
    directory = Path(cfg.designer.trajectories_dir)
    if not directory.exists():
        return []
    return sorted((p.stem, p) for p in directory.glob("*.yaml")
                  if not p.name.endswith(".meta.yaml"))


class LaunchDialog(QDialog):
    """Modal dialog collecting the launch parameters."""

    def __init__(self, cfg: AppConfig, last: LaunchParameters | None,
                 parent=None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self.setWindowTitle("Launch Mission — configuration")
        self.setMinimumWidth(440)
        layout = QVBoxLayout(self)

        form = QFormLayout()
        layout.addLayout(form)

        # ---- Mission type ----------------------------------------------------
        self._mode = QComboBox()
        self._mode.addItems([_REAL, _SIM])
        self._mode.setToolTip(
            f"{_REAL}: ros2 launch {cfg.launch.package} {cfg.launch.launch_file}\n"
            f"{_SIM}: ros2 launch {cfg.launch.package} {cfg.launch.sim_launch_file}")
        self._mode.currentTextChanged.connect(self._on_mode_changed)
        form.addRow("Mission type", self._mode)

        # ---- Common fields -----------------------------------------------------
        self._controller = QComboBox()
        form.addRow("Controller type", self._controller)

        self._trajectory = QComboBox()
        self._trajectory.addItems(cfg.launch.trajectories)
        # Designer-generated missions appear under a 'custom paths' section;
        # each entry carries the runtime YAML path as item data, turned into
        # trajectory:=from_yaml:<path> by parameters(). Works identically on
        # the real robot and in simulation (path_generation runs in both).
        customs = _list_custom_paths(cfg)
        if customs:
            self._trajectory.insertSeparator(self._trajectory.count())
            header = self._trajectory.count()
            self._trajectory.addItem("── custom paths ──")
            self._trajectory.model().item(header).setEnabled(False)
            for name, path in customs:
                anchored = io_yaml.read_geo_anchor(path) is not None
                label = f"custom: {name}" + (" (GPS)" if anchored else "")
                self._trajectory.addItem(label, str(path))
        form.addRow("Trajectory", self._trajectory)

        # ---- Real-robot-only fields ----------------------------------------------
        self._use_pinger = QCheckBox("Steer towards the USBL pinger (no path)")
        self._use_pinger.toggled.connect(self._on_pinger_toggled)
        self._use_pinger_label = QLabel("Use pinger")
        form.addRow(self._use_pinger_label, self._use_pinger)

        self._enable_motors = QCheckBox("ENABLE MOTORS — real thrust will be applied")
        self._enable_motors.setStyleSheet(f"color: {theme.WARN}; font-weight: bold;")
        self._motors_label = QLabel("Motors")
        form.addRow(self._motors_label, self._enable_motors)

        self._note = QLineEdit()
        self._note.setPlaceholderText("appended to robot-side log file names")
        self._note_label = QLabel("Log note")
        form.addRow(self._note_label, self._note)

        # ---- Simulation-only fields -------------------------------------------------
        self._robot_file = QComboBox()
        self._robot_file.setEditable(True)  # forward-compatible with new models
        self._robot_file.addItems(cfg.launch.sim_robot_files)
        self._robot_file_label = QLabel("Robot file")
        form.addRow(self._robot_file_label, self._robot_file)

        # ---- Free-form extras ------------------------------------------------------
        self._extra = QLineEdit()
        self._extra.setPlaceholderText("extra launch args, e.g. foo:=bar baz:=1")
        form.addRow("Extra args", self._extra)

        self._hint = QLabel()
        self._hint.setStyleSheet(f"color: {theme.TEXT_DIM}; font-size: 10px;")
        self._hint.setWordWrap(True)
        layout.addWidget(self._hint)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        buttons.button(QDialogButtonBox.StandardButton.Ok).setText("Launch")
        layout.addWidget(buttons)

        # ---- Restore last parameters ------------------------------------------------
        if last is not None:
            self._mode.setCurrentText(_SIM if last.simulation else _REAL)
            if last.trajectory.startswith("from_yaml:"):
                idx = self._trajectory.findData(
                    last.trajectory.partition(":")[2])
                if idx >= 0:
                    self._trajectory.setCurrentIndex(idx)
            else:
                self._trajectory.setCurrentText(last.trajectory)
            self._use_pinger.setChecked(last.use_pinger)
            self._enable_motors.setChecked(False)  # always re-confirm motors
            self._note.setText(last.note)
            self._robot_file.setCurrentText(last.robot_file)
        self._on_mode_changed(self._mode.currentText())
        if last is not None and last.controller_type in self._items(self._controller):
            self._controller.setCurrentText(last.controller_type)

    # ------------------------------------------------------------------ modes
    def _on_mode_changed(self, mode: str) -> None:
        sim = mode == _SIM
        # Controller list: Sim_launch always starts master_control, so the
        # empty "no controller" option only exists on the real robot.
        current = self._controller.currentText()
        self._controller.clear()
        if sim:
            controllers = [c for c in self._cfg.launch.controllers if c]
            self._controller.addItems(controllers)
            default = (current if current in controllers
                       else self._cfg.launch.sim_default_controller)
            self._controller.setCurrentText(default)
            self._controller.setToolTip(
                "Sim_launch.py always starts master_control; a controller is "
                "required.")
        else:
            self._controller.addItems(self._cfg.launch.controllers)
            self._controller.setCurrentText(current)
            self._controller.setToolTip(
                "Empty = robot interface only (manual 'move' commands, no "
                "controller node).")
        # Field gating
        for w in (self._use_pinger, self._use_pinger_label,
                  self._enable_motors, self._motors_label,
                  self._note, self._note_label):
            w.setVisible(not sim)
        for w in (self._robot_file, self._robot_file_label):
            w.setVisible(sim)
        self._trajectory.setEnabled(sim or not self._use_pinger.isChecked())
        launch_file = (self._cfg.launch.sim_launch_file if sim
                       else self._cfg.launch.launch_file)
        extra = ("Simulation graph: Gazebo world + simulation_interface — no "
                 "MAVROS, no pinger, no param_set (shutdown acknowledgement "
                 "is skipped accordingly)." if sim else
                 "Mission controls unlock once the required nodes report ready.")
        self._hint.setText(
            f"The station runs: ros2 launch {self._cfg.launch.package} "
            f"{launch_file} …\n{extra}")
        self.adjustSize()

    def _on_pinger_toggled(self, on: bool) -> None:
        # BlueBoat_launch.py only starts path_generation when use_pinger is False.
        if self._mode.currentText() == _REAL:
            self._trajectory.setEnabled(not on)

    @staticmethod
    def _items(combo: QComboBox) -> list[str]:
        return [combo.itemText(i) for i in range(combo.count())]

    # ------------------------------------------------------------------ result
    def parameters(self) -> LaunchParameters:
        sim = self._mode.currentText() == _SIM
        extra: dict[str, str] = {}
        for token in self._extra.text().split():
            if ":=" in token:
                key, value = token.split(":=", 1)
                extra[key] = value
        custom = self._trajectory.currentData()
        gps_src = gps_dst = ""
        if custom:
            src = Path(custom)
            # Deferred GPS deployment only applies to the REAL robot, whose
            # world origin changes every run. In simulation there is no GPS
            # (the georeference can never converge, so the deployed file
            # would never be written and the boat would hold at (0,0)
            # forever) and Gazebo's world frame is stable — so an anchored
            # mission executes its design-frame points directly.
            if (not sim) and io_yaml.read_geo_anchor(src) is not None:
                gps_src = str(src)
                gps_dst = str(io_yaml.deployed_path(src.parent, src.stem))
                trajectory = f"from_yaml:{gps_dst}"
            else:
                trajectory = f"from_yaml:{custom}"
        else:
            trajectory = self._trajectory.currentText()
        return LaunchParameters(
            enable_motors=(not sim) and self._enable_motors.isChecked(),
            note="" if sim else self._note.text().strip(),
            controller_type=self._controller.currentText(),
            trajectory=trajectory,
            use_pinger=(not sim) and self._use_pinger.isChecked(),
            simulation=sim,
            robot_file=self._robot_file.currentText().strip() or "thrusters_ur",
            gps_anchored_source=gps_src,
            gps_deployed_target=gps_dst,
            extra_args=extra,
        )
