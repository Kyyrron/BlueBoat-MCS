"""Mission launch configuration dialog.

Two mission types, matching the two launch files of the stack:

* **Real robot** — ``BlueBoat_launch.py`` as a **run session**
  (``session_mode:=True``, since 2026-10-09): the launch connects and idles,
  and runs are started and stopped from the station. The dialog chooses what
  the FIRST run executes — a designer path (built-in shapes are a simulation
  and CLI affair: a session places or deploys each run's path into one watched
  file, which only designer files can feed) or the USBL pinger — plus
  ``controller_type`` (PID / LoS / MPC: manual target and arrow keys go
  through ``master_control``, so a session always has one; MPC cannot follow
  the pinger), ``enable_motors`` and the log note. Update Mission changes all
  of that between runs (``mission_update_dialog.py``).
* **Gazebo simulation** — ``Sim_launch.py`` with ``robot_file``,
  ``trajectory``, ``controller_type``, ``note`` (and ``spawn_yaw`` for
  GPS-anchored missions). The simulation launch always starts
  ``master_control``, so an empty controller is not offered; motors and
  pinger do not exist in that graph and their fields are hidden to keep the
  dialog coherent with what will actually run. The **log note** does exist
  there — ``simulation_interface`` names its poslog CSV from it exactly as
  ``robot_interface`` does in a real-robot run — so that field stays visible in both
  modes; a simulated run's note always carries the ``sim`` marker
  (:meth:`LaunchParameters.wire_note`). A GPS-anchored custom path takes the
  SAME deferred-deploy pipeline as on real water: the station synthesises the
  GPS feed itself (first fix lands north of the path's first point) and spawns
  the boat at a **fixed** heading (``LaunchConfig.sim_spawn_yaw_deg``), so
  anchoring, deployment and hot-reload are exercised exactly as at sea and two
  runs of the same mission remain comparable.

  **Source** (Gazebo only, v1.1): *Controller* is the above; *From a
  ros2bag* replays a recorded real-robot run instead (``Replay_launch.py``):
  the operator picks a run folder (or its ``*-bag`` directory) and an initial
  playback rate, and the boat moves exactly as it did in the water — no
  controller, trajectory, robot file or note apply, and no sea state is asked
  for (the recording already holds the real sea).

The dialog returns a :class:`~mcs.ros.launch_manager.LaunchParameters`;
``to_cli`` then emits exactly the arguments the chosen launch file declares.
"""

from __future__ import annotations

import math
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core import replay as replay_core
from mcs.designer import io_yaml
from mcs.gui import theme
from mcs.ros.launch_manager import LaunchParameters

_REAL = "Real robot"
_SIM = "Gazebo simulation"
_SRC_CONTROL = "Controller"
_SRC_BAG = "From a ros2bag"


def list_custom_paths(cfg: AppConfig) -> list[tuple[str, Path]]:
    """Saved designer missions: (name, runtime yaml path), sorted.

    Read from disk on every call, so a path saved in the designer while a
    session is connected is offered by the next dialog that opens."""
    directory = Path(cfg.designer.trajectories_dir)
    if not directory.exists():
        return []
    return sorted((p.stem, p) for p in directory.glob("*.yaml")
                  if not p.name.endswith(".meta.yaml"))


def custom_badge(cfg: AppConfig, path: Path, session: bool = False
                 ) -> tuple[str, str]:
    """Suffix + tooltip for one custom-path entry.

    A GPS-anchored mission is geographically fixed and is never
    realigned (the boat turns toward it instead), so it is exempt from
    the start-alignment check -- the same exemption
    DesignerWindow._maybe_offer_alignment makes. Everything else is
    checked against its own samples: the world frame is local ENU
    (origin = launch point, +x = east), so a mission that does not
    start at (0,0) along +x makes the boat cut across to its start.

    In a real-robot run session (``session``) a non-anchored mission is
    PLACED at the boat when the run starts (first sample on the boat, first
    heading along its heading), so start alignment no longer matters there.

    Informational only: the launch path never rewrites a mission file.
    """
    if io_yaml.read_geo_anchor(path) is not None:
        return (" (GPS)",
                ("GPS-anchored: relocated into this run's world frame at "
                 "launch."))
    if session:
        return ("", ("Placed at the boat when the run starts: the path "
                     "begins at the boat and runs out ahead of it."))
    if io_yaml.read_start_misalignment(
            path, cfg.designer.start_align_tol_m,
            cfg.designer.start_align_tol_deg) is not None:
        return (" (not start-aligned)",
                ("This mission does not start at (0,0) heading east "
                 "(+x). The world frame is local ENU with its origin at "
                 "the launch point, so the boat will first cut across "
                 "to the mission start.\n"
                 "For real water, prefer a GPS-anchored mission; "
                 "otherwise use Edit \u25b8 Align to Start in the Survey "
                 "Pattern Designer."))
    return ("", "")


class LaunchDialog(QDialog):
    """Modal dialog collecting the launch parameters."""

    def __init__(self, cfg: AppConfig, last: LaunchParameters | None,
                 parent=None) -> None:
        super().__init__(parent)
        self._cfg = cfg
        self.setWindowTitle("Launch Mission — configuration")
        self.setMinimumWidth(440)
        layout = QVBoxLayout(self)

        form = self._form = QFormLayout()
        layout.addLayout(form)

        # ---- Mission type ----------------------------------------------------
        self._mode = QComboBox()
        self._mode.addItems([_REAL, _SIM])
        self._mode.setToolTip(
            f"{_REAL}: ros2 launch {cfg.launch.package} {cfg.launch.launch_file}\n"
            f"{_SIM}: ros2 launch {cfg.launch.package} {cfg.launch.sim_launch_file}")
        self._mode.currentTextChanged.connect(self._on_mode_changed)
        form.addRow("Mission type", self._mode)

        # ---- Gazebo source: live controller, or a recorded run (v1.1) ---------
        self._source = QComboBox()
        self._source.addItems([_SRC_CONTROL, _SRC_BAG])
        self._source.setToolTip(
            f"{_SRC_CONTROL}: the simulated boat is driven by master_control.\n"
            f"{_SRC_BAG}: the boat replays a recorded real-robot run exactly "
            f"(ros2 launch {cfg.launch.package} {cfg.launch.replay_launch_file}).")
        self._source.currentTextChanged.connect(
            lambda _t: self._on_mode_changed(self._mode.currentText()))
        self._source_label = QLabel("Source")
        form.addRow(self._source_label, self._source)

        bag_row = QWidget()
        bag_layout = QHBoxLayout(bag_row)
        bag_layout.setContentsMargins(0, 0, 0, 0)
        self._bag = QLineEdit()
        self._bag.setPlaceholderText("a run folder of Robot_data/, or its *-bag folder")
        self._bag.textChanged.connect(self._on_bag_changed)
        browse = QPushButton("Browse…")
        browse.clicked.connect(self._browse_bag)
        bag_layout.addWidget(self._bag, 1)
        bag_layout.addWidget(browse)
        self._bag_row = bag_row
        self._bag_label = QLabel("Recorded run")
        form.addRow(self._bag_label, bag_row)
        self._bag_info = QLabel()
        self._bag_info.setWordWrap(True)
        self._bag_info.setStyleSheet(f"color: {theme.TEXT_DIM}; font-size: 10px;")
        self._bag_info_label = QLabel("")
        form.addRow(self._bag_info_label, self._bag_info)

        self._rate = QComboBox()
        for r in cfg.replay.rates:
            self._rate.addItem(f"{r:g}×", float(r))
        idx = self._rate.findData(float(cfg.replay.default_rate))
        self._rate.setCurrentIndex(max(0, idx))
        self._rate.setToolTip("Initial playback rate; it can be changed, and the "
                              "replay paused, from the REPLAY box while it runs.")
        self._rate_label = QLabel("Playback rate")
        form.addRow(self._rate_label, self._rate)

        # ---- Common fields -----------------------------------------------------
        self._controller = QComboBox()
        form.addRow("Controller type", self._controller)

        # Filled per mission type by _fill_trajectories: in simulation the
        # built-in shapes, then designer-generated missions under a 'custom
        # paths' section; on the real robot (a run session) designer missions
        # only. Each custom entry carries the runtime YAML path as item data.
        self._trajectory = QComboBox()
        self._customs = list_custom_paths(cfg)
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
        self._note.setPlaceholderText("added to the run's log file names")
        self._note.setToolTip(
            "Free text put into the poslog CSV name, {date}-{note}-poslog.csv "
            "— robot_interface in a real-robot run, simulation_interface in "
            "Gazebo. A "
            "simulated run is always tagged 'sim' (sim-<note>).")
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
        self._fill_trajectories(sim=False)
        if last is not None:
            self._mode.setCurrentText(_SIM if last.simulation else _REAL)
            self._fill_trajectories(sim=last.simulation)
            if last.session:
                restore = last.gps_anchored_source or last.session_source
                idx = self._trajectory.findData(restore) if restore else -1
                if idx >= 0:
                    self._trajectory.setCurrentIndex(idx)
            elif last.trajectory.startswith("from_yaml:"):
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
            if last.replay_bag:
                self._source.setCurrentText(_SRC_BAG)
                self._bag.setText(last.replay_bag)
                idx = self._rate.findData(float(last.replay_rate))
                if idx >= 0:
                    self._rate.setCurrentIndex(idx)
        self._on_mode_changed(self._mode.currentText())
        if last is not None and last.controller_type in self._items(self._controller):
            self._controller.setCurrentText(last.controller_type)

    # ------------------------------------------------------------------ modes
    def _fill_trajectories(self, sim: bool) -> None:
        """Trajectory list for one mission type, keeping the selection when
        it still exists. Simulation: built-in shapes, then designer paths.
        Real robot (run session): designer paths only."""
        combo = self._trajectory
        keep_data, keep_text = combo.currentData(), combo.currentText()
        combo.blockSignals(True)
        combo.clear()
        if sim:
            combo.addItems(self._cfg.launch.trajectories)
            if self._customs:
                combo.insertSeparator(combo.count())
                header = combo.count()
                combo.addItem("── custom paths ──")
                combo.model().item(header).setEnabled(False)
        for name, path in self._customs:
            badge, tip = custom_badge(self._cfg, path, session=not sim)
            combo.addItem(f"{'custom: ' if sim else ''}{name}{badge}", str(path))
            if tip:
                combo.setItemData(combo.count() - 1, tip,
                                  Qt.ItemDataRole.ToolTipRole)
        if not sim and not self._customs:
            combo.addItem("(no designer path yet — create one, or follow "
                          "the pinger)")
            combo.model().item(0).setEnabled(False)
        idx = combo.findData(keep_data) if keep_data else combo.findText(keep_text)
        if idx >= 0:
            combo.setCurrentIndex(idx)
        combo.blockSignals(False)

    def _replaying(self) -> bool:
        return (self._mode.currentText() == _SIM
                and self._source.currentText() == _SRC_BAG)

    def _on_mode_changed(self, mode: str) -> None:
        sim = mode == _SIM
        self._fill_trajectories(sim)
        # Controller list: both launches need master_control now (a real-robot
        # run session drives manual targets and arrow keys through it), so the
        # empty "no controller" entry is offered in neither.
        current = self._controller.currentText()
        self._controller.clear()
        controllers = [c for c in self._cfg.launch.controllers if c]
        self._controller.addItems(controllers)
        if sim:
            default = (current if current in controllers
                       else self._cfg.launch.sim_default_controller)
            self._controller.setCurrentText(default)
            self._controller.setToolTip(
                "Sim_launch.py always starts master_control; a controller is "
                "required.")
        else:
            self._controller.setCurrentText(current if current in controllers
                                            else "PID")
            self._controller.setToolTip(
                "The controller of the first run; Update Mission can change "
                "it between runs. MPC cannot follow the pinger.")
        self._gate_mpc()
        # Field gating
        for w in (self._use_pinger, self._use_pinger_label,
                  self._enable_motors, self._motors_label):
            w.setVisible(not sim)
        for w in (self._robot_file, self._robot_file_label):
            w.setVisible(sim)
        self._trajectory.setEnabled(sim or not self._use_pinger.isChecked())
        # Gazebo source: a replay takes no controller, path, robot or note.
        replay = self._replaying()
        for w in (self._source, self._source_label):
            w.setVisible(sim)
        for w in (self._bag_row, self._bag_label, self._bag_info,
                  self._bag_info_label, self._rate, self._rate_label):
            w.setVisible(replay)
        for w in (self._controller, self._trajectory, self._note):
            self._set_row_visible(w, not replay)
        self._set_row_visible(self._robot_file, sim and not replay)
        launch_file = (self._cfg.launch.sim_launch_file if sim
                       else self._cfg.launch.launch_file)
        if replay:
            launch_file = self._cfg.launch.replay_launch_file
        extra = ("Replay: the boat follows the recording exactly (pose, "
                 "attitude, GPS, speed); targets, thrust and pinger are shown "
                 "as recorded. No controller and no sea state — wind, current "
                 "and waves cannot move it." if replay else
                 "Simulation graph: Gazebo world + simulation_interface — no "
                 "MAVROS, no pinger, no param_set (shutdown acknowledgement "
                 "is skipped accordingly)." if sim else
                 "Run session: the boat connects and goes into override but "
                 "does NOT move or log until you press START RUN (left panel). "
                 "Drive it meanwhile with Manual Target or the arrow keys.")
        self._hint.setText(
            f"The station runs: ros2 launch {self._cfg.launch.package} "
            f"{launch_file} …\n{extra}")
        self.adjustSize()

    def _on_pinger_toggled(self, on: bool) -> None:
        # Following the pinger, the path list means nothing.
        if self._mode.currentText() == _REAL:
            self._trajectory.setEnabled(not on)
        self._gate_mpc()

    def _gate_mpc(self) -> None:
        """MPC cannot follow the pinger (master_control's pinger branch is
        PID / LoS only): grey it out, and leave it if it was selected."""
        pinger = (self._mode.currentText() == _REAL
                  and self._use_pinger.isChecked())
        idx = self._controller.findText("MPC")
        if idx >= 0:
            self._controller.model().item(idx).setEnabled(not pinger)
            if pinger and self._controller.currentText() == "MPC":
                self._controller.setCurrentText("PID")

    def _set_row_visible(self, field: QWidget, on: bool) -> None:
        field.setVisible(on)
        label = self._form.labelForField(field)
        if label is not None:
            label.setVisible(on)

    # ------------------------------------------------------------- recorded run
    def _browse_bag(self) -> None:
        start = self._bag.text().strip() or self._cfg.replay.bag_root
        folder = QFileDialog.getExistingDirectory(
            self, "Recorded run (a Robot_data run folder or its bag)", start)
        if folder:
            self._bag.setText(folder)

    def _on_bag_changed(self, text: str) -> None:
        bag = replay_core.find_bag(text.strip()) if text.strip() else None
        summary = replay_core.bag_summary(bag) if bag else None
        if not text.strip():
            self._bag_info.setText("Every real-robot run since v1.1 has a bag "
                                   "in its Robot_data folder.")
        elif summary is None:
            self._bag_info.setText("No ros2bag here (no metadata.yaml in the "
                                   "folder or one level below).")
        else:
            self._bag_info.setText(summary.describe())

    def bag_problem(self) -> str:
        """Why the chosen recorded run cannot be replayed, or ""."""
        if not self._replaying():
            return ""
        bag = replay_core.find_bag(self._bag.text().strip())
        if bag is None:
            return "Choose a recorded run: no ros2bag was found in that folder."
        summary = replay_core.bag_summary(bag)
        if summary is None:
            return f"{bag}: unreadable bag metadata."
        if not summary.has_pose:
            return (f"{bag.name} holds neither /mavros/local_position/odom nor "
                    "/blueboat/odom: there is no motion to replay.")
        return ""

    def accept(self) -> None:
        problem = self.bag_problem()
        if problem:
            QMessageBox.warning(self, "Replay", problem)
            return
        super().accept()

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
        if self._replaying():
            bag = replay_core.find_bag(self._bag.text().strip())
            return LaunchParameters(
                simulation=True,
                controller_type="",
                trajectory="",
                replay_bag=str(bag) if bag else self._bag.text().strip(),
                replay_rate=float(self._rate.currentData() or 1.0),
                extra_args=extra,
            )
        custom = self._trajectory.currentData()
        gps_src = gps_dst = ""
        gps_simulated = False
        spawn_yaw = None
        if not sim:
            # Real robot = run session: path_generation always watches the
            # session file; the station writes each run's path into it.
            pinger = self._use_pinger.isChecked()
            source = "" if (pinger or not custom) else str(custom)
            session_file = self._cfg.launch.session_trajectory_file
            if source and io_yaml.read_geo_anchor(Path(source)) is not None:
                gps_src, gps_dst = source, session_file
            return LaunchParameters(
                enable_motors=self._enable_motors.isChecked(),
                note=self._note.text().strip(),
                controller_type=self._controller.currentText() or "PID",
                trajectory=f"from_yaml:{session_file}",
                use_pinger=pinger,
                simulation=False,
                robot_file=self._robot_file.currentText().strip() or "thrusters_ur",
                gps_anchored_source=gps_src,
                gps_deployed_target=gps_dst,
                session=True,
                session_source=source,
                extra_args=extra,
            )
        if custom:
            src = Path(custom)
            if io_yaml.read_geo_anchor(src) is not None:
                # GPS-anchored: the SAME deferred-deploy pipeline on real
                # water and in simulation. On real water MAVROS provides the
                # fixes; in simulation the station synthesises them (the
                # boat's first fix lands offset_north_m north of the path's
                # first point) and spawns the boat at the configured fixed
                # heading, so anchoring, deployment and hot-reload run exactly
                # as at sea, repeatably.
                gps_src = str(src)
                gps_dst = str(io_yaml.deployed_path(src.parent, src.stem))
                trajectory = f"from_yaml:{gps_dst}"
                if sim:
                    gps_simulated = True
                    # Fixed, not random: the spawn heading is the only spawn
                    # degree of freedom here, and two runs of one mission must
                    # start from the same pose to be comparable. Extra args
                    # may still override it for a single run.
                    spawn_yaw = math.radians(
                        self._cfg.launch.sim_spawn_yaw_deg)
            else:
                trajectory = f"from_yaml:{custom}"
        else:
            trajectory = self._trajectory.currentText()
        return LaunchParameters(
            enable_motors=(not sim) and self._enable_motors.isChecked(),
            note=self._note.text().strip(),
            controller_type=self._controller.currentText(),
            trajectory=trajectory,
            use_pinger=(not sim) and self._use_pinger.isChecked(),
            simulation=sim,
            robot_file=self._robot_file.currentText().strip() or "thrusters_ur",
            gps_anchored_source=gps_src,
            gps_deployed_target=gps_dst,
            gps_simulated=gps_simulated,
            spawn_yaw_rad=spawn_yaw,
            extra_args=extra,
        )
