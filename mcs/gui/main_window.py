"""Main window — composition root of the GUI.

Wires together the data store, the signal bus, the panels, the map and the
toolbar.  Owns the single 10 Hz refresh timer that drives all repainting
(messages update the store immediately; widgets repaint on the tick — this
is what keeps the UI smooth and flicker-free during long experiments).

A real-robot launch is a **run session** (:mod:`mcs.ros.run_session`): the
RUN box under the left panel starts and stops runs, the toolbar's Launch /
Stop buttons become Update Mission / Turn Off Connection, and the floating
ARROW CONTROL box (map, bottom-right) drives the boat from the keyboard.
"""

from __future__ import annotations

import logging
import math
from pathlib import Path

from PySide6.QtCore import QPoint, Qt, QTimer
from PySide6.QtGui import QCloseEvent
from PySide6.QtWidgets import (
    QLabel,
    QMainWindow,
    QMessageBox,
    QSplitter,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)

from mcs.config.settings import AppConfig
from mcs.core.signals import SignalBus
from mcs.gui import theme
from mcs.gui.arrow_box import FloatingArrowBox
from mcs.gui.bottom_toolbar import BottomToolbar
from mcs.gui.guardian_box import GuardianBox, ask_turn_off_choice
from mcs.gui.left_panel import LeftPanel
from mcs.gui.map.map_view import MapMode, MapView
from mcs.gui.mission_stats import FloatingStatsBox
from mcs.gui.replay_box import FloatingReplayBox
from mcs.gui.right_panel import RightPanel
from mcs.gui.run_panel import RunPanel
from mcs.gui.sea_state_box import FloatingSeaBox
from mcs.models.store import DataStore
from mcs.ros.command_center import CommandCenter
from mcs.ros.launch_manager import LaunchManager, LaunchParameters
from mcs.ros.ros_manager import RosManager
from mcs.ros.run_session import RunSession

_LOG = logging.getLogger(__name__)


class MainWindow(QMainWindow):
    """The BlueBoat Mission Control Station main window."""

    def __init__(self, cfg: AppConfig) -> None:
        super().__init__()
        self.setWindowTitle("BlueBoat Mission Control Station")
        self.resize(1500, 900)
        self._exit_in_progress = False
        self._exit_finalized = False
        self._exit_deadline = 0.0

        # ---- Core objects ----------------------------------------------------
        self.cfg = cfg
        self.bus = SignalBus()
        self.store = DataStore(cfg)
        self.ros = RosManager(cfg, self.bus)
        self.launcher = LaunchManager(cfg, self.bus)
        self.commands = CommandCenter(cfg, self.bus, self.ros, self.launcher)
        self.run_session = RunSession(cfg, self.bus, self.store, self.commands)

        # ---- Widgets -----------------------------------------------------------
        self.map_view = MapView(cfg, self.store)
        self.left_panel = LeftPanel(cfg, self.store)
        self.right_panel = RightPanel(cfg, self.store)
        self.left_panel.setMinimumWidth(340)
        self.left_panel.setMaximumWidth(450)

        self.right_panel.setMinimumWidth(340)
        self.right_panel.setMaximumWidth(600)
        self.toolbar = BottomToolbar(cfg, self.bus, self.launcher, self.commands)
        # RUN box: pinned under the left panel's scroll area (below ROS
        # DIAGNOSTICS, never scrolled away); visible during a run session.
        self.run_panel = RunPanel(self.store, self.run_session)
        self.left_panel.set_footer(self.run_panel)
        # GUARDIAN box (v1.1): the link guardian, at the top of the left
        # column; active during a real-robot session.
        self.guardian_box = GuardianBox()
        self.left_panel.set_header(self.guardian_box)
        self._guardian_ack_seq = None
        self.toolbar.set_stop_guard(self._guardian_stop_guard)

        splitter = QSplitter(Qt.Orientation.Horizontal)
        splitter.addWidget(self.left_panel)
        splitter.addWidget(self.map_view)
        splitter.addWidget(self.right_panel)
        splitter.setStretchFactor(0, 1)
        splitter.setStretchFactor(1, 4)
        splitter.setStretchFactor(2, 1)
        splitter.setSizes([320, 900, 320])

        # Floating Mission Stats Box (parented to the map_view so it floats without breaking the splitter)
        self.stats_box = FloatingStatsBox(self.store, self.map_view)
        self.right_panel.time_window_changed.connect(self.stats_box.refresh_stats)
        # Floating Sea State box — mirror of the stats box, glued to the
        # top-LEFT of the map view (i.e. right against the left panel).
        self.sea_box = FloatingSeaBox(self.store, self.map_view)
        # Bag replay (v1.1): same corner as the sea box, which a replay never
        # shows (no sea state in a replay).
        self.replay_box = FloatingReplayBox(cfg, self.store, self.map_view)
        # Arrow-key control: the bottom-RIGHT corner of the map view.
        self.arrow_box = FloatingArrowBox(cfg, self.store, self.commands,
                                          self.map_view)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(4, 4, 4, 0)
        layout.setSpacing(4)
        layout.addWidget(splitter, stretch=1)
        layout.addWidget(self.toolbar)
        self.setCentralWidget(central)

        self._status = QStatusBar()
        self.setStatusBar(self._status)
        self._inspect_label = QLabel("")
        self._inspect_label.setObjectName("valueLabel")
        self._status.addWidget(self._inspect_label, stretch=1)
        self._geo_label = QLabel("georef: —")
        self._status.addPermanentWidget(self._geo_label)
        self._ros_label = QLabel("ROS: starting…")
        self._status.addPermanentWidget(self._ros_label)

        self._connect_signals()

        # ---- Refresh tick -----------------------------------------------------
        period_ms = int(1000.0 / cfg.map.ui_refresh_hz)
        self._tick = QTimer(self)
        self._tick.timeout.connect(self._on_tick)
        self._tick.start(period_ms)

        # ---- Go ------------------------------------------------------------------
        self.ros.start()
        self._ros_label.setText(
            "ROS: connected" if self.ros.running else "ROS: unavailable")

    def resizeEvent(self, event):
        super().resizeEvent(event)

        width = self.width()

        if width < 1200:
            self.left_panel.setMaximumWidth(300)
            self.right_panel.setMaximumWidth(320)
        else:
            self.left_panel.setMaximumWidth(450)
            self.right_panel.setMaximumWidth(600)

    # ============================================================== signal wiring
    def _connect_signals(self) -> None:
        bus, store = self.bus, self.store

        # Telemetry → store (queued into the GUI thread by Qt)
        bus.odom_received.connect(store.on_odom)
        bus.gps_received.connect(store.on_gps)
        bus.compass_received.connect(store.on_compass)
        bus.mavros_state_received.connect(store.on_mavros_state)
        bus.battery_received.connect(store.on_battery)
        bus.pinger_body_received.connect(store.on_pinger_body)
        bus.uw_gps_raw_received.connect(store.on_uw_gps_raw)
        bus.monitoring_received.connect(store.on_monitoring)
        bus.thruster_received.connect(store.on_thruster)
        bus.controller_ready_received.connect(store.on_controller_ready)
        bus.param_mode_received.connect(store.on_param_mode)
        bus.sea_state_received.connect(store.on_sea_state)
        bus.replay_state_received.connect(store.on_replay_state)
        bus.guardian_status_received.connect(store.on_guardian_status)
        bus.mission_path_received.connect(store.on_mission_path)
        bus.mission_path_failed.connect(self._on_mission_path_failed)

        # Diagnostics / logs.
        # Each message is fanned out from ONE signal to BOTH sinks — the GUI
        # (operator-friendly) and the python logger (terminal = complete debug
        # output) — so the two can never desynchronize.
        bus.topic_stats_updated.connect(self.left_panel.on_topic_stats)
        bus.ros_log.connect(lambda msg: self._status.showMessage(msg, 8000))
        bus.ros_log.connect(lambda msg: _LOG.info("ros: %s", msg))
        bus.command_sent.connect(
            lambda desc: self._status.showMessage(f"Published: {desc}", 4000))
        bus.command_sent.connect(lambda desc: _LOG.info("published: %s", desc))
        bus.launch_state_changed.connect(
            lambda s: _LOG.info("launch state: %s", s))
        bus.estop_state_changed.connect(
            lambda s: _LOG.warning("safe-shutdown phase: %s", s)
            if s != "idle" else None)
        # Launch process output: dedicated GUI console (right panel) + DEBUG
        # level on the terminal (visible with --verbose), order-preserving in
        # both since one queued signal drives both slots.
        bus.launch_output.connect(self.right_panel.console.append_line)
        bus.launch_output.connect(lambda line: _LOG.debug("[launch] %s", line))

        # Panels ↔ map
        self.right_panel.layer_toggled.connect(self.map_view.set_layer_visible)
        self.right_panel.time_window_changed.connect(self.map_view.set_time_window)
        self.right_panel.clear_paths_requested.connect(self._on_clear_paths)
        self.right_panel.zoom_in_requested.connect(self.map_view.zoom_in)
        self.right_panel.zoom_out_requested.connect(self.map_view.zoom_out)
        self.right_panel.center_robot_requested.connect(self.map_view.center_on_robot)
        self.map_view.point_inspected.connect(self._inspect_label.setText)

        # Toolbar
        self.toolbar.manual_target_mode_changed.connect(self._on_manual_mode)
        self.toolbar.continue_mission_clicked.connect(self._on_continue_mission)
        self.toolbar.measure_mode_changed.connect(self._on_measure_mode)
        self.toolbar.create_pattern_clicked.connect(self._open_designer)
        self.toolbar.mission_launched.connect(self._on_mission_launched)
        self.sea_box.modify_sea_clicked.connect(self._on_modify_sea)
        self.replay_box.pause_requested.connect(
            lambda pause: self._send_replay("pause" if pause else "resume"))
        self.replay_box.rate_requested.connect(
            lambda rate: self._send_replay("rate", rate))
        self.toolbar.mission_stopped.connect(self._on_mission_stopped)
        bus.launch_state_changed.connect(self._on_launch_state)

        # Manual target clicks
        self.map_view.target_clicked.connect(self._on_target_clicked)

        # Run session (real-robot launches) and arrow control
        self.run_panel.start_clicked.connect(self.run_session.start_run)
        self.run_panel.stop_clicked.connect(self.run_session.stop_run)
        self.toolbar.update_mission_requested.connect(self._on_update_mission)
        self.run_session.message.connect(self._status.showMessage)
        self.run_session.message.connect(lambda msg, _ms: _LOG.info("run session: %s", msg))
        self.run_session.preview_requested.connect(self._on_session_preview)
        self.run_session.preview_cleared.connect(self._on_session_preview_cleared)
        self.run_session.deploy_requested.connect(self._arm_session_deployment)
        self.run_session.arrows_release_requested.connect(
            lambda: self.arrow_box.set_active(False, hold=False))
        self.run_session.manual_target_cleared.connect(self._clear_manual_target_ui)
        self.guardian_box.action_requested.connect(self._on_guardian_action)
        self.arrow_box.engaged.connect(self._on_arrows_engaged)
        self.arrow_box.released.connect(self._on_arrows_released)

    # ================================================================ tick
    def _on_tick(self) -> None:
        self.left_panel.refresh()
        self.right_panel.refresh()
        self.map_view.refresh()
        if (hasattr(self, "_pending_preview_trajectory") and self._pending_preview_trajectory is not None and self.store.map_frame_ready()):
            trajectory = self._pending_preview_trajectory
            self._pending_preview_trajectory = None
            self._request_path_preview(trajectory)

        # Keep floating stats box glued to the top-right of the map view 
        # (which inherently puts it glued to the top-left of the right panel)
        if hasattr(self, 'stats_box'):
            expected_x = self.map_view.width() - self.stats_box.width() - 8
            expected_y = 8
            if self.stats_box.pos().x() != expected_x or self.stats_box.pos().y() != expected_y:
                self.stats_box.move(expected_x, expected_y)
                self.stats_box.raise_()

        # Sea state box: the mirror position, glued to the top-left of the map
        # view. It hides itself outside a running simulation.
        if hasattr(self, 'sea_box'):
            self.sea_box.refresh()
            if self.sea_box.pos() != QPoint(8, 8):
                self.sea_box.move(8, 8)
                self.sea_box.raise_()
        self.replay_box.refresh()
        if not self.replay_box.isHidden() and self.replay_box.pos() != QPoint(8, 8):
            self.replay_box.move(8, 8)
            self.replay_box.raise_()
        self.toolbar.set_replay_ui(self.store.mission.replay
                                   and self.store.mission.launch_running)

        # Arrow control box: bottom-right of the map view, whenever a launch
        # with a controller runs (real session or Gazebo).
        m = self.store.mission
        self.arrow_box.set_available(m.launch_running and bool(m.controller_type))
        if not self.arrow_box.isHidden():
            pos = QPoint(self.map_view.width() - self.arrow_box.width() - 8,
                         self.map_view.height() - self.arrow_box.height() - 8)
            if self.arrow_box.pos() != pos:
                self.arrow_box.move(pos)
                self.arrow_box.raise_()

        self._refresh_guardian()

        # Run session: RUN box + session labels of the toolbar.
        ss = self.store.session
        self.run_panel.refresh()
        self.toolbar.set_session_ui(
            ss.active,
            can_update=self.run_session.can_update(),
            run_active=ss.run_active or ss.run_pending == "start",
            release_visible=ss.motors_enabled and self.commands.estop_latched)

        # Georeference status + satellite availability
        geo = self.store.geo
        if geo.fit is None:
            self._geo_label.setText("georef: waiting for GPS fix…")
        else:
            quality = "anchored" if geo.is_valid else "poor"
            color = theme.OK if geo.is_valid else theme.WARN
            self._geo_label.setText(
                f"georef: {quality} (rms {geo.fit.rms_m:.1f} m, "
                f"{geo.fit.n_pairs} fixes)")
            self._geo_label.setStyleSheet(f"color: {color};")
        self.right_panel.set_satellite_available(geo.is_valid)
        # Map orientation is handled inside MapView (QGC-style: north-up
        # fixed, the scene is ENU by construction, only the glyph rotates)
        # — nothing to drive from here.
        # Mission readiness → launch state promotion. The FCU-connected check
        # only applies to the real-robot graph; Sim_launch.py has no MAVROS.
        if self.launcher.state == "starting":
            fcu_ok = (self.store.robot.fcu_connected
                      or self.store.mission.simulation)
            if fcu_ok and self.store.robot.has_odom:
                self.launcher.notify_running()

    # ============================================================ mission events
    def _on_mission_launched(self, params: LaunchParameters) -> None:
        m = self.store.mission
        m.launch_running = True
        m.controller_type = params.controller_type
        m.use_pinger = params.use_pinger
        m.simulation = params.simulation
        m.gps_simulated = params.gps_simulated
        m.world_dir = params.world_dir
        m.sea_choice = params.sea
        m.replay = bool(params.replay_bag)
        m.manual_target = None
        self.commands.set_simulation_mode(params.simulation)
        self.store.reset_experiment()
        if params.session:
            # Real robot = run session: connect, idle, runs by hand.
            self.run_session.begin(params)
            self._status.showMessage(
                "Run session: the boat connects and goes into override but does "
                "not move or log until START RUN. Drive it with Manual Target "
                "or the arrow keys meanwhile.", 12000)
        # Every launch restarts the control stack, which latches a NEW world
        # origin — the previous run's odom<->GPS pairs are wrong by
        # construction, so the anchor must start fresh (real and sim alike).
        self.store.reset_georeference()
        # Personalized-world runs (world_dir set) get their GPS from the
        # simulator's mavros shim, anchored on the world's own origin —
        # arming the station's SimGps too would put a second publisher on
        # the same topic with an arbitrary receiver origin and deploy the
        # trajectory into the wrong frame relative to the world geometry.
        if params.replay_bag:
            # A replay publishes the RECORDED fixes: the only GPS publisher.
            self.commands.disarm_sim_gps()
            self._status.showMessage(
                "Replay of a recorded run: the boat follows the recording "
                "(pose, attitude, GPS, speed); pause or change the rate in the "
                "REPLAY box.", 12000)
        elif params.gps_simulated and not params.world_dir:
            self._arm_sim_gps(params)
        else:
            self.commands.disarm_sim_gps()
            if params.world_dir:
                self._status.showMessage(
                    "Personalized world: GPS fixes come from the simulator's "
                    "mavros shim — the path deploys into the world's own "
                    "frame.", 10000)
        # Mission-path preview: the station asks the path_generation SERVICE
        # (/path_request) for the whole path — the same call path_publisher
        # makes, but direct, so it works identically on the real robot and
        # in simulation (path_publisher is only launched by Sim_launch.py).
        # Sim_launch.py always starts path_generation; the real launch only
        # does so with a controller and use_pinger:=False.

        self._preview_retries_left = self.cfg.launch.path_request_max_retries
        self._last_preview_trajectory = None
        # For a mission with deferred GPS deployment, params.trajectory points
        # at the DEPLOYED file, which _start_gps_deployment is about to unlink
        # — requesting it now would preview a stale/absent file. The
        # deployment poll requests the preview itself once the file is
        # written; everything else previews via the pending/tick mechanism
        # (gated on map_frame_ready).
        deployment_pending = bool(params.gps_anchored_source) and not (
            params.simulation and not params.gps_simulated)
        # A run session previews nothing up front: an anchored first mission
        # is previewed by the deployment poll, a placed one only once its run
        # starts (RunSession.preview_requested).
        # A replay has no path server: its path (the run's -path.yaml
        # snapshot) is published on /set_path by bag_replay instead.
        self._pending_preview_trajectory = (
            None if (deployment_pending or params.session or params.replay_bag)
            else params.trajectory)
        self._start_gps_deployment(params)

    # ========================================================= simulated GPS
    def _arm_sim_gps(self, params: LaunchParameters) -> None:
        """Simulated GPS for a Gazebo run of a GPS-anchored mission.

        Receiver origin = the mission's first point + ``offset_north_m``
        NORTH, in the anchor's EN frame (a legacy ``theta_deg`` anchor is
        rotated into EN with exactly ``io_yaml.deploy_mission``'s
        convention). Arms the bridge's NavSatFix timer; the fixes then flow
        through the identical real-water pipeline (own subscription ->
        diagnostics -> store -> anchor -> deferred deploy). Never called for
        a personalized-world run: there the simulator's mavros shim is the
        one GPS publisher (see _on_mission_launched)."""
        from mcs.core.geo import local_en_to_latlon
        from mcs.core.sim_gps import SimGpsModel
        from mcs.designer import io_yaml

        src = Path(params.gps_anchored_source)
        anchor = io_yaml.read_geo_anchor(src)
        first = io_yaml.read_first_point(src)
        if anchor is None or first is None:
            _LOG.error("sim GPS not armed: %s has no geo_anchor/points", src)
            self._status.showMessage(
                f"Sim GPS NOT armed — {src.name} unreadable; the map will "
                "wait for a fix forever.", 10000)
            return
        theta = math.radians(float(anchor.get("theta_deg", 0.0)))
        c, s = math.cos(theta), math.sin(theta)
        e0 = c * first[0] + s * first[1]     # design -> EN, same rotation
        n0 = -s * first[0] + c * first[1]    # deploy_mission applies
        lat, lon = local_en_to_latlon(
            e0, n0 + self.cfg.sim_gps.offset_north_m,
            float(anchor["lat0"]), float(anchor["lon0"]))
        # Seeded: the fix noise feeds the anchor estimate, so an unseeded
        # generator would deploy the same mission a little differently every
        # run (see SimGpsConfig.noise_seed).
        self.commands.arm_sim_gps(
            SimGpsModel(lat, lon, self.cfg.sim_gps.noise_sigma_m,
                        self.cfg.sim_gps.noise_seed))
        yaw_deg = math.degrees(params.spawn_yaw_rad or 0.0)
        self._status.showMessage(
            f"Sim GPS armed: first fix at {lat:.6f}, {lon:.6f} "
            f"({self.cfg.sim_gps.offset_north_m:.0f} m N of the path start); "
            f"spawn heading {yaw_deg:+.0f}°.", 12000)
        _LOG.info("sim GPS armed: origin=(%.6f, %.6f), spawn_yaw=%.1f deg",
                  lat, lon, yaw_deg)

    # ======================================================== GPS deployment
    def _start_gps_deployment(self, params: LaunchParameters) -> None:
        """GPS-anchored mission: path_generation was pointed at a deployed
        file that does not exist yet (it holds position meanwhile). The
        robot's world origin is created at power-on; the odom↔GPS anchor is
        a pure translation (the world frame is local ENU) and converges from
        the first few GPS fixes with NO vehicle motion required — so this
        watcher typically fires within seconds: it converts the anchored
        mission into today's world frame and writes the deployed file;
        path_generation reloads it on its next path request and the boat
        transitions onto the true-GPS path. Every waypoint therefore lands
        on its real-world GPS coordinates regardless of where the robot was
        switched on. Runs on real water and in GPS-simulated Gazebo runs
        alike — empty Gazebo (station-synthesised fixes) and personalized
        worlds (the simulator's mavros shim publishes the fixes) both take
        this path; only a sim without simulated GPS (non-anchored mission)
        skips it."""
        self._stop_gps_deployment()
        if not params.gps_anchored_source or (
                params.simulation and not params.gps_simulated):
            return
        self._arm_deployment(Path(params.gps_anchored_source),
                             Path(params.gps_deployed_target))

    def _arm_session_deployment(self, source: str) -> None:
        """Update Mission chose a GPS-anchored path for the next run: deploy
        it into the session file as soon as the frame is anchored (at once
        when it already is) -- the same watcher a launch uses."""
        self._stop_gps_deployment()
        self._arm_deployment(Path(source), self.run_session.session_file)

    def _arm_deployment(self, src: Path, dst: Path) -> None:
        from mcs.designer import io_yaml  # lazy: PyYAML machinery
        self._gps_src = src
        self._gps_dst = dst
        self._gps_dst.unlink(missing_ok=True)  # never reuse a stale frame
        self._gps_hint_countdown = 0
        self._gps_timer = QTimer(self)
        self._gps_timer.timeout.connect(lambda: self._poll_gps_deployment(io_yaml))
        self._gps_timer.start(1000)
        self._status.showMessage(
            "GPS-anchored mission: waiting for GPS fixes — the path deploys "
            "automatically (no motion needed).", 10000)

    def _poll_gps_deployment(self, io_yaml) -> None:
        if not self.store.mission.launch_running:
            self._stop_gps_deployment()
            return
        geo = self.store.geo
        if not geo.is_valid or geo.fit is None:
            self._gps_hint_countdown -= 1
            if self._gps_hint_countdown <= 0:
                self._gps_hint_countdown = 20
                self._status.showMessage(
                    "GPS path pending: waiting for GPS fixes to anchor the "
                    "frame (check /mavros/global_position/global).", 8000)
            return
        try:
            io_yaml.deploy_mission(self._gps_src, geo.fit, self._gps_dst)
        except Exception as exc:  # noqa: BLE001 - surfaced, retried next poll
            _LOG.error("GPS deployment failed: %s", exc)
            self._status.showMessage(f"GPS deployment failed: {exc}", 8000)
            return
        self._stop_gps_deployment()
        _LOG.info("GPS mission deployed to %s (fit rms %.2f m)",
                  self._gps_dst, geo.fit.rms_m)
        if self.store.session.active:
            self.run_session.mark_deployed(str(self._gps_src))
            self._status.showMessage(
                f"GPS path deployed (georef rms {geo.fit.rms_m:.2f} m) — shown "
                "on the map, start circled 'S'; the boat follows it when the "
                "run starts.", 10000)
        else:
            self._status.showMessage(
                f"GPS path deployed (georef rms {geo.fit.rms_m:.2f} m) — the "
                "robot transitions onto the mission at its next path request.",
                10000)
        QTimer.singleShot(
            1500, lambda: self._request_path_preview(f"from_yaml:{self._gps_dst}"))

    def _stop_gps_deployment(self) -> None:
        timer = getattr(self, "_gps_timer", None)
        if timer is not None:
            timer.stop()
            timer.deleteLater()
            self._gps_timer = None

    def _request_path_preview(self, trajectory: str) -> None:
        """Request the full mission path over the right horizon.

        Default horizon comes from launch.path_preview_total_time_s; for a
        designer trajectory the YAML's own duration_s replaces it, so long
        custom missions are previewed completely instead of being cut at the
        legacy 120 s limit.
        """
        self._last_preview_trajectory = trajectory  # retry target on failure
        total = self.cfg.launch.path_preview_total_time_s
        if trajectory.startswith("from_yaml:"):
            try:
                import yaml  # PyYAML, already a station dependency
                data = yaml.safe_load(
                    Path(trajectory.partition(":")[2]).read_text()) or {}
                total = float(data.get("duration_s", total)) + 1.0
            except Exception as exc:  # noqa: BLE001 - preview is best-effort
                _LOG.warning("Could not read YAML duration (falling back to "
                             "the %.0f s horizon): %s", total, exc)
        self.commands.request_mission_path(
            total_time=total, dt=self.cfg.launch.path_preview_dt_s)

    def _on_mission_path_failed(self, err: str) -> None:
        """One failed/timed-out /path_request must not mean 'no path ever':
        re-arm the pending preview (bounded, spaced) while the mission runs."""
        retries = getattr(self, "_preview_retries_left", 0)
        trajectory = getattr(self, "_last_preview_trajectory", None)
        if (trajectory is not None and retries > 0
                and self.store.mission.launch_running):
            self._preview_retries_left = retries - 1
            _LOG.warning("path preview failed (%s) — retrying, %d attempt(s) "
                         "left", err, retries)
            self._status.showMessage(
                f"Path request failed ({err}) — retrying…", 5000)
            delay_ms = int(self.cfg.launch.path_request_retry_delay_s * 1000)
            QTimer.singleShot(
                delay_ms, lambda t=trajectory: self._rearm_path_preview(t))
            return
        _LOG.error("path preview failed: %s", err)
        self._status.showMessage(
            f"Path request failed: {err} — mission path preview unavailable.",
            30000)
        self.bus.launch_output.emit(f"[station] path preview failed: {err}")

    def _rearm_path_preview(self, trajectory: str) -> None:
        if self.store.mission.launch_running:
            self._pending_preview_trajectory = trajectory

    def _on_mission_stopped(self) -> None:
        pass  # state cleared on 'idle' launch_state

    def _on_launch_state(self, state: str) -> None:
        if state == "idle":
            self.store.mission.launch_running = False
            self.store.mission.manual_target = None
            self.store.mission.simulation = False
            self.store.mission.gps_simulated = False
            self.commands.disarm_sim_gps()
            # A finished run's path must not survive onto the next mission's
            # map (it would silently re-anchor with the next run's frame) —
            # and neither must a request left in flight by the dead run's
            # path_generation poison the next mission's preview.
            self.store.mission_path = None
            self._pending_preview_trajectory = None
            self._last_preview_trajectory = None
            self._preview_retries_left = 0
            self.commands.cancel_mission_path_request()
            self.commands.set_simulation_mode(False)
            self.map_view.clear_manual_target()
            self.toolbar.set_manual_target_active(False)
            if self.toolbar.manual_button.isChecked():
                self.toolbar.manual_button.setChecked(False)
            self.arrow_box.set_active(False, hold=False)
            self.run_session.end()
            self.store.guardian = None

    # ============================================================ link guardian
    def _real_session(self) -> bool:
        m = self.store.mission
        return m.launch_running and not m.simulation and not m.replay

    def _refresh_guardian(self) -> None:
        from pathlib import Path

        from mcs.core.guardian import summarize
        g = self.cfg.guardian
        ss, r = self.store.session, self.store.robot
        view = summarize(
            self.store.guardian, real_session=self._real_session(),
            companion_found=g.enabled and Path(g.script).expanduser().is_file(),
            fcu_connected=r.fcu_connected, armed=r.armed, mode=r.fcu_mode,
            run_active=ss.run_active or ss.run_pending == "start",
            estop_latched=self.commands.estop_latched)
        self.guardian_box.update_view(view, self.store.guardian)
        # Report each answered command once, in the status bar.
        ack = self.store.guardian.ack if self.store.guardian is not None else None
        if ack and ack.get("seq") != self._guardian_ack_seq:
            self._guardian_ack_seq = ack.get("seq")
            ok = "done" if ack.get("ok") else "FAILED"
            self._status.showMessage(
                f"Guardian {ack.get('cmd')}: {ok} — {ack.get('detail', '')}", 10000)

    def _on_guardian_action(self, action: str) -> None:
        if action == "rearm":
            self.commands.rearm()
            self._status.showMessage(
                "Arm again: the boat briefly leaves override, the autopilot arms, "
                "then override resumes.", 8000)
            return
        if self.commands.send_guardian_command(action) is None:
            QMessageBox.warning(self, "Guardian", "The command could not be sent "
                                "(ROS layer not running).")

    def _guardian_stop_guard(self, proceed) -> None:
        """Turn Off Connection: ask what the guardian does (user's order,
        2026-10-14), then proceed - or not."""
        from mcs.core.guardian import needs_turn_off_choice
        if not needs_turn_off_choice(self.store.guardian, real_session=self._real_session()):
            proceed()
            return
        choice = ask_turn_off_choice(self, exiting=False)
        if choice == "keep":
            proceed()
        elif choice == "off":
            self._guardian_off_then(proceed)

    def _guardian_off_then(self, proceed) -> None:
        self._status.showMessage("Turning the link guardian OFF and disarming…", 8000)

        def done(ok: bool, detail: str) -> None:
            if ok:
                proceed()
                return
            QMessageBox.critical(
                self, "Guardian still ON",
                f"The guardian could not be turned off: {detail}.\n\nNothing was "
                "terminated. Try again, or choose 'Keep ON' to let the boat come home.")
        self.commands.guardian_off_and_disarm(done)

    # ============================================================ manual target
    def _on_manual_mode(self, armed: bool) -> None:
        """Manual Target button = one-shot *arming* of the next map click.

        Arming/disarming never publishes anything; the [0.0, 0.0] resume
        message is published exclusively by 'Continue Original Mission'.
        """
        if armed:
            self.toolbar.measure_button.setChecked(False)
            self.map_view.set_mode(MapMode.MANUAL_TARGET)
            self._status.showMessage(
                "Manual Target armed: the next map click is published as the "
                "target, then the map returns to normal interaction.", 8000)
        else:
            self.map_view.set_mode(MapMode.NORMAL)

    def _on_target_clicked(self, x: float, y: float) -> None:
        if self.arrow_box.active:
            # A clicked target is a newer intent than the keys: end arrow
            # control (no hold point) before publishing it.
            self.arrow_box.set_active(False, hold=False)
        if x == 0.0 and y == 0.0:
            # (0,0) is the reserved resume sentinel of master_control — nudge it.
            x = 1e-3
        self.commands.publish_manual_target(x, y)
        self.store.mission.manual_target = (x, y)
        self.map_view.show_manual_target(x, y)
        # One-shot: disarm immediately so pan / inspect / measure work while
        # the boat drives to the target (the crosshair and the predicted path
        # persist through the store until the mission is resumed).
        self.toolbar.manual_button.setChecked(False)  # -> _on_manual_mode(False)
        self.toolbar.set_manual_target_active(True)

    def _on_continue_mission(self) -> None:
        """'Continue Original Mission': publish [0.0, 0.0], nothing else.

        Between the runs of a session the same button reads 'Release Target'
        and the same sentinel just lets go of the target (zero thrust)."""
        self.commands.resume_original_mission()
        self._clear_manual_target_ui()
        ss = self.store.session
        if ss.active and not (ss.run_active or ss.run_pending == "start"):
            self._status.showMessage(
                "Published [0.0, 0.0] — target released; no run is active, the "
                "boat holds zero thrust.", 6000)
        else:
            self._status.showMessage(
                "Published [0.0, 0.0] — master_control resumes the original "
                "mission.", 6000)

    def _clear_manual_target_ui(self) -> None:
        self.store.mission.manual_target = None
        self.map_view.clear_manual_target()
        self.toolbar.set_manual_target_active(False)

    # ============================================================ run session
    def _on_update_mission(self) -> None:
        """Update Mission (toolbar, run session, between runs)."""
        from mcs.gui.dialogs.mission_update_dialog import MissionUpdateDialog
        if not self.run_session.can_update():
            self._status.showMessage(
                "Update Mission is only possible between runs, once "
                "master_control reports.", 6000)
            return
        dlg = MissionUpdateDialog(self.cfg, self.store.session.next, self)
        if dlg.exec() != dlg.DialogCode.Accepted:
            return
        self.run_session.update_mission(dlg.selection())

    def _on_session_preview(self, trajectory: str) -> None:
        """A run's path was just written: preview it (once the map is ready)."""
        self._preview_retries_left = self.cfg.launch.path_request_max_retries
        self._pending_preview_trajectory = trajectory

    def _on_session_preview_cleared(self) -> None:
        self._stop_gps_deployment()
        self.store.mission_path = None
        self._pending_preview_trajectory = None
        self._last_preview_trajectory = None
        self.commands.cancel_mission_path_request()

    # ============================================================ arrow control
    def _on_arrows_engaged(self) -> None:
        if self.toolbar.manual_button.isChecked():
            self.toolbar.manual_button.setChecked(False)   # disarm map clicks
        self._status.showMessage(
            "Arrow control ON: ↑ ahead, ↓ astern, ← / → turn. Any running path "
            "or pinger run is paused.", 8000)

    def _on_arrows_released(self, hold: bool) -> None:
        """Arrow control OFF: hold the boat where it is (a manual target at its
        position, through the normal manual-target code), then release."""
        r = self.store.robot
        if hold and r.has_odom:
            self._on_target_clicked(r.x, r.y)
            self._status.showMessage(
                "Arrow control OFF — holding position here. Use "
                f"'{self.toolbar.continue_button.text()}' to let go.", 8000)
        self.commands.release_arrows()

    def _on_clear_paths(self) -> None:
        self.store.clear_tracks()
        self.map_view.refresh()
        self._status.showMessage("Robot and pinger trails cleared.", 4000)

    def _on_measure_mode(self, on: bool) -> None:
        if on and self.toolbar.manual_button.isChecked():
            self.toolbar.manual_button.setChecked(False)  # disarm selection only
        self.map_view.set_mode(MapMode.MEASURE if on else MapMode.NORMAL)

    # ================================================================ designer
    def _on_modify_sea(self) -> None:
        """SEA STATE box 'Change sea state…': live sea-state change (sim)."""
        from mcs.core.sea import SeaCatalog, SeaChoice, find_presets_file
        from mcs.gui.dialogs.sea_state_dialog import SeaStateDialog
        if not self.store.mission.simulation:
            return
        catalog = SeaCatalog.load(find_presets_file(self.cfg.sea.presets_file))
        last = self.store.mission.sea_choice
        dlg = SeaStateDialog(catalog, self.cfg.sea,
                             last if isinstance(last, SeaChoice) else None,
                             live=True, parent=self)
        if dlg.exec() != dlg.DialogCode.Accepted:
            return
        choice = dlg.choice()
        if self.commands.send_sea_state(choice.command_json(dlg.ramp_s())):
            self.store.mission.sea_choice = choice
            self._status.showMessage(
                f"Sea state command sent: {choice.summary(catalog)} "
                f"(ramp {dlg.ramp_s():.0f} s)", 8000)
        else:
            self._status.showMessage("ROS is not running — sea state not sent", 8000)

    def _send_replay(self, cmd: str, rate: float | None = None) -> None:
        """REPLAY box: pause / resume / playback rate of the bag replay."""
        if not self.commands.send_replay_command(cmd, rate):
            self._status.showMessage("ROS is not running — replay command not sent", 8000)

    def _open_designer(self) -> None:
        """Open the Survey Pattern Designer (one shared, non-modal instance).

        The store is passed for the single explicit "anchor on the robot's
        current fix" action only -- the editor is otherwise independent of
        live telemetry, deliberately (see DesignerWindow._active_fit)."""
        from mcs.designer.designer_window import DesignerWindow  # lazy import
        if getattr(self, "_designer", None) is None:
            self._designer = DesignerWindow(self.cfg, self.store, parent=self)
            self._designer.destroyed.connect(
                lambda: setattr(self, "_designer", None))
        self._designer.show()
        self._designer.raise_()
        self._designer.activateWindow()

    # ================================================================ shutdown
    def closeEvent(self, event: QCloseEvent) -> None:
        """Application Exit is a node-termination path, so it must honour the
        same guarantee as E-STOP / Stop Mission: 'default' is published and
        its transmission confirmed BEFORE any node dies. The sequence is
        asynchronous, so the first close is deferred (event.ignore()) while
        CommandCenter.safe_app_exit runs; the window closes itself again once
        the sequence — and the launch-process exit — have completed."""
        if self._exit_finalized:
            self.ros.stop()
            event.accept()
            return
        if self.launcher.running:
            if not self._exit_in_progress:
                from mcs.core.guardian import needs_turn_off_choice
                if needs_turn_off_choice(self.store.guardian,
                                         real_session=self._real_session()):
                    # Link guardian ON (v1.1): the same choice as Turn Off
                    # Connection, in place of the plain confirmation.
                    choice = ask_turn_off_choice(self, exiting=True)
                    if choice == "keep":
                        self._start_exit_sequence()
                    elif choice == "off":
                        self._guardian_off_then(self._start_exit_sequence)
                    event.ignore()
                    return
                answer = QMessageBox.question(
                    self, "Mission running",
                    "A mission is still running. Publish 'default', stop the "
                    "nodes safely and quit?",
                    QMessageBox.StandardButton.Yes
                    | QMessageBox.StandardButton.Cancel)
                if answer != QMessageBox.StandardButton.Yes:
                    event.ignore()
                    return
                self._start_exit_sequence()
            event.ignore()  # keep the process (and the DDS writer) alive
            return
        self.ros.stop()
        event.accept()

    def _start_exit_sequence(self) -> None:
        """Safe app exit: the N1 sequence, then close once the launch is gone."""
        if self._exit_in_progress:
            return
        self._exit_in_progress = True
        self.bus.shutdown_sequence_finished.connect(self._on_exit_sequence_done)
        self.commands.safe_app_exit()

    def _on_exit_sequence_done(self) -> None:
        """Safe-shutdown confirmed; now wait for the launch tree to exit."""
        self.bus.shutdown_sequence_finished.disconnect(self._on_exit_sequence_done)
        self._exit_deadline = 0.0
        self._poll_exit_ready()

    def _poll_exit_ready(self) -> None:
        # Give the SIGINT→SIGTERM→SIGKILL escalation (driven by QTimers that
        # need the event loop) up to its own budget before finalizing anyway.
        budget_s = (self.cfg.launch.sigint_timeout_s
                    + self.cfg.launch.sigterm_timeout_s + 3.0)
        self._exit_deadline += 0.2
        if self.launcher.running and self._exit_deadline < budget_s:
            QTimer.singleShot(200, self._poll_exit_ready)
            return
        self._exit_finalized = True
        self.close()  # re-enters closeEvent on the finalized branch
