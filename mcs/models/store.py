"""Central data store: live state snapshots and full-experiment histories.

The store lives in the GUI thread.  Slots connected to the
:class:`~mcs.core.signals.SignalBus` update it; widgets *read* from it on a
fixed 10 Hz refresh tick.  Recording is never interrupted by what the
timeline currently displays.

Design notes
------------
* All times are station-clock seconds stamped at message reception
  (:func:`mcs.core.clock.station_clock`: ``time.monotonic()``, sim-paced
  under ``/clock`` in a simulation launch), giving one consistent clock
  across every source topic.
* Derived quantities that already exist on a ROS topic are *not* recomputed
  here (e.g. the current path target comes from ``/monitoring_data``, the
  filtered pinger -- position, uncertainty, status, statistics -- from
  ``/blueboat/pinger/estimate``, taken verbatim in its world frame).  The only
  computations performed are pure display transforms: the LEGACY pinger body
  vector -> world frame (N4), distances, travelled-distance integration,
  speed statistics.
* **Run sessions** (real-robot launches): histories record only
  while a RUN is active (:meth:`DataStore.recording`). Between runs the live
  state (boat glyph, pinger, target) keeps updating, but nothing is appended,
  so the trail, statistics, distance plot and timeline stay frozen on the last
  run until the next Start Run resets them. Outside a session (simulation)
  recording is unconditional.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from mcs.config.settings import AppConfig
from mcs.core.clock import station_clock
from mcs.core.geo import GeoReferencer
from mcs.core.pinger_filter import (
    MODE_FOLLOW,
    MODE_LOG,
    MODE_OFF,
    SOURCE_FILTER,
    SOURCE_LEGACY,
    decision_code,
    effective_mode,
    same_params,
)
from mcs.core.series import TimeSeries


def now_mono() -> float:
    """Now on the station clock (``time.monotonic()`` outside a simulation
    launch; sim-paced under ``/clock`` in one -- core/clock.py)."""
    return station_clock().now()


class TargetMode(Enum):
    """What the boat is currently steering towards (drives the target panel/plot)."""

    NONE = "none"
    PATH = "path"          # trajectory following (pinger mode off, or log with a path)
    PINGER = "pinger"      # pinger homing (pinger mode follow = use_pinger)
    MANUAL = "manual"      # operator manual target (overrides everything)


@dataclass
class RobotState:
    """Latest robot telemetry snapshot."""

    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = 0.0                      # sqrt(u^2 + v^2) from odom twist
    lat: float | None = None
    lon: float | None = None
    compass_heading: float | None = None    # true heading, rad CCW-from-east
    fcu_connected: bool = False
    armed: bool = False                     # /mavros/state: the AUTOPILOT's arming
    fcu_mode: str = ""                      # /mavros/state: ArduPilot mode name
    battery_v: float | None = None          # pack voltage, None = not reported
    battery_pct: float | None = None        # charge fraction 0..1
    battery_t: float | None = None          # reception time of the last message
    param_mode: str = "—"                   # 'default' / 'override'
    controller_ready: bool = False
    thrust_right: float = 0.0
    thrust_left: float = 0.0
    travelled_m: float = 0.0
    has_odom: bool = False


#: NextMission.kind() values (also the configure's ``mission`` tag, but for a path).
RUN_PINGER, RUN_PINGER_LOG, RUN_NO_PATH, RUN_PATH = "pinger", "pinger_log", "no_path", "path"


def finite_float(value) -> float | None:
    """A finite float, or None (JSON null / garbage / NaN)."""
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


@dataclass
class PingerState:
    """The LEGACY pinger: the old algorithm's body vector
    (``/blueboat/pinger/legacy_body``, "dragged by the boat between fixes"),
    turned into world frame once per message (N4). Reset at every launch and
    every run start."""

    t: float = 0.0
    body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    world: tuple[float, float] | None = None
    distance_m: float | None = None
    last_raw_update_t: float | None = None  # last /uw_gps_data packet
    seen: bool = False


@dataclass
class PingerEstimate:
    """The FILTERED pinger (2026-11-24): the latest
    ``/blueboat/pinger/estimate`` (``blueboat_pinger_estimate/1``) and the
    vector ``master_control`` steers to (``/blueboat/pinger_coordinates``).

    ``world`` and ``ellipse`` are the estimate's own ``x, y`` and 95 %
    ``ellipse`` -- WORLD frame (``/blueboat/odom`` local ENU) at the source,
    taken verbatim: nothing here re-transforms them (N3/N4 spirit). Every
    other number the panel shows (status, fix age and rate, r95, velocity,
    decision statistics, range) is read from ``payload``. Reset at every
    launch and every run start; the estimator's ~1 Hz heartbeat restores it."""

    t: float | None = None                  # reception time of the last estimate
    payload: dict = field(default_factory=dict)
    world: tuple[float, float] | None = None
    ellipse: tuple[float, float, float] | None = None   # a, b (m), theta (rad CCW from east)
    body: tuple[float, float, float] | None = None      # /blueboat/pinger_coordinates
    body_t: float | None = None

    @property
    def status(self) -> str:
        return str(self.payload.get("status") or "no_data")

    @property
    def source(self) -> str:
        """What /blueboat/pinger_coordinates carries: filter | legacy."""
        return str(self.payload.get("source") or SOURCE_FILTER)


@dataclass
class PingerFilterState:
    """The filter's applied parameter set (``/blueboat/pinger/filter_config``,
    latched) against what the station last sent in a ``configure``.

    *Applied* means the echo answers the sent ``seq``, says ``ok`` and holds
    the sent parameters (and source). A refusal (``ok: false``) leaves the
    previous set in force; its ``detail`` is the reason."""

    echo: dict = field(default_factory=dict)
    echo_t: float | None = None
    sent_seq: int | None = None
    sent_preset: str = ""
    sent_params: dict = field(default_factory=dict)
    sent_source: str = ""

    def mark_sent(self, seq: int, block: dict | None, source: str) -> None:
        self.sent_seq = seq
        self.sent_preset = str((block or {}).get("preset") or "")
        self.sent_params = dict((block or {}).get("params") or {})
        self.sent_source = str(source or "")

    def answered(self) -> bool:
        return self.sent_seq is not None and self.echo.get("seq") == self.sent_seq

    def refusal(self) -> str | None:
        """The refusal reason when the echo of the sent seq says not ok."""
        if self.answered() and not self.echo.get("ok"):
            return str(self.echo.get("detail") or "refused")
        return None

    def applied(self) -> bool:
        if not self.answered() or not self.echo.get("ok"):
            return False
        if self.sent_source and self.echo.get("pinger_source") != self.sent_source:
            return False
        return not self.sent_params or same_params(self.echo.get("params"),
                                                   self.sent_params)


@dataclass
class MissionState:
    """Mission-level status assembled from launch config + telemetry."""

    launch_running: bool = False
    controller_type: str = ""
    use_pinger: bool = False                # pinger mode follow (Control's alias)
    pinger_mode: str = MODE_OFF             # off | follow | log (run_state in a session)
    has_path: bool = True                   # False: a run without a path (off / log)
    path_complete: bool = False             # run_state: a non-loop path ended
    simulation: bool = False                # Sim_launch.py graph (no MAVROS/pinger)
    gps_simulated: bool = False             # sim run with a synthesised NavSatFix
                                            # feed (GPS-anchored mission): the full
                                            # GPS pipeline — anchor gate, tiles,
                                            # deferred deploy — runs as on real water
    manual_target: tuple[float, float] | None = None
    world_dir: str = ""                     # personalized world folder ("" = empty Gazebo)
    # Gazebo replay of a recorded real-robot run: the recorded GPS,
    # compass and odom drive the map exactly as on real water, so the map
    # waits for a real GPS anchor; nothing commands the replayed boat.
    replay: bool = False
    sea_choice: object = None               # SeaChoice chosen at launch (sim only)
    started_t: float | None = None          # first odom after launch (session: of the run)
    stopped_t: float | None = None          # session: when the last run stopped
    path_target: tuple[float, float] | None = None   # x_d, y_d from /monitoring_data
    # False only in an idle run session: no path / pinger is being followed.
    following: bool = True

    @property
    def target_mode(self) -> TargetMode:
        if self.manual_target is not None:
            return TargetMode.MANUAL
        if (not self.launch_running or self.controller_type == ""
                or not self.following):
            return TargetMode.NONE
        if self.use_pinger:
            return TargetMode.PINGER
        if not self.has_path:
            return TargetMode.NONE          # no path: Manual Target / arrows only
        return TargetMode.PATH

    def elapsed_s(self) -> float | None:
        if self.started_t is None:
            return None
        end = self.stopped_t if self.stopped_t is not None else now_mono()
        return max(0.0, end - self.started_t)


@dataclass
class NextMission:
    """What the next run of a session executes (launch dialog / Update Mission).

    ``pinger_mode`` (2026-11-24): ``off`` (no pinger following), ``follow``
    (steer to the pinger, no path), ``log`` (log the pinger only). In ``off``
    and ``log`` the run follows its path, or with no path -- ``source`` "",
    the dialogs' "(no path)" -- it only holds manual targets and arrow keys. ``use_pinger`` is its alias for an older control stack
    (= mode follow); a mission built with ``use_pinger`` alone gets the mode
    from it. ``pinger_filter`` is the ``configure``'s ``{"preset", "params"}``
    block, ``pinger_preset`` the preset it was built from."""

    source: str = ""                        # designer runtime YAML ("" = no path)
    name: str = ""
    anchored: bool = False                  # source carries a geo_anchor
    use_pinger: bool = False
    controller_type: str = "PID"
    note: str = ""
    pinger_mode: str = ""                   # "" = from use_pinger
    pinger_source: str = SOURCE_FILTER
    pinger_preset: str = ""
    pinger_filter: dict | None = None

    def __post_init__(self) -> None:
        self.pinger_mode = effective_mode(self.pinger_mode, self.use_pinger)
        self.use_pinger = self.pinger_mode == MODE_FOLLOW
        if self.pinger_source not in (SOURCE_FILTER, SOURCE_LEGACY):
            self.pinger_source = SOURCE_FILTER

    @property
    def has_path(self) -> bool:
        """The configure's ``has_path``: False for off / log with no path
        (following the pinger, the path means nothing: True, as Control's default)."""
        return self.pinger_mode == MODE_FOLLOW or bool(self.source)

    @property
    def uses_path(self) -> bool:
        """The run follows a designer path (its file is written at START RUN)."""
        return self.pinger_mode != MODE_FOLLOW and self.has_path

    def kind(self) -> str:
        """What the run does, the one classification every description and
        the configure's ``mission`` tag branch on: ``pinger`` (follow),
        ``pinger_log`` / ``no_path`` (no path, pinger logged or not), or
        ``path`` (a designer path; then :attr:`logs_pinger` may add the log)."""
        if self.pinger_mode == MODE_FOLLOW:
            return RUN_PINGER
        if not self.source:
            return RUN_PINGER_LOG if self.pinger_mode == MODE_LOG else RUN_NO_PATH
        return RUN_PATH

    @property
    def logs_pinger(self) -> bool:
        return self.pinger_mode == MODE_LOG

    def summary(self) -> str:
        kind = self.kind()
        if kind == RUN_PINGER:
            what = "USBL pinger"
        elif kind == RUN_PINGER_LOG:
            what = "pinger log only (no path)"
        elif kind == RUN_NO_PATH:
            what = "no path (Manual Target / arrows)"
        else:
            what = (self.name or "no path") + (" + pinger log" if self.logs_pinger else "")
        return f"{what} · {self.controller_type}"


@dataclass
class SessionState:
    """A real-robot run session (connected; runs started and stopped by hand).

    ``state`` / ``log`` are the last ``/blueboat/run_state`` (master_control)
    and ``/blueboat/run_log`` (robot_interface) JSON payloads; ``run_active``
    follows master_control's acknowledgement, ``run_pending`` the command the
    station sent and is still waiting on."""

    active: bool = False
    motors_enabled: bool = False            # launched with enable_motors:=True
    next: NextMission = field(default_factory=NextMission)
    run_active: bool = False
    run_pending: str = ""                   # "start" | "stop" | ""
    run: int = 0                            # run counter (current / last)
    state: dict = field(default_factory=dict)
    state_t: float | None = None
    log: dict = field(default_factory=dict)
    log_t: float | None = None
    arrow_active: bool = False
    arrow_cmd: tuple[float, float] = (0.0, 0.0)   # (right, left) N
    # (2026-11-25) The current run's safety way back, [(lat, lon), ...], as sent to the
    # link guardian at START RUN ([] = none: a link loss retraces the trail). Drawn on
    # the map while the guardian reports it loaded.
    safety_way_back: list = field(default_factory=list)
    # (2026-11-26) The way back of the path the map shows before its run starts (the next
    # run's GPS-anchored path, or a Gazebo launch's): drawn like the designer, so a path's
    # way back is visible whenever the path is. [] = none.
    planned_safety_way_back: list = field(default_factory=list)

    @property
    def run_in_progress(self) -> bool:
        """A run is active, or its start was sent and is awaiting the ack."""
        return self.run_active or self.run_pending == "start"

    def state_fresh(self, max_age_s: float) -> bool:
        return self.state_t is not None and (now_mono() - self.state_t) <= max_age_s


@dataclass
class Statistics:
    duration_s: float = 0.0
    travelled_m: float = 0.0
    avg_speed: float = 0.0
    max_speed: float = 0.0


@dataclass
class DataStore:
    """All live states + histories, owned by the GUI thread."""

    cfg: AppConfig
    robot: RobotState = field(default_factory=RobotState)
    pinger: PingerState = field(default_factory=PingerState)          # legacy
    pinger_est: PingerEstimate = field(default_factory=PingerEstimate)  # filtered
    pinger_filter: PingerFilterState = field(default_factory=PingerFilterState)
    mission: MissionState = field(default_factory=MissionState)
    session: SessionState = field(default_factory=SessionState)

    def __post_init__(self) -> None:
        self.geo = GeoReferencer(self.cfg.geo)
        # Histories (t is monotonic reception time)
        self.robot_track = TimeSeries(dim=3)      # x, y, yaw
        self.pinger_track = TimeSeries(dim=2)     # legacy world x, y
        self.pinger_est_track = TimeSeries(dim=2)  # filtered estimate x, y
        self.pinger_fixes = TimeSeries(dim=3)      # world fix x, y, decision code
        self.speed_hist = TimeSeries(dim=1)
        self.thrust_hist = TimeSeries(dim=2)      # right, left
        self.target_dist_hist = TimeSeries(dim=1)  # robot<->active-target distance
        self.mission_path: np.ndarray | None = None  # (n, 3) x, y, yaw
        self._t0: float | None = None             # experiment time origin
        self.sea = None                           # SeaReadout | None (sim only)
        self.replay = None                        # ReplayReadout | None (bag replay)
        self.guardian = None                      # GuardianReadout | None (real sessions)

    # -------------------------------------------------------------- updates
    def on_odom(self, t: float, pose, twist) -> None:
        r = self.robot
        prev = (r.x, r.y) if r.has_odom else None
        r.t = t
        r.x, r.y, r.yaw = float(pose[0]), float(pose[1]), float(pose[5])
        r.speed = math.hypot(float(twist[0]), float(twist[1]))
        r.has_odom = True
        if not self.recording():
            # Idle run session: the glyph moves, nothing is recorded.
            return
        if prev is not None:
            step = math.hypot(r.x - prev[0], r.y - prev[1])
            if step < 5.0:  # reject origin-reset jumps from the odometry source
                r.travelled_m += step
        if self._t0 is None:
            self._t0 = t
        if self.mission.launch_running and self.mission.started_t is None:
            self.mission.started_t = t

        self.robot_track.append(t, (r.x, r.y, r.yaw))
        self.speed_hist.append(t, (r.speed,))
        # Geo pairing happens in on_gps (at GPS rate, with a freshness guard
        # on the odom pose): pairing at odom rate would reuse one GPS fix
        # against several odom positions and bias the estimate. The pinger
        # world position is deliberately NOT recomputed here (on_pinger_body).
        self._record_target_distance(t)

    def on_sea_state(self, t: float, payload: dict) -> None:
        """Latest simulator sea state (``/sim/sea_state`` JSON)."""
        from mcs.core.sea import parse_readback
        r = parse_readback(payload, received_mono=t)
        if r is not None:
            self.sea = r

    def on_guardian_status(self, t: float, payload: dict) -> None:
        """Latest link-guardian report (``/link_guardian/status`` JSON)."""
        from mcs.core.guardian import GuardianReadout
        r = GuardianReadout.from_json(payload, received_mono=t)
        if r is not None:
            self.guardian = r

    def on_replay_state(self, t: float, payload: dict) -> None:
        """Latest bag-replay state (``/replay/state`` JSON)."""
        from mcs.core.replay import parse_state
        r = parse_state(payload, received_mono=t)
        if r is not None:
            self.replay = r

    def on_compass(self, t: float, heading_deg: float) -> None:
        """Absolute heading from /mavros/global_position/compass_hdg
        (degrees, clockwise from north). Convert to the math convention used
        everywhere else (radians, CCW from east): east=0, north=pi/2, so
        heading_math = 90 - compass_deg."""
        a = math.radians(90.0 - float(heading_deg))
        self.robot.compass_heading = math.atan2(math.sin(a), math.cos(a))

    def on_gps(self, t: float, lat: float, lon: float) -> None:
        if lat == 0.0 and lon == 0.0:
            return
        self.robot.lat, self.robot.lon = lat, lon
        # Feed the georeferencer at GPS rate, only when the odom pose is
        # fresh enough to count as concurrent with this fix.
        r = self.robot
        if r.has_odom and (t - r.t) < 0.5:
            self.geo.add_pair(t, r.x, r.y, lat, lon)

    def on_mavros_state(self, t: float, connected: bool, armed: bool, mode: str) -> None:
        # The connection flag feeds the readiness count; the arming state and
        # the ArduPilot mode the GUARDIAN box - the autopilot's own arming,
        # distinct from robot_interface's motor gate.
        self.robot.fcu_connected = connected
        self.robot.armed = armed
        self.robot.fcu_mode = mode

    def on_battery(self, t: float, volts, pct) -> None:
        """A field the FCU does not report arrives as None and must not
        overwrite the last good reading -- only the stamp always advances,
        so the panel can grey the row out when the stream dies."""
        self.robot.battery_t = t
        if volts is not None:
            self.robot.battery_v = volts
        if pct is not None:
            self.robot.battery_pct = pct

    def on_pinger_body(self, t: float, xyz) -> None:
        """LEGACY pinger vector received (``/blueboat/pinger/legacy_body``):
        THIS is the only place a pinger world position is computed, using
        the robot pose concurrent with the message (N4). Between messages
        the marker stays fixed in the world — composing an older body-frame
        vector with newer robot poses would drag the pinger along with the
        robot. The legacy algorithm itself dead-reckons the vector between
        fixes ("dragged by the boat"); that lag comes from the source. The
        FILTERED pinger is never transformed here (on_pinger_estimate)."""
        p = self.pinger
        p.t = t
        p.body = (float(xyz[0]), float(xyz[1]), float(xyz[2]) if len(xyz) > 2 else 0.0)
        if any(abs(c) > 1e-9 for c in p.body):
            p.seen = True
        p.distance_m = math.hypot(p.body[0], p.body[1]) if p.seen else None
        r = self.robot
        if not (p.seen and r.has_odom):
            return
        c, s = math.cos(r.yaw), math.sin(r.yaw)
        xw = r.x + c * p.body[0] - s * p.body[1]
        yw = r.y + s * p.body[0] + c * p.body[1]
        p.world = (xw, yw)
        if not self.recording():
            return
        last = self.pinger_track.last()
        if last is None or (t - last[0]) > 0.2:  # trail stored at <= 5 Hz
            self.pinger_track.append(t, (xw, yw))

    def on_uw_gps_raw(self, t: float) -> None:
        self.pinger.last_raw_update_t = t

    def on_pinger_coordinates(self, t: float, xyz) -> None:
        """``/blueboat/pinger_coordinates``: the body vector master_control
        steers to (filtered, zeros until the filter initialises; or legacy).
        Stored for the read-out only."""
        e = self.pinger_est
        e.body = (float(xyz[0]), float(xyz[1]), float(xyz[2]) if len(xyz) > 2 else 0.0)
        e.body_t = t

    def on_pinger_estimate(self, t: float, payload: dict) -> None:
        """``/blueboat/pinger/estimate``: position and 95 % ellipse taken
        VERBATIM (world frame at the source -- no transform here); the
        filtered track grows by one point at each new position."""
        e = self.pinger_est
        e.t = t
        e.payload = payload
        x, y = finite_float(payload.get("x")), finite_float(payload.get("y"))
        if x is None or y is None:
            e.world = None
            e.ellipse = None
            return
        moved = e.world != (x, y)
        e.world = (x, y)
        e.ellipse = None
        ell = payload.get("ellipse")
        if isinstance(ell, dict):
            a, b, th = finite_float(ell.get("a")), finite_float(ell.get("b")), finite_float(ell.get("theta"))
            if a is not None and b is not None and th is not None:
                e.ellipse = (a, b, th)
        if moved and self.recording():
            self.pinger_est_track.append(t, (x, y))

    def on_pinger_fix(self, t: float, payload: dict) -> None:
        """``/blueboat/pinger/fix``: one processed fix; its WORLD position (as
        the estimator computed it) and its decision feed the raw-fix layer."""
        w = payload.get("world")
        if not isinstance(w, list | tuple) or len(w) < 2:
            return
        x, y = finite_float(w[0]), finite_float(w[1])
        if x is None or y is None or not self.recording():
            return
        self.pinger_fixes.append(t, (x, y, float(decision_code(payload.get("decision")))))

    def on_pinger_filter_config(self, t: float, payload: dict) -> None:
        """``/blueboat/pinger/filter_config`` (latched): the applied set."""
        self.pinger_filter.echo = payload
        self.pinger_filter.echo_t = t

    def on_monitoring(self, t: float, data) -> None:
        # [t_ctrl, x, y, psi, x_d, y_d, psi_d, u1, u2]
        # x_d/y_d/psi_d are WORLD-frame for every controller branch —
        # guaranteed by BlueBoat-Control/blueboat_control/src/master_control.py,
        # which captures the world target before its inRobotFrame() conversion
        # (the '--- world-frame monitoring target ---' markers, all branches).
        # (Do NOT re-add a robot->world fixup here: it would double-convert.
        # If ~/ros2_ws/install holds a STALE build that predates that capture,
        # LoS-path/manual/pinger targets arrive robot-frame and will display
        # off-path — colcon build in ~/ros2_ws, do not compensate here.)
        if len(data) >= 7:
            self.mission.path_target = (float(data[4]), float(data[5]))
        if self.recording():
            self._record_target_distance(t)

    def on_thruster(self, t: float, right: float, left: float) -> None:
        self.robot.thrust_right, self.robot.thrust_left = right, left
        if self.recording():
            self.thrust_hist.append(t, (right, left))

    def on_controller_ready(self, t: float, ready: bool) -> None:
        self.robot.controller_ready = ready

    def on_param_mode(self, t: float, mode: str) -> None:
        self.robot.param_mode = mode

    def on_mission_path(self, poses) -> None:
        self.mission_path = np.asarray(poses, dtype=np.float64) if len(poses) else None

    # -------------------------------------------------------------- derived
    def recording(self) -> bool:
        """Do histories record right now? Always, except in a run session
        between runs (a start already sent counts as running, so the first
        samples of a run are never lost waiting for its acknowledgement)."""
        return not self.session.active or self.session.run_in_progress

    def active_target_world(self) -> tuple[float, float] | None:
        """World position of whatever the boat is currently steering to."""
        mode = self.mission.target_mode
        if mode is TargetMode.MANUAL:
            return self.mission.manual_target
        if mode is TargetMode.PINGER:
            return self.pinger_target_world()
        if mode is TargetMode.PATH:
            return self.mission.path_target
        return None

    def pinger_target_world(self) -> tuple[float, float] | None:
        """Where a pinger-following run steers: the FILTERED estimate -- or
        the legacy position when the estimator reports that
        /blueboat/pinger_coordinates carries the legacy vector
        (``pinger_source: legacy``)."""
        if self.pinger_est.payload and self.pinger_est.source == SOURCE_LEGACY:
            return self.pinger.world
        return self.pinger_est.world

    def clear_tracks(self) -> None:
        """Operator 'Clear Paths': wipe the robot and pinger trails (legacy,
        filtered, raw fixes) drawn on the map. Live states (poses, targets,
        stats, histories used by the plots) are untouched."""
        self.robot_track.clear()
        self.pinger_track.clear()
        self.pinger_est_track.clear()
        self.pinger_fixes.clear()

    def reset_pinger(self) -> None:
        """Forget every pinger snapshot and trail (each launch and each run
        start): a previous launch's or run's pinger is never shown as live."""
        self.pinger = PingerState()
        self.pinger_est = PingerEstimate()
        for series in (self.pinger_track, self.pinger_est_track, self.pinger_fixes):
            series.clear()

    def reset_pinger_filter(self) -> None:
        """A new launch: no parameter set sent or applied yet."""
        self.pinger_filter = PingerFilterState()

    def robot_true_heading(self) -> float | None:
        """Robot heading referenced to true north/east (CCW from east), or
        None if no heading source is available yet.

        Prefers /mavros/global_position/compass_hdg (magnetometer, absolute).
        Falls back to the odom yaw directly: /blueboat/odom's yaw is absolute
        ENU (0 = East, CCW+) on both the real boat and the simulator, so no
        frame correction is applied — see GPS_MAP_ARCHITECTURE.md."""
        if self.robot.compass_heading is not None:
            return self.robot.compass_heading
        if self.robot.has_odom:
            return self.robot.yaw
        return None

    def map_frame_ready(self) -> bool:
        """True once the map may draw. Two simulation modes exist:

        * non-GPS sim (no anchored mission): immediately — the sim world is
          already ENU, identity anchor, no tiles;
        * GPS-sim (anchored mission, synthesised NavSatFix feed), the Gazebo
          replay of a recorded run (its recorded fixes) and real water: once
          the odom->GPS translation is anchored (a few fixes, no vehicle
          motion required).

        Nothing is drawn on the map before this."""
        return self.geo.is_valid or (
            self.mission.simulation and not self.mission.gps_simulated
            and not self.mission.replay)

    def reset_georeference(self) -> None:
        """Fresh anchor for a fresh run — called on every mission launch.

        Each launch restarts the control stack, which latches a NEW world
        origin; pairs recorded against the previous run's origin are wrong
        by construction, and the rolling fit window would blend the two
        frames for minutes. Consumers read ``store.geo`` per tick, so
        swapping the object is safe."""
        self.geo = GeoReferencer(self.cfg.geo)

    def active_target_distance(self) -> float | None:
        tgt = self.active_target_world()
        if tgt is None or not self.robot.has_odom:
            return None
        return math.hypot(tgt[0] - self.robot.x, tgt[1] - self.robot.y)

    def _record_target_distance(self, t: float) -> None:
        d = self.active_target_distance()
        if d is None:
            return
        last = self.target_dist_hist.last()
        if last is None or (t - last[0]) > 0.1:  # 10 Hz max
            self.target_dist_hist.append(t, (d,))

    # ---------------------------------------------------------------- time
    @property
    def t0(self) -> float:
        return self._t0 if self._t0 is not None else now_mono()

    def recorded_duration(self) -> float:
        rng = self.robot_track.t_range()
        return (rng[1] - rng[0]) if rng else 0.0

    def statistics(self, rel_t0: float, rel_t1: float) -> Statistics:
        """Statistics over the [rel_t0, rel_t1] experiment-time window."""
        a, b = self.t0 + rel_t0, self.t0 + rel_t1
        st = Statistics(duration_s=max(0.0, rel_t1 - rel_t0))
        ts, xy = self.robot_track.window(a, b)
        if len(ts) >= 2:
            d = np.diff(xy[:, 0:2], axis=0)
            steps = np.hypot(d[:, 0], d[:, 1])
            st.travelled_m = float(steps[steps < 5.0].sum())
        _, sp = self.speed_hist.window(a, b)
        if len(sp):
            st.avg_speed = float(sp.mean())
            st.max_speed = float(sp.max())
        return st

    def reset_experiment(self) -> None:
        """Clear histories (called on new mission launch and at START RUN).
        The pinger snapshots go too (reset_pinger)."""
        for s in (self.robot_track, self.speed_hist,
                  self.thrust_hist, self.target_dist_hist):
            s.clear()
        self.reset_pinger()
        self.robot.travelled_m = 0.0
        self.mission.started_t = None
        self.mission.stopped_t = None
        self._t0 = None
        self.sea = None
        self.replay = None
