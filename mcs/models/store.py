"""Central data store: live state snapshots and full-experiment histories.

The store lives in the GUI thread.  Slots connected to the
:class:`~mcs.core.signals.SignalBus` update it; widgets *read* from it on a
fixed 10 Hz refresh tick.  Recording is never interrupted by what the
timeline currently displays.

Design notes
------------
* All times are ``time.monotonic()`` seconds stamped at message reception,
  giving one consistent clock across every source topic.
* Derived quantities that already exist on a ROS topic are *not* recomputed
  here (e.g. the current path target comes from ``/monitoring_data``).  The
  only computations performed are pure display transforms: pinger body ->
  world frame, distances, travelled-distance integration, speed statistics.
* **Run sessions** (real-robot launches, 2026-10-09): histories record only
  while a RUN is active (:meth:`DataStore.recording`). Between runs the live
  state (boat glyph, pinger, target) keeps updating, but nothing is appended,
  so the trail, statistics, distance plot and timeline stay frozen on the last
  run until the next Start Run resets them. Outside a session (simulation)
  recording is unconditional, as before.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum

import numpy as np

from mcs.config.settings import AppConfig
from mcs.core.geo import GeoReferencer
from mcs.core.series import TimeSeries


def now_mono() -> float:
    return time.monotonic()


class TargetMode(Enum):
    """What the boat is currently steering towards (drives the target panel/plot)."""

    NONE = "none"
    PATH = "path"          # trajectory following (use_pinger = False)
    PINGER = "pinger"      # pinger homing (use_pinger = True)
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


@dataclass
class PingerState:
    """Latest pinger snapshot (body frame from topic, world frame derived)."""

    t: float = 0.0
    body: tuple[float, float, float] = (0.0, 0.0, 0.0)
    world: tuple[float, float] | None = None
    distance_m: float | None = None
    last_raw_update_t: float | None = None  # last /uw_gps_data packet
    seen: bool = False


@dataclass
class MissionState:
    """Mission-level status assembled from launch config + telemetry."""

    launch_running: bool = False
    controller_type: str = ""
    use_pinger: bool = False
    simulation: bool = False                # Sim_launch.py graph (no MAVROS/pinger)
    gps_simulated: bool = False             # sim run with a synthesised NavSatFix
                                            # feed (GPS-anchored mission): the full
                                            # GPS pipeline — anchor gate, tiles,
                                            # deferred deploy — runs as on real water
    manual_target: tuple[float, float] | None = None
    world_dir: str = ""                     # personalized world folder ("" = empty Gazebo)
    # Gazebo replay of a recorded real-robot run (v1.1): the recorded GPS,
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
        return TargetMode.PINGER if self.use_pinger else TargetMode.PATH

    def elapsed_s(self) -> float | None:
        if self.started_t is None:
            return None
        end = self.stopped_t if self.stopped_t is not None else now_mono()
        return max(0.0, end - self.started_t)


@dataclass
class NextMission:
    """What the next run of a session executes (launch dialog / Update Mission)."""

    source: str = ""                        # designer runtime YAML ("" = pinger)
    name: str = ""
    anchored: bool = False                  # source carries a geo_anchor
    use_pinger: bool = False
    controller_type: str = "PID"
    note: str = ""

    def summary(self) -> str:
        what = "USBL pinger" if self.use_pinger else (self.name or "no path")
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
    pinger: PingerState = field(default_factory=PingerState)
    mission: MissionState = field(default_factory=MissionState)
    session: SessionState = field(default_factory=SessionState)

    def __post_init__(self) -> None:
        self.geo = GeoReferencer(self.cfg.geo)
        # Histories (t is monotonic reception time)
        self.robot_track = TimeSeries(dim=3)      # x, y, yaw
        self.pinger_track = TimeSeries(dim=2)     # world x, y
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
        # NOTE: geo pairing happens in on_gps (at GPS rate, with a freshness
        # guard on the odom pose) — pairing here at odom rate re-used one GPS
        # fix against ~4 different odom positions and biased the estimate.
        # NOTE: the pinger world position is deliberately NOT recomputed here.
        # Re-anchoring a (possibly stale, dead-reckoned) body-frame pinger
        # vector to every new robot pose made the pinger marker follow the
        # robot's motion; it is now computed once per pinger message, with
        # the pose concurrent with that message (see on_pinger_body).
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
        import math as _m
        a = _m.radians(90.0 - float(heading_deg))
        self.robot.compass_heading = _m.atan2(_m.sin(a), _m.cos(a))

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
        # the ArduPilot mode the GUARDIAN box (v1.1) - the autopilot's own
        # arming, distinct from robot_interface's motor gate.
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
        """Pinger message received: THIS is the only place the pinger's world
        position is computed, using the robot pose concurrent with the
        message. Between messages the marker stays fixed in the world —
        composing an older body-frame vector with newer robot poses (the
        previous behaviour) dragged the pinger along with the robot.
        Residual lag now comes only from the source (robot_interface: Waterlinked
        'filaco' filtering + dead-reckoning drift, see 03_ros_integration)."""
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
        ss = self.session
        if not ss.active:
            return True
        return ss.run_active or ss.run_pending == "start"

    def active_target_world(self) -> tuple[float, float] | None:
        """World position of whatever the boat is currently steering to."""
        mode = self.mission.target_mode
        if mode is TargetMode.MANUAL:
            return self.mission.manual_target
        if mode is TargetMode.PINGER:
            return self.pinger.world
        if mode is TargetMode.PATH:
            return self.mission.path_target
        return None

    def clear_tracks(self) -> None:
        """Operator 'Clear Paths': wipe the robot and pinger trails drawn on
        the map. Live states (poses, targets, stats, histories used by the
        plots) are untouched."""
        self.robot_track.clear()
        self.pinger_track.clear()

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
        """Clear histories (called on new mission launch, on request)."""
        for s in (self.robot_track, self.pinger_track, self.speed_hist,
                  self.thrust_hist, self.target_dist_hist):
            s.clear()
        self.robot.travelled_m = 0.0
        self.mission.started_t = None
        self.mission.stopped_t = None
        self._t0 = None
        self.sea = None
        self.replay = None
