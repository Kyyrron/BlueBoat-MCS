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
    armed: bool = False
    fcu_mode: str = "—"
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
    manual_target: tuple[float, float] | None = None
    started_t: float | None = None          # first odom after launch
    path_target: tuple[float, float] | None = None   # x_d, y_d from /monitoring_data

    @property
    def target_mode(self) -> TargetMode:
        if self.manual_target is not None:
            return TargetMode.MANUAL
        if not self.launch_running or self.controller_type == "":
            return TargetMode.NONE
        return TargetMode.PINGER if self.use_pinger else TargetMode.PATH

    def elapsed_s(self) -> float | None:
        if self.started_t is None:
            return None
        return now_mono() - self.started_t


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

    # -------------------------------------------------------------- updates
    def on_odom(self, t: float, pose, twist) -> None:
        r = self.robot
        prev = (r.x, r.y) if r.has_odom else None
        r.t = t
        r.x, r.y, r.yaw = float(pose[0]), float(pose[1]), float(pose[5])
        r.speed = math.hypot(float(twist[0]), float(twist[1]))
        r.has_odom = True
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
        if r.lat is not None and r.lon is not None:
            self.geo.add_pair(t, r.x, r.y, r.lat, r.lon)
        # NOTE: the pinger world position is deliberately NOT recomputed here.
        # Re-anchoring a (possibly stale, dead-reckoned) body-frame pinger
        # vector to every new robot pose made the pinger marker follow the
        # robot's motion; it is now computed once per pinger message, with
        # the pose concurrent with that message (see on_pinger_body).
        self._record_target_distance(t)

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

    def on_mavros_state(self, t: float, connected: bool, armed: bool, mode: str) -> None:
        self.robot.fcu_connected = connected
        self.robot.armed = armed
        self.robot.fcu_mode = mode

    def on_pinger_body(self, t: float, xyz) -> None:
        """Pinger message received: THIS is the only place the pinger's world
        position is computed, using the robot pose concurrent with the
        message. Between messages the marker stays fixed in the world —
        composing an older body-frame vector with newer robot poses (the
        previous behaviour) dragged the pinger along with the robot.
        Residual lag now comes only from the robot side (Waterlinked
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
        last = self.pinger_track.last()
        if last is None or (t - last[0]) > 0.2:  # trail stored at <= 5 Hz
            self.pinger_track.append(t, (xw, yw))

    def on_uw_gps_raw(self, t: float) -> None:
        self.pinger.last_raw_update_t = t

    def on_monitoring(self, t: float, data) -> None:
        # [t_ctrl, x, y, psi, x_d, y_d, psi_d, u1, u2]
        # x_d/y_d/psi_d are WORLD-frame for every controller branch —
        # guaranteed by the patched integration/master_control.py, which
        # captures the world target before its inRobotFrame() conversion.
        # (Do NOT re-add a robot->world fixup here: with the patched
        # controller it would double-convert. If running an UNPATCHED
        # master_control, LoS-path/manual/pinger targets arrive robot-frame
        # and will display off-path — deploy the patched file instead.)
        if len(data) >= 7:
            self.mission.path_target = (float(data[4]), float(data[5]))
        self._record_target_distance(t)

    def on_thruster(self, t: float, right: float, left: float) -> None:
        self.robot.thrust_right, self.robot.thrust_left = right, left
        self.thrust_hist.append(t, (right, left))

    def on_controller_ready(self, t: float, ready: bool) -> None:
        self.robot.controller_ready = ready

    def on_param_mode(self, t: float, mode: str) -> None:
        self.robot.param_mode = mode

    def on_mission_path(self, poses) -> None:
        self.mission_path = np.asarray(poses, dtype=np.float64) if len(poses) else None

    # -------------------------------------------------------------- derived
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
        None if no absolute heading source is available yet.

        Prefers /mavros/global_position/compass_hdg, which is ABSOLUTE and
        available immediately — unlike the odom yaw, whose world frame is
        launch-zeroed (yaw 0 at launch, so the glyph would face east at the
        start regardless of the real heading). Falls back to the georeference
        offset (yaw + theta) only once heading-aligned."""
        if self.robot.compass_heading is not None:
            return self.robot.compass_heading
        if not self.robot.has_odom:
            return None
        fit = self.geo.fit
        if fit is None or not fit.heading_aligned:
            return None
        return fit.world_yaw_to_true(self.robot.yaw)

    def world_frame_ready(self) -> bool:
        """True once the odom->GPS heading alignment has been established."""
        fit = self.geo.fit
        return fit is not None and fit.heading_aligned

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
        self._t0 = None
