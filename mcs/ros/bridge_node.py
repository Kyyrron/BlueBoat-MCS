"""The single ROS2 node of the station.

Responsibilities
----------------
* Subscribe to every telemetry topic of the existing stack and forward each
  message to the :class:`~mcs.core.signals.SignalBus` (thread boundary).
* Track per-topic reception statistics (rate / age / status) for the
  diagnostics panel.
* Expose the two command publishers (``/blueboat/input_str`` and
  ``/blueboat/manual_target``) through thread-safe helpers callable from
  the GUI thread.
* Query the mission path from the ``/path_request`` service exactly as
  ``path_publisher.py`` does (same request pattern, reused, not reinvented).

No control computation happens here — the node is a pure telemetry/command
bridge, mirroring the philosophy of QGroundControl's link layer.
"""

from __future__ import annotations

import json
import logging
import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R  # same dependency as the stack
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

from mcs.config.settings import AppConfig
from mcs.core.signals import SignalBus

_LOG = logging.getLogger(__name__)

try:  # mavros_msgs may be absent (a workspace without MAVROS) — degrade gracefully
    from mavros_msgs.msg import State as MavrosState

    MAVROS_MSGS_AVAILABLE = True
except ImportError:  # pragma: no cover
    MAVROS_MSGS_AVAILABLE = False

try:  # Custom interface package of the existing stack (path service)
    from blueboat_interfaces.srv import RequestPath

    BLUEBOAT_IFACES_AVAILABLE = True
except ImportError:  # pragma: no cover
    BLUEBOAT_IFACES_AVAILABLE = False


# --------------------------------------------------------------------------
@dataclass
class TopicStats:
    """Reception statistics for one monitored topic."""

    name: str
    expected_hz: float | None = None
    warn_age_s: float = 2.0
    stale_age_s: float = 8.0
    last_rx: float | None = None            # monotonic time of last message
    _stamps: list[float] = field(default_factory=list)

    def mark(self, t: float, window_s: float) -> None:
        self.last_rx = t
        self._stamps.append(t)
        cutoff = t - window_s
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.pop(0)

    def rate_hz(self, now: float, window_s: float) -> float:
        stamps = [s for s in self._stamps if s >= now - window_s]
        if len(stamps) < 2:
            return 0.0
        span = stamps[-1] - stamps[0]
        return (len(stamps) - 1) / span if span > 0 else 0.0

    def status(self, now: float) -> str:
        """'ok' | 'warn' | 'stale' | 'never'."""
        if self.last_rx is None:
            return "never"
        age = now - self.last_rx
        if age <= self.warn_age_s:
            return "ok"
        if age <= self.stale_age_s:
            return "warn"
        return "stale"


# --------------------------------------------------------------------------
class BridgeNode(Node):
    """Telemetry/command bridge between the ROS graph and the station."""

    def __init__(self, cfg: AppConfig, bus: SignalBus) -> None:
        super().__init__("mission_control_station")
        self._cfg = cfg
        self._bus = bus
        self._pub_lock = threading.Lock()
        t = cfg.topics

        # ---- Topic statistics -------------------------------------------
        self._stats: dict[str, TopicStats] = {}
        for name in vars(t).values():
            if not isinstance(name, str) or name == t.path_request:
                continue
            warn = cfg.diagnostics.warn_age_s.get(name, 2.0)
            self._stats[name] = TopicStats(
                name=name,
                expected_hz=cfg.diagnostics.expected_hz.get(name),
                warn_age_s=warn,
                stale_age_s=warn * cfg.diagnostics.stale_age_multiplier,
            )

        # ---- Subscriptions ------------------------------------------------
        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, t.odom, self._on_odom, 10)
        self.create_subscription(NavSatFix, t.gps, self._on_gps, best_effort)
        self.create_subscription(Float64, t.compass_hdg, self._on_compass, best_effort)
        # /mavros/battery is a sensor stream like the two above: subscribe
        # BEST_EFFORT, which is compatible with either publisher policy
        # (a RELIABLE subscriber against a BEST_EFFORT publisher receives
        # nothing at all -- CM-4).
        self.create_subscription(BatteryState, t.battery, self._on_battery, best_effort)
        self.create_subscription(Float32MultiArray, t.pinger_body, self._on_pinger, 10)
        self.create_subscription(Float32MultiArray, t.uw_gps_raw, self._on_uw_gps, 10)
        self.create_subscription(Float32MultiArray, t.monitoring, self._on_monitoring, 10)
        self.create_subscription(Float32MultiArray, t.thruster_input, self._on_thruster, 10)
        self.create_subscription(Bool, t.controller_ready, self._on_ctrl_ready, 10)
        self.create_subscription(String, t.param_mode, self._on_param_mode, 10)
        if MAVROS_MSGS_AVAILABLE:
            self.create_subscription(MavrosState, t.mavros_state, self._on_state, 10)
        else:
            bus.ros_log.emit("mavros_msgs not available — FCU state display disabled.")
        # Simulator sea state: the node latches its status (TRANSIENT_LOCAL,
        # depth 1) so a station started after the sim still gets the current
        # situation immediately; the reader matches that durability.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, t.sea_state, self._on_sea_state, latched)

        # ---- Publishers ----------------------------------------------------
        self._pub_input_str = self.create_publisher(String, t.input_str, 10)
        self._pub_manual_target = self.create_publisher(Float32MultiArray, t.manual_target, 10)
        self._pub_sea_command = self.create_publisher(String, t.sea_state_command, 10)

        # ---- Simulated GPS (Gazebo runs of GPS-anchored missions) ----------
        # The station itself publishes NavSatFix on the real GPS topic, which
        # its OWN subscription above then receives (DDS local loopback), so
        # the entire real data path — QoS, diagnostics marking, SignalBus,
        # store pairing — is exercised with zero new wire names. Nothing else
        # publishes that topic in a sim graph (no MAVROS). The publisher and
        # timer exist permanently; a disarmed timer tick is a no-op, so
        # arming never creates/destroys ROS entities.
        self._sim_gps_lock = threading.Lock()
        self._sim_gps_model = None      # armed by the GUI thread via set_sim_gps
        self._last_odom: tuple[float, float, float] | None = None  # (t_mono, x, y)
        self._pub_sim_gps = self.create_publisher(NavSatFix, t.gps, best_effort)
        self.create_timer(1.0 / max(cfg.sim_gps.rate_hz, 0.1), self._publish_sim_gps)

        # ---- Path service client -------------------------------------------
        self._path_client = None
        if BLUEBOAT_IFACES_AVAILABLE:
            self._path_client = self.create_client(RequestPath, t.path_request)
        else:
            bus.ros_log.emit(
                "blueboat_interfaces not available — mission path display disabled."
            )
        # _path_future / _path_issued_t are touched by the ROS thread only;
        # the GUI thread communicates through _path_request_args and
        # _path_cancel under the lock.
        self._path_future = None
        self._path_issued_t = 0.0
        self._path_pending_lock = threading.Lock()
        self._path_request_args: tuple[float, float] | None = None
        self._path_cancel = False

        # ---- Housekeeping timer (runs in the ROS thread) --------------------
        self.create_timer(cfg.diagnostics.update_period_s, self._emit_stats)
        self.create_timer(0.2, self._poll_path_future)

    # ================================================================ inputs
    def _mark(self, topic: str) -> float:
        t = time.monotonic()
        st = self._stats.get(topic)
        if st is not None:
            st.mark(t, self._cfg.diagnostics.rate_window_s)
        return t

    def _on_odom(self, msg: Odometry) -> None:
        t = self._mark(self._cfg.topics.odom)
        p = msg.pose.pose
        quat = [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
        try:
            roll, pitch, yaw = R.from_quat(quat).as_euler("xyz", degrees=False)
        except ValueError:  # zero quaternion before first fix
            roll = pitch = yaw = 0.0
        pose = [p.position.x, p.position.y, p.position.z, roll, pitch, yaw]
        tw = msg.twist.twist
        twist = [tw.linear.x, tw.linear.y, tw.linear.z,
                 tw.angular.x, tw.angular.y, tw.angular.z]
        # No lock: _on_odom and the sim-GPS timer both run on the node's
        # single-threaded executor.
        self._last_odom = (t, p.position.x, p.position.y)
        self._bus.odom_received.emit(t, pose, twist)

    def _on_compass(self, msg: Float64) -> None:
        # /mavros/global_position/compass_hdg: absolute heading in DEGREES,
        # clockwise from north (0=N, 90=E). Preferred glyph heading source;
        # the fallback is the odom yaw, which is absolute ENU too.
        t = self._mark(self._cfg.topics.compass_hdg)
        self._bus.compass_received.emit(t, float(msg.data))

    def _on_gps(self, msg: NavSatFix) -> None:
        t = self._mark(self._cfg.topics.gps)
        self._bus.gps_received.emit(t, float(msg.latitude), float(msg.longitude))

    def _on_battery(self, msg: BatteryState) -> None:
        """/mavros/battery (sys_status plugin).

        ``percentage`` is a **0..1 fraction** in sensor_msgs/BatteryState --
        MAVROS divides the MAVLink ``battery_remaining`` percent by 100. Both
        it and ``voltage`` are NaN (or negative) when the FCU does not report
        them, which is passed on as ``None`` rather than displayed as a number.
        """
        t = self._mark(self._cfg.topics.battery)
        volts = float(msg.voltage)
        pct = float(msg.percentage)
        self._bus.battery_received.emit(
            t,
            volts if math.isfinite(volts) and volts > 0.0 else None,
            pct if math.isfinite(pct) and pct >= 0.0 else None,
        )

    def _on_state(self, msg) -> None:
        t = self._mark(self._cfg.topics.mavros_state)
        self._bus.mavros_state_received.emit(
            t, bool(msg.connected), bool(msg.armed), str(msg.mode)
        )

    def _on_pinger(self, msg: Float32MultiArray) -> None:
        t = self._mark(self._cfg.topics.pinger_body)
        data = list(msg.data)
        if len(data) >= 2:
            self._bus.pinger_body_received.emit(t, data)

    def _on_uw_gps(self, msg: Float32MultiArray) -> None:
        t = self._mark(self._cfg.topics.uw_gps_raw)
        self._bus.uw_gps_raw_received.emit(t)

    def _on_monitoring(self, msg: Float32MultiArray) -> None:
        t = self._mark(self._cfg.topics.monitoring)
        self._bus.monitoring_received.emit(t, list(msg.data))

    def _on_thruster(self, msg: Float32MultiArray) -> None:
        t = self._mark(self._cfg.topics.thruster_input)
        data = list(msg.data)
        if len(data) >= 2:
            # Convention from master_control / robot_interface: [right, left]
            self._bus.thruster_received.emit(t, float(data[0]), float(data[1]))

    def _on_ctrl_ready(self, msg: Bool) -> None:
        t = self._mark(self._cfg.topics.controller_ready)
        self._bus.controller_ready_received.emit(t, bool(msg.data))

    def _on_sea_state(self, msg: String) -> None:
        t = self._mark(self._cfg.topics.sea_state)
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if isinstance(payload, dict):
            self._bus.sea_state_received.emit(t, payload)

    def _on_param_mode(self, msg: String) -> None:
        t = self._mark(self._cfg.topics.param_mode)
        self._bus.param_mode_received.emit(t, str(msg.data))

    # =============================================================== outputs
    # publish() on rclpy publishers is safe to call from other threads; the
    # lock only serialises our own bookkeeping.
    def publish_input_str(self, command: str) -> None:
        """Publish on /blueboat/input_str ('default', 'override', 'stop', ...)."""
        with self._pub_lock:
            msg = String()
            msg.data = command
            self._pub_input_str.publish(msg)
        self._bus.command_sent.emit(f"input_str ← '{command}'")

    def input_str_subscriber_count(self) -> int:
        """Number of DDS subscriptions currently *matched* to the input_str
        publisher (ROS graph discovery). ``> 0`` proves ``robot_interface``'s
        subscription has completed the reliable-QoS handshake with our writer,
        i.e. a publish will be delivered (and retransmitted if needed) by DDS.
        Thread-safe: rmw graph queries may be called from any thread."""
        return self._pub_input_str.get_subscription_count()

    def publish_sea_state_command(self, payload: str) -> None:
        """Publish a JSON command on /sim/sea_state/command (sim only)."""
        with self._pub_lock:
            msg = String()
            msg.data = payload
            self._pub_sea_command.publish(msg)
        self._bus.command_sent.emit("sea_state/command ← " + payload[:60])

    def publish_manual_target(self, x: float, y: float) -> None:
        """Publish a manual target; (0, 0) resumes the original mission."""
        with self._pub_lock:
            msg = Float32MultiArray()
            msg.data = [float(x), float(y)]
            self._pub_manual_target.publish(msg)
        self._bus.command_sent.emit(f"manual_target ← [{x:.2f}, {y:.2f}]")

    # ---------------------------------------------------------- simulated GPS
    def set_sim_gps(self, model) -> None:
        """Arm (a SimGpsModel) or disarm (None) the simulated GPS feed.

        Thread-safe: called from the GUI thread; the ROS timer reads under
        the same lock. After the swap the model is touched only by the ROS
        thread, so the model itself needs no locking. Duck-typed on purpose
        — the bridge stays import-light."""
        with self._sim_gps_lock:
            self._sim_gps_model = model

    def _publish_sim_gps(self) -> None:
        """ROS-thread timer: synthesise one NavSatFix from the latest odom."""
        with self._sim_gps_lock:
            model = self._sim_gps_model
        if model is None or self._last_odom is None:
            return
        t, x, y = self._last_odom
        if time.monotonic() - t > 0.5:
            return  # Gazebo paused or starting: no fake fixes from stale poses
        lat, lon = model.fix_for(x, y)
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.latitude = float(lat)
        msg.longitude = float(lon)
        self._pub_sim_gps.publish(msg)

    # ---------------------------------------------------------- path service
    def request_mission_path(self, total_time: float, dt: float) -> None:
        """Asynchronously fetch the mission path (same call as path_publisher)."""
        if self._path_client is None:
            self._bus.mission_path_failed.emit("blueboat_interfaces missing")
            return
        with self._path_pending_lock:
            self._path_request_args = (total_time, dt)

    def cancel_mission_path_request(self) -> None:
        """Drop any pending or in-flight path request (the mission ended).

        Thread-safe: the GUI thread only raises a flag; the ROS-thread poll
        timer does the future teardown, so ``_path_future`` stays owned by
        one thread. Without this, a request left in flight by a dying
        ``path_generation`` would block every later request forever."""
        with self._path_pending_lock:
            self._path_request_args = None
            self._path_cancel = True

    def _poll_path_future(self) -> None:
        """ROS-thread timer: issue pending requests, harvest completed ones."""
        if self._path_client is None:
            return
        with self._path_pending_lock:
            cancel, self._path_cancel = self._path_cancel, False
        if cancel:
            if self._path_future is not None:
                self._path_future.cancel()
                self._path_future = None
            return  # even a completed reply belongs to the ended mission
        # Harvest — or time out — the in-flight request
        if self._path_future is not None:
            if self._path_future.done():
                future, self._path_future = self._path_future, None
                try:
                    result = future.result()
                except Exception as exc:  # noqa: BLE001
                    self._bus.mission_path_failed.emit(str(exc))
                    return
                poses = []
                for ps in result.path.poses:
                    q = ps.pose.orientation
                    yaw = float(np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                                           1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
                    poses.append((ps.pose.position.x, ps.pose.position.y, yaw))
                if not poses:
                    self._bus.mission_path_failed.emit(
                        "path service returned an empty path")
                    return
                self._bus.mission_path_received.emit(poses)
            elif (time.monotonic() - self._path_issued_t
                    > self._cfg.launch.path_request_timeout_s):
                # A hung call — path_generation died mid-request — must not
                # block every later request for the rest of the session.
                self._path_future.cancel()
                self._path_future = None
                self._bus.mission_path_failed.emit(
                    "no reply within "
                    f"{self._cfg.launch.path_request_timeout_s:.0f} s")
            return
        # Issue. The pending args stay queued until the request can actually
        # go out, so a newer GUI request always overwrites an older one.
        if not self._path_client.service_is_ready():
            return
        with self._path_pending_lock:
            args, self._path_request_args = self._path_request_args, None
        if args is None:
            return
        total_time, dt = args
        request = RequestPath.Request()
        n = int(total_time / dt) + 1
        request.path_request.data = np.linspace(0.0, total_time, n, dtype=float)
        self._path_future = self._path_client.call_async(request)
        self._path_issued_t = time.monotonic()

    # ------------------------------------------------------------ statistics
    def _emit_stats(self) -> None:
        now = time.monotonic()
        window = self._cfg.diagnostics.rate_window_s
        snapshot = {
            name: {
                "rate": st.rate_hz(now, window),
                "expected": st.expected_hz,
                "age": (now - st.last_rx) if st.last_rx is not None else None,
                "status": st.status(now),
            }
            for name, st in self._stats.items()
        }
        self._bus.topic_stats_updated.emit(snapshot)
