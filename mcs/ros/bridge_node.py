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

import logging
import threading
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R  # same dependency as the stack
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float32MultiArray, Float64, String

from mcs.config.settings import AppConfig
from mcs.core.signals import SignalBus

_LOG = logging.getLogger(__name__)

try:  # mavros_msgs may be absent on a dev laptop — degrade gracefully
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

        # ---- Publishers ----------------------------------------------------
        self._pub_input_str = self.create_publisher(String, t.input_str, 10)
        self._pub_manual_target = self.create_publisher(Float32MultiArray, t.manual_target, 10)

        # ---- Path service client -------------------------------------------
        self._path_client = None
        if BLUEBOAT_IFACES_AVAILABLE:
            self._path_client = self.create_client(RequestPath, t.path_request)
        else:
            bus.ros_log.emit(
                "blueboat_interfaces not available — mission path display disabled."
            )
        self._path_future = None
        self._path_pending_lock = threading.Lock()
        self._path_request_args: tuple[float, float] | None = None

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
        self._bus.odom_received.emit(t, pose, twist)

    def _on_compass(self, msg: Float64) -> None:
        # /mavros/global_position/compass_hdg: absolute heading in DEGREES,
        # clockwise from north (0=N, 90=E). Available immediately, unlike the
        # launch-zeroed odom yaw — used to orient the glyph from the start.
        t = self._mark(self._cfg.topics.compass_hdg)
        self._bus.compass_received.emit(t, float(msg.data))

    def _on_gps(self, msg: NavSatFix) -> None:
        t = self._mark(self._cfg.topics.gps)
        self._bus.gps_received.emit(t, float(msg.latitude), float(msg.longitude))

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

    def publish_manual_target(self, x: float, y: float) -> None:
        """Publish a manual target; (0, 0) resumes the original mission."""
        with self._pub_lock:
            msg = Float32MultiArray()
            msg.data = [float(x), float(y)]
            self._pub_manual_target.publish(msg)
        self._bus.command_sent.emit(f"manual_target ← [{x:.2f}, {y:.2f}]")

    # ---------------------------------------------------------- path service
    def request_mission_path(self, total_time: float, dt: float) -> None:
        """Asynchronously fetch the mission path (same call as path_publisher)."""
        if self._path_client is None:
            self._bus.mission_path_failed.emit("blueboat_interfaces missing")
            return
        with self._path_pending_lock:
            self._path_request_args = (total_time, dt)

    def _poll_path_future(self) -> None:
        """ROS-thread timer: issue pending requests, harvest completed ones."""
        if self._path_client is None:
            return
        # Harvest
        if self._path_future is not None and self._path_future.done():
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
            self._bus.mission_path_received.emit(poses)
            return
        # Issue
        with self._path_pending_lock:
            args, self._path_request_args = self._path_request_args, None
        if args is None or self._path_future is not None:
            if args is not None:  # a request was pending while another ran
                with self._path_pending_lock:
                    self._path_request_args = args
            return
        if not self._path_client.service_is_ready():
            with self._path_pending_lock:  # retry on next poll
                self._path_request_args = args
            return
        total_time, dt = args
        request = RequestPath.Request()
        n = int(total_time / dt) + 1
        request.path_request.data = np.linspace(0.0, total_time, n, dtype=float)
        self._path_future = self._path_client.call_async(request)

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
