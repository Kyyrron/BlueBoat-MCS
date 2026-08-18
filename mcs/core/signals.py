"""The signal bus: single boundary between the ROS thread and the GUI thread.

Every ROS callback in :mod:`mcs.ros.bridge_node` terminates in a signal
emission here.  Qt delivers cross-thread signals through queued connections,
so slots connected on the GUI side always run in the GUI thread.  No widget
ever imports rclpy; no ROS callback ever touches a widget.
"""

from __future__ import annotations

from PySide6.QtCore import QObject, Signal


class SignalBus(QObject):
    """Application-wide signals. One instance, injected everywhere."""

    # ---- Telemetry (emitted from the ROS thread) -------------------------
    #: t_mono, pose [x, y, z, roll, pitch, yaw], twist [u, v, w, p, q, r]
    odom_received = Signal(float, object, object)
    #: t_mono, latitude, longitude
    gps_received = Signal(float, float, float)
    compass_received = Signal(float, float)   # (t, heading_deg CW-from-north)
    #: t_mono, connected, armed, mode string
    mavros_state_received = Signal(float, bool, bool, str)
    #: t_mono, pinger position in robot/body frame [x, y, z]
    pinger_body_received = Signal(float, object)
    #: t_mono (wall reception time) — raw underwater-GPS packet seen
    uw_gps_raw_received = Signal(float)
    #: t_mono, [t_ctrl, x, y, psi, x_d, y_d, psi_d, u1, u2]
    monitoring_received = Signal(float, object)
    #: t_mono, right thrust (N), left thrust (N)
    thruster_received = Signal(float, float, float)
    #: t_mono, ready flag
    controller_ready_received = Signal(float, bool)
    #: t_mono, mode string ('default' / 'override' / ...)
    param_mode_received = Signal(float, str)

    # ---- Mission path ----------------------------------------------------
    #: list[(x, y, yaw)] returned by the /path_request service
    mission_path_received = Signal(object)
    mission_path_failed = Signal(str)

    # ---- Diagnostics -----------------------------------------------------
    #: dict[topic_name, TopicStats]
    topic_stats_updated = Signal(object)
    #: informational message from the ROS layer
    ros_log = Signal(str)
    ros_started = Signal()
    ros_stopped = Signal()

    # ---- Mission launch process -------------------------------------------
    launch_state_changed = Signal(str)      # 'idle' | 'starting' | 'running' | 'stopping'
    launch_output = Signal(str)             # raw stdout/stderr lines

    # ---- High-level command feedback --------------------------------------
    command_sent = Signal(str)              # human-readable description
    estop_state_changed = Signal(str)       # 'idle'|'publishing'|'confirmed'|'timeout'
    #: Emitted once a safe-shutdown sequence (publish 'default' → confirm →
    #: terminate nodes) has fully completed. Used by Stop Mission and by the
    #: asynchronous application-exit path.
    shutdown_sequence_finished = Signal()
