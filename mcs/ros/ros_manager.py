"""ROS2 lifecycle management.

Runs a single :class:`~mcs.ros.bridge_node.BridgeNode` inside a
``SingleThreadedExecutor`` on a dedicated Python thread.  The GUI thread
never blocks on ROS; the ROS thread never touches widgets.  All data flows
out through the :class:`~mcs.core.signals.SignalBus` (queued Qt signals),
and all commands flow in through the thread-safe publisher wrappers exposed
by the bridge node.

If ``rclpy`` (or ``mavros_msgs``) is not importable — e.g. developing the
GUI on a laptop without ROS sourced — the manager degrades gracefully: the
application starts, and the diagnostics panel reports the ROS layer as
unavailable.
"""

from __future__ import annotations

import logging
import threading

from mcs.config.settings import AppConfig
from mcs.core.signals import SignalBus

_LOG = logging.getLogger(__name__)

try:  # Optional at import time so the GUI can run without a sourced ROS env.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor

    ROS_AVAILABLE = True
except ImportError:  # pragma: no cover - environment dependent
    ROS_AVAILABLE = False


class RosManager:
    """Owns rclpy init/spin/shutdown and the bridge node."""

    def __init__(self, cfg: AppConfig, bus: SignalBus) -> None:
        self._cfg = cfg
        self._bus = bus
        self._thread: threading.Thread | None = None
        self._executor = None
        self.node = None  # BridgeNode | None

    @property
    def available(self) -> bool:
        return ROS_AVAILABLE

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------ API
    def start(self) -> None:
        if not ROS_AVAILABLE:
            self._bus.ros_log.emit(
                "rclpy not importable — running in GUI-only mode. "
                "Source your ROS2 environment and restart to connect."
            )
            return
        if self.running:
            return

        from mcs.ros.bridge_node import BridgeNode  # deferred: needs rclpy

        rclpy.init()
        self.node = BridgeNode(self._cfg, self._bus)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._thread = threading.Thread(
            target=self._spin, name="ros-executor", daemon=True
        )
        self._thread.start()
        self._bus.ros_started.emit()
        _LOG.info("ROS executor thread started")

    def stop(self) -> None:
        if not self.running:
            return
        assert self._executor is not None
        self._executor.shutdown(timeout_sec=2.0)
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001 - already shut down
            pass
        if self._thread:
            self._thread.join(timeout=3.0)
        self._thread = None
        self._bus.ros_stopped.emit()
        _LOG.info("ROS executor stopped")

    # ------------------------------------------------------------- internal
    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception as exc:  # noqa: BLE001 - surfaced to the operator
            _LOG.exception("ROS executor crashed")
            self._bus.ros_log.emit(f"ROS executor error: {exc}")
