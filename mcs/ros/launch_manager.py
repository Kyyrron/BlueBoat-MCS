"""Mission launch process management.

Runs ``ros2 launch <package> <file> arg:=value ...`` as a child process and
manages its lifecycle:

* **Start** — spawns the process in its own session so the whole node tree
  can be signalled as a group; stdout/stderr are streamed to the GUI console.
* **Graceful stop** — sends **SIGINT** first (``ros2 launch`` forwards it to
  every node for a clean shutdown, exactly like Ctrl-C in a terminal), then
  escalates to SIGTERM and finally SIGKILL only after configurable timeouts.
* The application itself always stays alive; the mission may be relaunched.

The Emergency-Stop *sequencing* (publish ``default`` before any termination)
is implemented in :mod:`mcs.ros.command_center`; this class only knows how
to start and stop the process tree.
"""

from __future__ import annotations

import logging
import os
import signal
import subprocess
import threading
from dataclasses import dataclass, field

from PySide6.QtCore import QObject, QTimer

from mcs.config.settings import AppConfig
from mcs.core.signals import SignalBus

_LOG = logging.getLogger(__name__)


@dataclass
class LaunchParameters:
    """Arguments of the selected launch file.

    Two launch targets exist, with different declared arguments:

    * ``BlueBoat_launch.py`` (real robot): ``enable_motors``, ``note``,
      ``controller_type``, ``trajectory``, ``use_pinger``.
    * ``Sim_launch.py`` (Gazebo): ``robot_file``, ``trajectory``,
      ``controller_type`` only — it always starts ``master_control`` (so the
      controller must be non-empty) and never MAVROS / robot_interface /
      param_set / pinger nodes.

    ``to_cli`` emits exactly the arguments the chosen file declares; passing
    real-robot arguments to the simulation launch would abort it.
    """

    enable_motors: bool = False
    note: str = ""
    controller_type: str = ""
    trajectory: str = "station_keeping"
    use_pinger: bool = False
    simulation: bool = False
    robot_file: str = "thrusters_ur"
    # GPS-anchored custom paths: source design YAML + the deployed file the
    # station writes once the run's georeference is established (the
    # 'trajectory' argument already points path_generation at the latter).
    gps_anchored_source: str = ""
    gps_deployed_target: str = ""
    extra_args: dict[str, str] = field(default_factory=dict)

    def to_cli(self) -> list[str]:
        def b(v: bool) -> str:
            return "True" if v else "False"

        if self.simulation:
            args = [
                f"robot_file:={self.robot_file}",
                f"trajectory:={self.trajectory}",
                f"controller_type:={self.controller_type}",
            ]
        else:
            args = [
                f"enable_motors:={b(self.enable_motors)}",
                f"trajectory:={self.trajectory}",
                f"use_pinger:={b(self.use_pinger)}",
            ]
            if not self.controller_type == "":
                args += [f"controller_type:={self.controller_type}"]
            if not self.note == "":
                args += [f"note:={self.note}"]
        args += [f"{k}:={v}" for k, v in self.extra_args.items()]
        return args


class LaunchManager(QObject):
    """Owns the ``ros2 launch`` child process."""

    def __init__(self, cfg: AppConfig, bus: SignalBus) -> None:
        super().__init__()
        self._cfg = cfg
        self._bus = bus
        self._proc: subprocess.Popen[bytes] | None = None
        self._reader: threading.Thread | None = None
        self._state = "idle"
        self.last_parameters: LaunchParameters | None = None

    # ------------------------------------------------------------------ API
    @property
    def state(self) -> str:
        return self._state

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, params: LaunchParameters) -> bool:
        if self.running:
            self._bus.launch_output.emit("A mission is already running.")
            return False
        launch_file = (self._cfg.launch.sim_launch_file if params.simulation
                       else self._cfg.launch.launch_file)
        cmd = [
            "ros2", "launch",
            self._cfg.launch.package, launch_file,
            *params.to_cli(),
        ]
        self._bus.launch_output.emit("$ " + " ".join(cmd))
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,  # own process group -> group signalling
            )
        except FileNotFoundError:
            self._bus.launch_output.emit(
                "ERROR: 'ros2' not found. Source your ROS2 environment before "
                "starting the station."
            )
            self._proc = None
            return False
        self.last_parameters = params
        self._set_state("starting")
        self._reader = threading.Thread(
            target=self._pump_output, name="launch-output", daemon=True
        )
        self._reader.start()
        return True

    def notify_running(self) -> None:
        """Called by the readiness watcher once required nodes are up."""
        if self.running and self._state == "starting":
            self._set_state("running")

    def stop(self) -> None:
        """Graceful SIGINT -> SIGTERM -> SIGKILL shutdown of the node tree."""
        if not self.running:
            self._finalise()
            return
        assert self._proc is not None
        self._set_state("stopping")
        pgid = os.getpgid(self._proc.pid)
        self._bus.launch_output.emit("Stopping mission (SIGINT to launch group)…")
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            self._finalise()
            return

        def escalate(sig: signal.Signals, label: str) -> None:
            if self.running:
                self._bus.launch_output.emit(f"Nodes still alive — sending {label}.")
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    pass

        QTimer.singleShot(
            int(self._cfg.launch.sigint_timeout_s * 1000),
            lambda: escalate(signal.SIGTERM, "SIGTERM"),
        )
        QTimer.singleShot(
            int((self._cfg.launch.sigint_timeout_s
                 + self._cfg.launch.sigterm_timeout_s) * 1000),
            lambda: escalate(signal.SIGKILL, "SIGKILL"),
        )
        # Poll for exit without blocking the GUI thread.
        self._poll_exit()

    # ------------------------------------------------------------- internal
    def _poll_exit(self) -> None:
        if self.running:
            QTimer.singleShot(200, self._poll_exit)
            return
        self._finalise()

    def _finalise(self) -> None:
        if self._proc is not None:
            code = self._proc.poll()
            self._bus.launch_output.emit(f"Mission process exited (code {code}).")
        self._proc = None
        self._set_state("idle")

    def _pump_output(self) -> None:
        assert self._proc is not None and self._proc.stdout is not None
        for raw in self._proc.stdout:
            line = raw.decode(errors="replace").rstrip()
            if line:
                self._bus.launch_output.emit(line)
        # Process ended on its own (crash or completion) — reflect it.
        if self._state not in ("idle", "stopping"):
            self._bus.launch_output.emit("Launch process terminated.")

    def _set_state(self, state: str) -> None:
        self._state = state
        self._bus.launch_state_changed.emit(state)
