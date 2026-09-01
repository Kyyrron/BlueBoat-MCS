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
from pathlib import Path

from PySide6.QtCore import QObject, QTimer

from mcs.config.settings import DEFAULT_CONFIG_DIR, AppConfig
from mcs.core.signals import SignalBus

_LOG = logging.getLogger(__name__)


@dataclass
class LaunchParameters:
    """Arguments of the selected launch file.

    Two launch targets exist, with different declared arguments:

    * ``BlueBoat_launch.py`` (real robot): ``enable_motors``, ``note``,
      ``controller_type``, ``trajectory``, ``use_pinger``.
    * ``Sim_launch.py`` (Gazebo): ``robot_file``, ``trajectory``,
      ``controller_type``, ``data_dir``, ``spawn_yaw`` — it always starts
      ``master_control`` (so the controller must be non-empty) and never
      MAVROS / robot_interface / param_set / pinger nodes.

    ``to_cli`` emits exactly the arguments the chosen file declares; passing
    real-robot arguments to the simulation launch would abort it.
    ``spawn_yaw:=`` is emitted only when ``spawn_yaw_rad`` is set, before
    ``extra_args`` — so an operator ``spawn_yaw:=`` in Extra args overrides
    it (in ``ros2 launch`` the last occurrence wins).
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
    # Simulation of a GPS-anchored mission: the station synthesises the GPS
    # feed itself and spawns the boat with this heading (radians ENU).
    gps_simulated: bool = False
    spawn_yaw_rad: float | None = None
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
            if self.spawn_yaw_rad is not None:
                args.append(f"spawn_yaw:={self.spawn_yaw_rad:.6f}")
        else:
            args = [
                f"enable_motors:={b(self.enable_motors)}",
                f"trajectory:={self.trajectory}",
                f"use_pinger:={b(self.use_pinger)}",
            ]
            if self.controller_type != "":
                args += [f"controller_type:={self.controller_type}"]
            if self.note != "":
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
        self._exit_poll_active = False
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
        # A stable working directory for the whole node tree: tools that
        # resolve relative paths (acados codegen historically dropped
        # acados_ocp.json + c_generated_code/ into whatever directory the
        # station happened to be started from) land their artifacts in one
        # known place — never this repository. The environment is inherited
        # as-is: the station runs from a sourced shell.
        launch_cwd = DEFAULT_CONFIG_DIR / "launch_cwd"
        try:
            launch_cwd.mkdir(parents=True, exist_ok=True)
        except OSError:  # unwritable home: fall back to inheriting our cwd
            launch_cwd = Path.cwd()
        try:
            self._proc = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                cwd=str(launch_cwd),
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
        # Watch for the process dying on its own (crash, bad launch argument):
        # without this the state machine stayed at 'starting'/'running' forever
        # and the Launch button never re-enabled.
        self._watch_exit()
        return True

    def notify_running(self) -> None:
        """Promote 'starting' -> 'running' once telemetry says the graph is up.

        Called from ``MainWindow._on_tick`` — there is no separate watcher.
        The condition is odometry flowing plus, on the real-robot graph only,
        an FCU-connected MAVROS state; ``Sim_launch.py`` has no MAVROS and is
        exempted there.  Promotion is a status indication: it does not gate
        the Launch/Stop controls, which are driven by the state itself.
        """
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
        # Poll for exit without blocking the GUI thread (no-op if the watch
        # started by start() is already running).
        self._watch_exit()

    # ------------------------------------------------------------- internal
    def _watch_exit(self) -> None:
        """Begin the GUI-thread exit watch; at most one chain at a time."""
        if self._exit_poll_active:
            return
        self._exit_poll_active = True
        self._poll_exit()

    def _poll_exit(self) -> None:
        if self.running:
            QTimer.singleShot(200, self._poll_exit)
            return
        self._exit_poll_active = False
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
