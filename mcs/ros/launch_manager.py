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
from mcs.core.sea import SeaChoice
from mcs.core.signals import SignalBus

_LOG = logging.getLogger(__name__)

#: Marker every simulated run's log note carries, so a Gazebo poslog can never
#: be mistaken for a field record. Matches ``Sim_launch.py``'s own ``note``
#: default, which is what an empty operator note still resolves to.
SIM_NOTE_PREFIX = "sim"


def sanitize_note(note: str) -> str:
    """The operator's log note, safe to put in a filename and on a CLI.

    The note goes verbatim into ``{date}-{note}-poslog.csv`` on both the real
    boat (``robot_interface``) and in simulation (``simulation_interface``),
    and rides there as one ``note:=`` token — so whitespace would split the
    token and a path separator would scatter the log. Runs of anything that
    is not a letter, digit, dot or underscore collapse to a single '-'.
    """
    out = "".join(c if (c.isalnum() or c in "._") else "-" for c in note)
    while "--" in out:
        out = out.replace("--", "-")
    return out.strip("-")


@dataclass
class LaunchParameters:
    """Arguments of the selected launch file.

    Three launch targets exist, with different declared arguments:

    * ``BlueBoat_launch.py`` (real robot): ``enable_motors``, ``note``,
      ``controller_type``, ``trajectory``, ``use_pinger``.
    * ``Sim_launch.py`` (empty Gazebo): ``robot_file``, ``trajectory``,
      ``controller_type``, ``data_dir``, ``note``, ``spawn_yaw`` — it always
      starts ``master_control`` (so the controller must be non-empty) and
      never MAVROS / robot_interface / param_set / pinger nodes.
    * ``blueboat_sss_sim full_mission_launch.py`` (personalized world,
      selected when ``world_dir`` is set): ``world_dir``, ``with_control``,
      ``trajectory_file``, ``controller_type``, ``note`` — same control graph
      as ``Sim_launch.py`` plus the simulated sonar and a mavros shim that
      publishes the GPS fixes from the world's own anchor. It declares NO
      ``robot_file``/``trajectory``/``spawn_yaw``.

    ``note`` is the operator's free-text log tag and reaches all three: it is
    what ``{date}-{note}-poslog.csv`` is named after, on the boat and in
    Gazebo alike. :meth:`wire_note` is the effective value — sanitised, and
    prefixed with ``sim`` for a simulated run so the two sets of logs stay
    distinguishable.

    ``to_cli`` emits exactly the arguments the chosen file declares; passing
    another target's arguments would abort ``ros2 launch``.
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
    # Personalized Gazebo world (blueboat_sss_sim full_mission_launch.py):
    # absolute world folder, set by the post-launch world-choice dialog;
    # "" = plain Sim_launch.py. In this mode the world's mavros shim owns
    # the GPS feed and the station's SimGps is left disarmed.
    world_dir: str = ""
    # Sea state (current + waves) of a simulated run: ``sea_*`` arguments of
    # full_mission_launch.py in world mode; in the empty Gazebo world
    # (Sim_launch.py declares none) a companion sea_state_launch.py process
    # carries them (LaunchManager.start). None / null = calm, no companion.
    sea: SeaChoice | None = None
    extra_args: dict[str, str] = field(default_factory=dict)

    def wire_note(self) -> str:
        """The ``note:=`` value, or "" when no argument should be emitted.

        Simulated runs always carry the ``sim`` marker — ``sim`` alone with
        no operator note (exactly ``Sim_launch.py``'s default, so the empty
        case stays byte-identical to before), ``sim-<note>`` with one.
        """
        note = sanitize_note(self.note)
        if not self.simulation:
            return note
        return f"{SIM_NOTE_PREFIX}-{note}" if note else SIM_NOTE_PREFIX

    def sea_args(self) -> list[str]:
        return self.sea.launch_args() if (self.sea and not self.sea.is_null) else []

    def to_cli(self) -> list[str]:
        def b(v: bool) -> str:
            return "True" if v else "False"

        if self.simulation and self.world_dir:
            args = [
                f"world_dir:={self.world_dir}",
                "with_control:=true",
                f"trajectory_file:={self.gps_deployed_target}",
                f"controller_type:={self.controller_type}",
                f"note:={self.wire_note()}",
                *self.sea_args(),          # before extra_args: an override wins
            ]
        elif self.simulation:
            args = [
                f"robot_file:={self.robot_file}",
                f"trajectory:={self.trajectory}",
                f"controller_type:={self.controller_type}",
                f"note:={self.wire_note()}",
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
            if self.wire_note() != "":
                args += [f"note:={self.wire_note()}"]
        args += [f"{k}:={v}" for k, v in self.extra_args.items()]
        return args


def companion_command(cfg, params: LaunchParameters) -> list[str] | None:
    """The sea-state companion launch for the empty Gazebo world, or None.

    ``Sim_launch.py`` (BlueBoat-Control) declares no ``sea_*`` argument, so
    the sea state rides a second ``ros2 launch`` of the simulator's
    ``sea_state_launch.py`` against the stock world name; a null choice
    (no current, calm) starts nothing, keeping that path byte-identical to
    today's. World mode needs no companion: ``full_mission_launch.py``
    includes ``sea_state_launch.py`` itself. Module scope and Qt-free so
    the smoke test can pin it."""
    if not (params.simulation and not params.world_dir and params.sea
            and not params.sea.is_null):
        return None
    return ["ros2", "launch", cfg.sea.companion_package,
            cfg.sea.companion_launch_file,
            f"world_name:={cfg.sea.stock_world_name}", *params.sea.launch_args()]


def launch_target(cfg, params: LaunchParameters) -> tuple[str, str]:
    """(package, launch file) for the given parameters.

    Module-level and Qt-free so the smoke test can pin the three-way choice
    without a LaunchManager. *cfg* is the ``LaunchConfig`` dataclass.
    """
    if params.simulation and params.world_dir:
        return cfg.sim_world_package, cfg.sim_world_launch_file
    if params.simulation:
        return cfg.package, cfg.sim_launch_file
    return cfg.package, cfg.launch_file


class LaunchManager(QObject):
    """Owns the ``ros2 launch`` child process."""

    def __init__(self, cfg: AppConfig, bus: SignalBus) -> None:
        super().__init__()
        self._cfg = cfg
        self._bus = bus
        self._proc: subprocess.Popen[bytes] | None = None
        self._sea_proc: subprocess.Popen[bytes] | None = None
        self._reader: threading.Thread | None = None
        self._state = "idle"
        self._exit_poll_active = False
        # Pending SIGTERM/SIGKILL escalation, held so it can be disarmed the
        # moment the tree exits rather than firing into the next launch.
        self._escalation_timers: list[QTimer] = []
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
        package, launch_file = launch_target(self._cfg.launch, params)
        cmd = [
            "ros2", "launch",
            package, launch_file,
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
        self._start_companion(params, launch_cwd)
        # Watch for the process dying on its own (crash, bad launch argument):
        # without this the state machine stayed at 'starting'/'running' forever
        # and the Launch button never re-enabled.
        self._watch_exit()
        return True

    def _start_companion(self, params: LaunchParameters, cwd: Path) -> None:
        """Best effort: the sea-state companion never aborts a mission."""
        cmd = companion_command(self._cfg, params)
        if cmd is None:
            return
        self._bus.launch_output.emit("$ " + " ".join(cmd) + "   [sea]")
        try:
            self._sea_proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                cwd=str(cwd), start_new_session=True)
        except OSError as exc:
            self._bus.launch_output.emit(
                f"[sea] companion launch failed ({exc}); mission continues "
                "without sea state")
            self._sea_proc = None
            return
        threading.Thread(target=self._pump_companion, name="sea-output",
                         daemon=True).start()

    def _pump_companion(self) -> None:
        proc = self._sea_proc
        if proc is None or proc.stdout is None:
            return
        for raw in proc.stdout:
            line = raw.decode(errors="replace").rstrip()
            if line:
                self._bus.launch_output.emit("[sea] " + line)

    def _signal_companion(self, sig: signal.Signals, proc=None) -> None:
        """Signal the sea-state companion.

        `proc` is passed explicitly by the escalation chain, which must act on
        the companion it was started for and never on whatever happens to be in
        `self._sea_proc` by the time it fires.
        """
        proc = self._sea_proc if proc is None else proc
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except ProcessLookupError:
            pass

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
        self._signal_companion(signal.SIGINT)
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            self._finalise()
            return

        # The escalation fires seconds later, and the Launch button re-enables
        # the moment the state goes idle. Both lambdas therefore capture the
        # process they were armed for: reading self._proc / self._sea_proc at
        # fire time meant a relaunch inside the 12 s window killpg'd a recycled
        # pgid and SIGTERM'd the NEW sea companion. They are also cancelled by
        # _finalise() as soon as the tree is actually gone.
        doomed, doomed_sea = self._proc, self._sea_proc

        def escalate(sig: signal.Signals, label: str) -> None:
            if self._proc is not doomed:
                return          # that launch is long gone; this is a later one
            self._signal_companion(sig, doomed_sea)
            if self.running:
                self._bus.launch_output.emit(f"Nodes still alive — sending {label}.")
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    pass

        self._cancel_escalation()
        for delay_s, sig, label in (
                (self._cfg.launch.sigint_timeout_s, signal.SIGTERM, "SIGTERM"),
                (self._cfg.launch.sigint_timeout_s
                 + self._cfg.launch.sigterm_timeout_s, signal.SIGKILL, "SIGKILL")):
            timer = QTimer(self)
            timer.setSingleShot(True)
            timer.timeout.connect(
                lambda s=sig, lbl=label: escalate(s, lbl))
            timer.start(int(delay_s * 1000))
            self._escalation_timers.append(timer)
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

    def _cancel_escalation(self) -> None:
        """Disarm any pending SIGTERM/SIGKILL escalation."""
        for timer in self._escalation_timers:
            timer.stop()
        self._escalation_timers.clear()

    def _finalise(self) -> None:
        self._cancel_escalation()
        if self._proc is not None:
            code = self._proc.poll()
            self._bus.launch_output.emit(f"Mission process exited (code {code}).")
        self._proc = None
        if self._sea_proc is not None:
            self._signal_companion(signal.SIGTERM)
            self._sea_proc = None
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
