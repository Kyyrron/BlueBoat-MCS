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
import sys
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

    Four launch targets exist, with different declared arguments:

    * ``BlueBoat_launch.py`` (real robot): ``enable_motors``, ``note``,
      ``controller_type``, ``trajectory``, ``use_pinger``, ``session_mode``.
      Every station launch of it is a **run session** (``session=True``):
      ``session_mode:=True`` and ``trajectory:=from_yaml:<session file>`` --
      the one file path_generation watches, which the station fills for each
      run (:mod:`mcs.ros.run_session`). ``session_source`` is the designer
      file the first run will execute ("" when following the pinger).
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
    * ``Replay_launch.py`` (v1.1, "From a ros2bag", selected when
      ``replay_bag`` is set): ``bag``, ``rate``, ``world_dir`` (optional, a
      personalized world). The boat moves kinematically along a recorded
      real-robot run; no controller, no sea state, no note, no companion.

    ``note`` is the operator's free-text log tag and reaches all three: it is
    what ``{date}-{note}-poslog.csv`` is named after, in real-robot runs and in
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
    # carries them (LaunchManager.start). None / null = calm: no sea args,
    # but the companion still runs (it carries the hull, see companion_command).
    sea: SeaChoice | None = None
    # Run session (real-robot launches): connect and idle; runs are started
    # and stopped from the station. See the class docstring.
    session: bool = False
    session_source: str = ""
    # Gazebo replay of a recorded run (a run folder or its bag directory) and
    # its initial playback rate. Implies ``simulation``.
    replay_bag: str = ""
    replay_rate: float = 1.0
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

        if self.replay_bag:
            args = [f"bag:={self.replay_bag}", f"rate:={self.replay_rate:g}"]
            if self.world_dir:
                args.append(f"world_dir:={self.world_dir}")
        elif self.simulation and self.world_dir:
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
            if self.session:
                args += ["session_mode:=True"]
        args += [f"{k}:={v}" for k, v in self.extra_args.items()]
        return args


def companion_command(cfg, params: LaunchParameters) -> list[str] | None:
    """The sea-state companion launch for the empty Gazebo world, or None.

    ``Sim_launch.py`` (BlueBoat-Control) declares no ``sea_*`` argument, so
    the sea state rides a second ``ros2 launch`` of the simulator's
    ``sea_state_launch.py`` against the stock world name. It runs for
    EVERY empty-world launch, a null choice (no current, calm) included,
    with no ``sea_*`` argument then (the simulator's own calm defaults):
    since 2026-10-09 the simulator's catamaran hydrostatics carry the hull
    in every run, so without the companion this path alone floated on
    Gazebo's URDF box — and a live "Change sea state…" had no listener.
    World mode needs no companion: ``full_mission_launch.py`` includes
    ``sea_state_launch.py`` itself. Module scope and Qt-free so the smoke
    test can pin it."""
    if params.replay_bag or not (params.simulation and not params.world_dir):
        # A replay feels no sea: the recording holds the real one, and the
        # replay description takes no force (blueboat_description replay:=true).
        return None
    return ["ros2", "launch", cfg.sea.companion_package,
            cfg.sea.companion_launch_file,
            f"world_name:={cfg.sea.stock_world_name}", *params.sea_args()]


def guardian_command(cfg, params: LaunchParameters) -> list[str] | None:
    """The link-guardian companion for a real-robot launch, or None.

    ``guardian.py watch`` (superproject ``link_guardian/laptop/``) switches the
    on-boat guardian ON for the session, moves HOME to each run start, reports
    on ``/link_guardian/status`` and obeys ``/link_guardian/command`` (the
    GUARDIAN box). Every real-robot launch gets it (by the user's order,
    2026-10-14: the boat must be able to return home whenever this laptop
    disappears); simulation and bag replay never do. ``cfg`` is the
    ``AppConfig``. Module scope and Qt-free so the smoke test can pin it; None
    also when the script is missing (the box then says so)."""
    if params.simulation or params.replay_bag or not cfg.guardian.enabled:
        return None
    script = Path(cfg.guardian.script).expanduser()
    if not script.is_file():
        return None
    # The station's own interpreter: rclpy comes from the sourced workspace,
    # exactly as for the station itself.
    return [sys.executable, "-u", str(script), "watch"]


def launch_target(cfg, params: LaunchParameters) -> tuple[str, str]:
    """(package, launch file) for the given parameters.

    Module-level and Qt-free so the smoke test can pin the three-way choice
    without a LaunchManager. *cfg* is the ``LaunchConfig`` dataclass.
    """
    if params.replay_bag:
        return cfg.package, cfg.replay_launch_file
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
        # Companion processes of the current launch, each in its own session:
        # ('sea', sea_state_launch.py) for an empty-world Gazebo run,
        # ('guardian', guardian.py watch) for a real-robot run.
        self._companions: list[tuple[str, subprocess.Popen[bytes]]] = []
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
        self._companions = []
        self._start_companion("sea", companion_command(self._cfg, params), launch_cwd,
                              "mission continues without sea state")
        guardian = guardian_command(self._cfg, params)
        self._start_companion("guardian", guardian, launch_cwd,
                              "the link guardian is NOT supervised from the station")
        if (guardian is None and not params.simulation and not params.replay_bag
                and self._cfg.guardian.enabled):
            self._bus.launch_output.emit(
                f"[guardian] {self._cfg.guardian.script} not found - the link guardian "
                "cannot be switched on from the station.")
        # Watch for the process dying on its own (crash, bad launch argument):
        # without this the state machine stayed at 'starting'/'running' forever
        # and the Launch button never re-enabled.
        self._watch_exit()
        return True

    def _start_companion(self, tag: str, cmd: list[str] | None, cwd: Path,
                         consequence: str) -> None:
        """Best effort: a companion never aborts a mission."""
        if cmd is None:
            return
        self._bus.launch_output.emit("$ " + " ".join(cmd) + f"   [{tag}]")
        try:
            proc = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                cwd=str(cwd), start_new_session=True)
        except OSError as exc:
            self._bus.launch_output.emit(
                f"[{tag}] companion launch failed ({exc}); {consequence}")
            return
        self._companions.append((tag, proc))
        threading.Thread(target=self._pump_companion, args=(tag, proc),
                         name=f"{tag}-output", daemon=True).start()

    def _pump_companion(self, tag: str, proc: subprocess.Popen[bytes]) -> None:
        if proc.stdout is None:
            return
        for raw in proc.stdout:
            line = raw.decode(errors="replace").rstrip()
            if line:
                self._bus.launch_output.emit(f"[{tag}] " + line)

    def _signal_companions(self, sig: signal.Signals, procs=None) -> None:
        """Signal the companions.

        `procs` is passed explicitly by the escalation chain, which must act on
        the companions it was started for and never on whatever happens to be
        in `self._companions` by the time it fires.
        """
        procs = [p for _, p in self._companions] if procs is None else procs
        for proc in procs:
            if proc.poll() is not None:
                continue
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
        self._signal_companions(signal.SIGINT)
        try:
            os.killpg(pgid, signal.SIGINT)
        except ProcessLookupError:
            self._finalise()
            return

        # The escalation fires seconds later, and the Launch button re-enables
        # the moment the state goes idle. Both lambdas therefore capture the
        # process they were armed for: reading self._proc / the companions at
        # fire time meant a relaunch inside the 12 s window killpg'd a recycled
        # pgid and SIGTERM'd the NEW sea companion. They are also cancelled by
        # _finalise() as soon as the tree is actually gone.
        doomed = self._proc
        doomed_companions = [p for _, p in self._companions]

        def escalate(sig: signal.Signals, label: str) -> None:
            if self._proc is not doomed:
                return          # that launch is long gone; this is a later one
            self._signal_companions(sig, doomed_companions)
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
        if self._companions:
            self._signal_companions(signal.SIGTERM)
            self._companions = []
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
