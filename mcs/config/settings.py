"""Typed application configuration.

Every ROS topic name, monitoring threshold, controller-approximation gain and
visual parameter used anywhere in the application is defined here and can be
overridden from a JSON file (``~/.config/blueboat_mcs/config.json`` by default,
or a path given on the command line).

Nothing else in the code base hard-codes a topic name.
"""

from __future__ import annotations

import dataclasses
import json
import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

_LOG = logging.getLogger(__name__)

DEFAULT_CONFIG_DIR = Path.home() / ".config" / "blueboat_mcs"
DEFAULT_CONFIG_FILE = DEFAULT_CONFIG_DIR / "config.json"


@dataclass
class TopicsConfig:
    """Names of every ROS2 topic / service the station talks to."""

    # --- Subscriptions (data produced by the existing stack) ---
    odom: str = "/blueboat/odom"
    gps: str = "/mavros/global_position/global"
    compass_hdg: str = "/mavros/global_position/compass_hdg"
    mavros_state: str = "/mavros/state"
    battery: str = "/mavros/battery"
    pinger_body: str = "/blueboat/pinger_coordinates"
    uw_gps_raw: str = "/uw_gps_data"
    monitoring: str = "/monitoring_data"
    thruster_input: str = "/thruster_input"
    controller_ready: str = "/blueboat/controller_ready"
    param_mode: str = "/blueboat/param_mode"
    # Run sessions (real-robot launches only): master_control's run state
    # (String JSON, heartbeated 1 Hz) and robot_interface's per-run log state.
    run_state: str = "/blueboat/run_state"
    run_log: str = "/blueboat/run_log"

    # --- Simulation only (BlueBoat-SSS-Sim sea_state_node): the live sea
    # state (String JSON, latched, 2 Hz) and the command that changes it.
    sea_state: str = "/sim/sea_state"
    # --- Bag replay only (blueboat_control bag_replay, v1.1): the replay's
    # state (String JSON, latched, 2 Hz) and its pause / rate command.
    replay_state: str = "/replay/state"

    # --- Publications (commands consumed by the existing stack) ---
    input_str: str = "/blueboat/input_str"
    manual_target: str = "/blueboat/manual_target"
    sea_state_command: str = "/sim/sea_state/command"
    replay_command: str = "/replay/command"
    # Run-session commands to master_control (String JSON: configure / start /
    # stop) and the arrow-key thrust stream ([right, left] N, [] = release).
    run_command: str = "/blueboat/run_command"
    arrow_thrust: str = "/blueboat/arrow_thrust"

    # --- Services ---
    path_request: str = "/path_request"


@dataclass
class DiagnosticsConfig:
    """Health thresholds for the topic monitor.

    A topic is *OK* (green) while its message age is below ``warn_age_s``,
    *WARN* (orange) below ``stale_age_s`` and *STALE* (red) beyond that.
    ``expected_hz`` is informative only (shown next to the measured rate).
    """

    warn_age_s: dict[str, float] = field(default_factory=lambda: {
        "/blueboat/odom": 0.5,
        "/mavros/state": 2.0,
        "/mavros/global_position/global": 2.0,
        "/mavros/global_position/compass_hdg": 2.0,
        "/mavros/battery": 5.0,
        "/blueboat/pinger_coordinates": 1.0,
        "/uw_gps_data": 3.0,
        "/monitoring_data": 0.5,
        "/thruster_input": 0.5,
        "/blueboat/controller_ready": 30.0,
        "/blueboat/param_mode": 60.0,
        "/sim/sea_state": 3.0,
        "/blueboat/run_state": 3.0,
        "/blueboat/run_log": 3.0,
    })
    stale_age_multiplier: float = 4.0
    expected_hz: dict[str, float] = field(default_factory=lambda: {
        "/blueboat/odom": 20.0,
        "/mavros/state": 1.0,
        "/mavros/global_position/global": 5.0,
        "/mavros/global_position/compass_hdg": 5.0,
        "/mavros/battery": 1.0,
        "/blueboat/pinger_coordinates": 20.0,
        "/uw_gps_data": 2.0,
        "/monitoring_data": 20.0,
        "/thruster_input": 20.0,
        "/sim/sea_state": 2.0,
        "/blueboat/run_state": 1.0,
        "/blueboat/run_log": 1.0,
    })
    rate_window_s: float = 5.0
    update_period_s: float = 1.0


@dataclass
class LaunchConfig:
    """Defaults for the mission launch dialog and the launch subprocess."""

    package: str = "blueboat_control"
    launch_file: str = "BlueBoat_launch.py"
    # Gazebo simulation alternative (Sim_launch.py): declares
    # robot_file / trajectory / controller_type / data_dir / spawn_yaw and
    # always starts master_control + path_generation + path_publisher +
    # simulation_interface — no MAVROS, no robot_interface/param_set,
    # no pinger.
    sim_launch_file: str = "Sim_launch.py"
    # Personalized Gazebo worlds (BlueBoat-SSS-Sim, ~/worlds/<path>/<world>/):
    # full_mission_launch.py declares world_dir / with_control /
    # trajectory_file / controller_type (+ use_stock_world, quiet,
    # with_mavros_shim, sim_origin_*, acquisition args) and NO spawn_yaw.
    # Its mavros shim publishes the GPS fixes from the world's own anchor,
    # so the station's SimGps stays disarmed in this mode.
    sim_world_package: str = "blueboat_sss_sim"
    sim_world_launch_file: str = "full_mission_launch.py"
    # Gazebo replay of a recorded real-robot run (v1.1, blueboat_control):
    # declares bag / world_dir / rate / heave / start_paused / gui. The boat
    # is moved kinematically from the bag; no controller, no sea state.
    replay_launch_file: str = "Replay_launch.py"
    worlds_root: str = str(Path.home() / "worlds")
    sim_robot_files: list[str] = field(default_factory=lambda: ["thrusters_ur"])
    sim_default_controller: str = "MPC"
    # Gazebo spawn heading, DEGREES ENU (0 = east), emitted as Sim_launch.py's
    # spawn_yaw:= (radians) for a simulated GPS-anchored mission. The boat
    # always spawns at world (0, 0); this is the only spawn degree of freedom
    # the station controls, and it is FIXED rather than random so two runs of
    # the same mission start identically. full_mission_launch.py declares no
    # spawn_yaw (world (0, 0), heading east). To start from another heading
    # for one run, pass spawn_yaw:=<radians> in Extra args — it is appended
    # after this one and ros2 launch takes the last occurrence.
    sim_spawn_yaw_deg: float = 0.0
    # Controller choices. A real-robot launch is a run session and needs a
    # controller (manual target and arrow keys go through master_control), so
    # the empty "robot interface only" entry is offered for nothing any more;
    # it stays here only so an old config.json naming it still loads.
    controllers: list[str] = field(default_factory=lambda: ["", "PID", "LoS", "MPC"])
    trajectories: list[str] = field(default_factory=lambda: [
        "station_keeping", "circle", "straight_line", "sin",
        "fsin", "square", "kin_square", "seabed_scanning",
    ])
    sigint_timeout_s: float = 8.0
    sigterm_timeout_s: float = 4.0
    # Mission-path preview: the station requests the whole path from the
    # path_generation SERVICE (/path_request) — never via path_publisher —
    # over this horizon. For from_yaml trajectories the YAML's own
    # duration_s overrides it automatically.
    path_preview_total_time_s: float = 120.0
    path_preview_dt_s: float = 0.5
    # An in-flight /path_request with no reply after this long is dropped
    # (path_generation died mid-call, e.g. Stop/E-STOP while requesting) —
    # without the deadline the stuck future would block every later request
    # for the rest of the app session. A dropped or failed request is
    # re-armed up to max_retries times, spaced by retry_delay.
    path_request_timeout_s: float = 10.0
    path_request_max_retries: int = 3
    path_request_retry_delay_s: float = 2.0
    # Run sessions. The ONE trajectory file path_generation watches in a
    # real-robot session (trajectory:=from_yaml:<this>): the station writes
    # each run's path here -- GPS-deployed, or placed at the boat when the run
    # starts. A hidden directory, so it never collides with a designer mission
    # name and the mission lists never show it.
    session_trajectory_file: str = str(
        DEFAULT_CONFIG_DIR / "trajectories" / ".session" / "run.yaml")


@dataclass
class RunConfig:
    """Run-session behaviour (real-robot launches)."""

    # master_control's /blueboat/run_state older than this = not connected
    # (it heartbeats at 1 Hz).
    state_fresh_s: float = 3.0
    # A configure / start / stop not acknowledged within this is reported.
    ack_timeout_s: float = 3.0
    # A started run whose position log robot_interface has not confirmed by
    # then is flagged in the RUN box (field data would be missing).
    log_confirm_timeout_s: float = 3.0


@dataclass
class ReplayConfig:
    """Gazebo replay of a recorded run ("From a ros2bag", v1.1)."""

    # Where the launch dialog's folder picker opens: the run folders that
    # robot_interface files a bag into (Robot_data/<stem>/<..>-bag/).
    bag_root: str = str(Path.home() / "ros2_ws" / "data" / "Robot_data")
    rates: list[float] = field(default_factory=lambda: [0.25, 0.5, 1.0, 2.0, 4.0])
    default_rate: float = 1.0
    # /replay/state older than this = the replay node is not answering.
    state_fresh_s: float = 3.0
    # GPS points sampled from the bag to choose eligible worlds.
    world_track_points: int = 400


@dataclass
class ArrowConfig:
    """Arrow-key thrust control (the floating box at the map's bottom-right)."""

    max_power_n: float = 20.0       # slider range 0..max (master_control's
                                    # thrust_limit saturates above 20 N anyway)
    default_power_n: float = 10.0
    turn_n: float = 5.0             # default Rotation: +/- added per side by
                                    # Left / Right (the name predates the slider)
    min_turn_n: float = 3.0         # Rotation slider range
    max_turn_n: float = 15.0
    publish_hz: float = 20.0        # stream rate; master_control's dead-man
                                    # (arrow_timeout, 0.5 s) zeroes a gap


@dataclass
class SeaConfig:
    """Sea state (current + waves) for simulated missions.

    The physics and the preset table live in BlueBoat-SSS-Sim
    (``sea_state_node``, ``config/sea_states.yaml``); the station reads the
    installed table (``presets_file`` "" = auto-locate through
    ``$AMENT_PREFIX_PATH`` / ``~/ros2_ws/install``) and passes the choice as
    ``sea_*`` launch arguments — to ``full_mission_launch.py`` directly in a
    personalized world, and through a companion
    ``ros2 launch blueboat_sss_sim sea_state_launch.py world_name:=ocean``
    process next to ``Sim_launch.py`` in the empty Gazebo world (that
    launch file declares no sea arguments). The companion runs for every
    empty-world launch, a null choice (no current, calm) included — with no
    sea argument then — because the simulator's catamaran hydrostatics carry
    the hull in every run (since 2026-10-09).
    """

    presets_file: str = ""
    schedules_dir: str = str(Path.home() / ".config" / "blueboat_mcs" / "sea_states")
    default_current: str = "none"
    default_waves: str = "calm"
    companion_package: str = "blueboat_sss_sim"
    companion_launch_file: str = "sea_state_launch.py"
    stock_world_name: str = "ocean"
    ramp_s: float = 20.0          # live changes ramp over this many seconds


@dataclass
class LosApproximation:
    """Parameters of the *display-only* LoS future-path approximation.

    These mirror the constants in ``master_control.solve_LoS`` and translate
    the thrust law into an approximate kinematic response.  They influence
    nothing but the dashed prediction line on the map.
    """

    k_v: float = 0.15               # same as master_control
    v_max: float = 1.2              # m/s, saturation of the boat response
    thrust_to_speed: float = 0.35   # m/s per Newton of common-mode thrust
    k_psi: float = 10.0             # same as master_control
    r_max: float = 0.8              # rad/s, saturation of the yaw response
    thrust_to_yaw_rate: float = 0.08  # rad/s per Newton of differential thrust
    sim_dt: float = 0.25            # s, integration step
    horizon_s: float = 120.0        # s, max simulated duration
    reached_distance_m: float = 1.0  # popup + prediction stop threshold


@dataclass
class MapConfig:
    """Map rendering configuration."""

    tile_url: str = (
        "https://server.arcgisonline.com/ArcGIS/rest/services/"
        "World_Imagery/MapServer/tile/{z}/{y}/{x}"
    )
    tile_cache_dir: str = str(DEFAULT_CONFIG_DIR / "tile_cache")
    tile_max_zoom: int = 19
    trajectory_max_points_drawn: int = 20_000  # decimation limit per repaint
    ui_refresh_hz: float = 10.0


@dataclass
class GeoConfig:
    """Online odom<->GPS georeferencing parameters (translation-only:
    /blueboat/odom is local ENU, so no rotation is estimated and no vehicle
    motion is needed — the anchor converges from the first few fixes)."""

    fit_window_s: float = 180.0     # use pairs from the last N seconds
    min_pairs: int = 5              # ~1 s of GPS before the anchor is trusted
    refit_period_s: float = 5.0     # throttle once the anchor is supported
    max_residual_m: float = 6.0     # above this the fit is flagged low-quality


@dataclass
class SimGpsConfig:
    """Simulated robot GPS for Gazebo runs of GPS-anchored missions.

    The bridge node synthesises NavSatFix messages from the sim odom (world
    metres -> lat/lon about the receiver origin, pure translation). The
    receiver origin — the boat's FIRST fix — is placed ``offset_north_m``
    north of the mission's first point, so the anchor/deferred-deploy
    pipeline runs exactly as on real water. Non-anchored sim runs simulate
    no GPS at all."""

    offset_north_m: float = 10.0   # first fix this far north of the path start
    noise_sigma_m: float = 0.4     # Gaussian noise per EN axis, metres
    rate_hz: float = 5.0           # matches the real MAVROS fix rate
    # Seed of that noise. FIXED, never None: the fix noise feeds the anchor
    # estimate, which is where the deployed path lands relative to the boat,
    # so an unseeded generator would make the same mission start a metre or
    # so away from one run to the next. Change it to draw a different (still
    # repeatable) realisation.
    noise_seed: int = 20260913


@dataclass
class DesignerConfig:
    """Survey Pattern Designer settings."""

    trajectories_dir: str = str(DEFAULT_CONFIG_DIR / "trajectories")
    grid_snap_m: float = 1.0          # Ctrl-drag / fixed-distance creation step
    waypoint_snap_px: float = 12.0    # snap-to-waypoint radius (screen px)
    sample_ds_m: float = 0.25         # spatial resolution of exported samples
    default_speed_mps: float = 0.5    # time-parameterization cruise speed
    preview_arrow_every_m: float = 8.0
    undo_depth: int = 100
    # "Does the mission start at the world origin, heading +x?" — the world
    # frame is local ENU (origin = launch point, +x = East), so an aligned
    # mission starts at the boat's launch position heading EAST. Shared by
    # the designer's Align to Start and by the launch dialog's badge, so the
    # two can never disagree about the same file. GPS-anchored missions are
    # geographically fixed and exempt.
    start_align_tol_m: float = 0.05
    start_align_tol_deg: float = 2.0


@dataclass
class AppConfig:
    """Root configuration object."""

    topics: TopicsConfig = field(default_factory=TopicsConfig)
    diagnostics: DiagnosticsConfig = field(default_factory=DiagnosticsConfig)
    launch: LaunchConfig = field(default_factory=LaunchConfig)
    los: LosApproximation = field(default_factory=LosApproximation)
    map: MapConfig = field(default_factory=MapConfig)
    geo: GeoConfig = field(default_factory=GeoConfig)
    sim_gps: SimGpsConfig = field(default_factory=SimGpsConfig)
    designer: DesignerConfig = field(default_factory=DesignerConfig)
    sea: SeaConfig = field(default_factory=SeaConfig)
    run: RunConfig = field(default_factory=RunConfig)
    arrows: ArrowConfig = field(default_factory=ArrowConfig)
    replay: ReplayConfig = field(default_factory=ReplayConfig)
    estop_confirm_timeout_s: float = 2.0
    estop_flush_delay_s: float = 0.3

    # ------------------------------------------------------------------ I/O
    @classmethod
    def load(cls, path: Path | None = None) -> AppConfig:
        """Load the configuration, merging a JSON override file if present."""
        cfg = cls()
        path = path or DEFAULT_CONFIG_FILE
        if path.exists():
            try:
                overrides = json.loads(path.read_text())
                _merge_dataclass(cfg, overrides)
                _LOG.info("Loaded configuration overrides from %s", path)
            except (json.JSONDecodeError, OSError) as exc:
                _LOG.warning("Could not read config %s: %s — using defaults", path, exc)
        return cfg

    def save(self, path: Path | None = None) -> None:
        path = path or DEFAULT_CONFIG_FILE
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(dataclasses.asdict(self), indent=2))


def _merge_dataclass(obj: Any, overrides: dict[str, Any]) -> None:
    """Recursively apply a dict of overrides onto a dataclass instance."""
    for key, value in overrides.items():
        if not hasattr(obj, key):
            _LOG.warning("Unknown config key ignored: %s", key)
            continue
        current = getattr(obj, key)
        if dataclasses.is_dataclass(current) and isinstance(value, dict):
            _merge_dataclass(current, value)
        else:
            setattr(obj, key, value)
