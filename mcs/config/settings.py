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
    pinger_body: str = "/blueboat/pinger_coordinates"
    uw_gps_raw: str = "/uw_gps_data"
    monitoring: str = "/monitoring_data"
    thruster_input: str = "/thruster_input"
    controller_ready: str = "/blueboat/controller_ready"
    param_mode: str = "/blueboat/param_mode"

    # --- Publications (commands consumed by the existing stack) ---
    input_str: str = "/blueboat/input_str"
    manual_target: str = "/blueboat/manual_target"

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
        "/blueboat/pinger_coordinates": 1.0,
        "/uw_gps_data": 3.0,
        "/monitoring_data": 0.5,
        "/thruster_input": 0.5,
        "/blueboat/controller_ready": 30.0,
        "/blueboat/param_mode": 60.0,
    })
    stale_age_multiplier: float = 4.0
    expected_hz: dict[str, float] = field(default_factory=lambda: {
        "/blueboat/odom": 20.0,
        "/mavros/state": 1.0,
        "/mavros/global_position/global": 5.0,
        "/blueboat/pinger_coordinates": 20.0,
        "/uw_gps_data": 2.0,
        "/monitoring_data": 20.0,
        "/thruster_input": 20.0,
    })
    rate_window_s: float = 5.0
    update_period_s: float = 1.0


@dataclass
class LaunchConfig:
    """Defaults for the mission launch dialog and the launch subprocess."""

    package: str = "blueboat_control"
    launch_file: str = "BlueBoat_launch.py"
    # Gazebo simulation alternative (Sim_launch.py): declares only
    # robot_file / trajectory / controller_type and always starts
    # master_control + path_generation + path_publisher + simulation_interface
    # — no MAVROS, no robot_interface/param_set, no pinger.
    sim_launch_file: str = "Sim_launch.py"
    sim_robot_files: list[str] = field(default_factory=lambda: ["thrusters_ur"])
    sim_default_controller: str = "MPC"
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
    # Nodes considered "required" before mission controls are enabled.
    readiness_topics: list[str] = field(default_factory=lambda: [
        "/mavros/state", "/blueboat/odom",
    ])


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
    """Online odom<->GPS georeferencing parameters."""

    fit_window_s: float = 180.0     # use pairs from the last N seconds
    min_spread_m: float = 4.0       # boat must have moved this far to fit
    min_pairs: int = 25
    refit_period_s: float = 5.0
    max_residual_m: float = 6.0     # above this the fit is flagged low-quality


@dataclass
class DesignerConfig:
    """Mission Pattern Designer settings."""

    trajectories_dir: str = str(DEFAULT_CONFIG_DIR / "trajectories")
    grid_snap_m: float = 1.0          # Ctrl-drag / fixed-distance creation step
    waypoint_snap_px: float = 12.0    # snap-to-waypoint radius (screen px)
    sample_ds_m: float = 0.25         # spatial resolution of exported samples
    default_speed_mps: float = 0.5    # time-parameterization cruise speed
    preview_arrow_every_m: float = 8.0
    undo_depth: int = 100


@dataclass
class AppConfig:
    """Root configuration object."""

    topics: TopicsConfig = field(default_factory=TopicsConfig)
    diagnostics: DiagnosticsConfig = field(default_factory=DiagnosticsConfig)
    launch: LaunchConfig = field(default_factory=LaunchConfig)
    los: LosApproximation = field(default_factory=LosApproximation)
    map: MapConfig = field(default_factory=MapConfig)
    geo: GeoConfig = field(default_factory=GeoConfig)
    designer: DesignerConfig = field(default_factory=DesignerConfig)
    estop_confirm_timeout_s: float = 2.0
    estop_flush_delay_s: float = 0.3

    # ------------------------------------------------------------------ I/O
    @classmethod
    def load(cls, path: Path | None = None) -> "AppConfig":
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
