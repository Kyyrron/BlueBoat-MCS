# BlueBoat-MCS — Mission Control Station

Operator ground-station GUI for a BlueRobotics BlueBoat USV used in a master's
thesis on aspect-aware side-scan sonar survey. Runs on the basestation laptop,
**not** on the boat.

**The station supervises and commands. It performs no control computation and
duplicates no logic that already exists on a ROS topic.** Every number it shows
comes from a topic or service published by the existing `blueboat_control`
stack. When something looks wrong on screen, the first question is whether the
producing node is right, not whether the GUI should compensate.

Build system: standalone Python (no colcon, no `setup.py`). The package is run
from the repo root; ROS2 comes from the sourced workspace, never from pip.

---

## 1. Layout

```
<repo root>/
├── mcs/                   the application (importable package)
│   ├── main.py            bootstrap; `run.py` is a thin launcher
│   ├── config/settings.py ALL topics, thresholds, gains, paths (dataclasses)
│   ├── core/              signals.py (thread boundary), series.py
│   │                      (append-only growable series, never wrapped —
│   │                      recording is unbounded), geo.py (odom↔GPS),
│   │                      los_predictor.py
│   ├── models/store.py    DataStore: the single in-memory state snapshot
│   ├── ros/               ros_manager (rclpy thread), bridge_node (all
│   │                      subs/pubs/service), launch_manager, command_center
│   ├── gui/               main_window, left/right/bottom panels, console,
│   │                      mission_stats, theme, widgets, map/, plot/, dialogs/
│   ├── designer/          Survey Pattern Designer (widget-free logic + Qt UI)
│   └── utils/             empty package (no modules)
├── docs/                  01..08 + HEADING_AND_MAP_ALIGNMENT.md
├── smoke_test.py          headless regression script (see §7)
├── build.sh               colcon build in ~/ros2_ws, then `python3 run.py`
├── ruff.toml              lint config (see §7)
├── requirements.txt       runtime deps only
├── requirements-dev.txt   ruff, pinned; never installed on the basestation
└── README.md
```

`ruff.toml` and `requirements-dev.txt` are listed in `.gitignore` and are
**untracked**. They exist in this working tree; a fresh clone has neither, so it
has no lint config and no pinned ruff until they are recreated. Everything else
in the tree above is tracked.

Robot-side code lives in the **`BlueBoat-Control` submodule**, not here — see §6.

### Threading model
`rclpy` spins on its own thread. `core/signals.py::SignalBus` is the **only**
crossing point: ROS callbacks emit Qt queued signals, `DataStore` slots mutate
state on the GUI thread, and widgets repaint from a single ~10 Hz tick in
`MainWindow` (`MapConfig.ui_refresh_hz`). Never touch Qt objects from a ROS
callback, and never block the ROS thread on GUI work.

`DataStore` is the single source of truth for the GUI. Widgets read it; they do
not subscribe to ROS directly.

### Degraded mode
If `rclpy` is not importable the app still starts, GUI-only (this is what makes
`smoke_test.py` possible without ROS). Keep new ROS imports lazy or guarded so
this keeps working.

---

## 2. ROS2 interface — the exact contract

Everything here is configured in `mcs/config/settings.py::TopicsConfig`; change
names there, never inline. Peer module for all of these is `blueboat_control`
(nodes `robot_interface.py`, `master_control.py`, `path_generation.py`,
`param_set`), plus MAVROS.

### Subscriptions

| Topic | Type | Produced by | Used for | Notes |
|---|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface.py` | pose, yaw, speed, trail | **Re-zeroed at that node's first callback** — position *and* yaw. World origin = boat position at launch; world **+x = boat heading at launch**; yaw starts at 0. Not the MAVROS power-on frame. |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | MAVROS | georeference, GPS read-out | BEST_EFFORT QoS. `lat==0 and lon==0` means no fix and is discarded. |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | MAVROS | **robot glyph heading** | Degrees **clockwise from north** (0=N, 90=E). Converted to the app-wide convention (radians CCW from east) as `radians(90 - hdg)`. Absolute and available immediately. Subscribed **BEST_EFFORT** — this and the GPS fix are the only two; every other subscription uses the default reliable depth-10 profile. |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | armed / connected / flight mode | Only subscribed when `mavros_msgs` imports; absent in simulation. |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | `robot_interface.py` | USBL pinger marker | **ROBOT/BODY frame.** With `fixed_pinger=False` (the default) it is seeded from the Waterlinked *filtered* (`filaco`) position and dead-reckoned at odom rate between USBL updates. |
| `/uw_gps_data` | `std_msgs/Float32MultiArray` | `uwgps_log.py` | raw USBL freshness | 19 values: date(7), aco xyz, ant xyz, lat/lon/dep, filaco xyz. |
| `/monitoring_data` | `std_msgs/Float32MultiArray` | `master_control.py` | target display, distance plot | `[t, x, y, psi, x_d, y_d, psi_d, u1, u2]`, published at the controller's 20 Hz loop rate. `x_d/y_d/psi_d` are **WORLD frame for every controller branch** (§6, non-negotiable N3). |
| `/thruster_input` | `std_msgs/Float32MultiArray` | `master_control.py` | motor read-out | Order is **`[right, left]`** in Newtons. |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface.py` | readiness gating | Default QoS (depth 10, volatile). **Re-published every 1 s** rather than once, because a one-shot handshake races DDS discovery — that periodic republish, not a latched QoS, is what makes a late subscriber receive it. |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set` | safe-shutdown acknowledgement | `'default'` / `'override'`. This echo is what proves an E-STOP command landed. |

### Publications

| Topic | Type | Consumed by | Contract |
|---|---|---|---|
| `/blueboat/input_str` | `std_msgs/String` | `robot_interface.py` (dispatch), `param_set` | Values: `enable`, `default`, `override`, `stop`, `arm`, `disarm`, `move <l> <r> <s>`. Any unrecognised token falls through to `move_callback`. The station publishes only `default` / `override`. |
| `/blueboat/manual_target` | `std_msgs/Float32MultiArray` | `master_control.py` | `[x, y]` in the **WORLD frame**. `[0.0, 0.0]` is the *resume-original-mission sentinel*, not a coordinate — a genuine click at the origin is nudged by `1e-3`. Only the explicit "Continue Original Mission" action may publish `[0,0]`. |

### Services

| Service | Type | Role |
|---|---|---|
| `/path_request` | `blueboat_interfaces/srv/RequestPath` | **Client.** Request is `std_msgs/Float32MultiArray path_request` (an array of path-parameter values); response is `nav_msgs/Path path`. This is how the mission path is fetched for display. |

### Frame conventions (the recurring source of bugs)
- Manual target: **world** on the wire. Pinger target: **body** on the wire.
  `master_control` converts the manual target via `inRobotFrame()` before
  `solve_LoS()` and passes the pinger vector straight through, so **both are
  robot-frame at the solver** — that part is consistent by design.
- Yaw everywhere in the app: radians, CCW-positive about +z, y-up world.
- Odometry yaw is launch-relative, so it is *not* an absolute heading. Absolute
  heading comes from the compass (see `DataStore.robot_true_heading()`, which
  prefers the compass and falls back to `yaw + georef theta`).

---

## 3. NON-NEGOTIABLE constraints

Violating any of these breaks another module, hardware safety, or field data.

**N1 — Safe shutdown before terminating a launch.** E-STOP, Stop Mission and
app exit must all run the `command_center` sequence: publish `default` on
`/blueboat/input_str` → verify a matched subscriber via
`get_subscription_count()` → wait for the `/blueboat/param_mode` echo (one
republish at T/2) → flush delay → *only then* terminate the process. Killing the
launch first can leave the motors in override. In a simulation graph
(`CommandCenter.set_simulation_mode(True)`) the *acknowledgement wait* is
skipped — `Sim_launch.py` starts neither `robot_interface` nor `param_set`, so
no echo can arrive — but the publish still happens before the flush and the
terminate, so the ordering guarantee is unchanged.

**N2 — `[0,0]` on `/blueboat/manual_target` is a control-handover sentinel.**
Never publish it as a position.

**N3 — Never re-apply a frame correction to `/monitoring_data` in the app.**
`x_d/y_d` are world-frame at the source: `master_control` captures the world
target before its `inRobotFrame()` conversion, at every branch (the
`# --- world-frame monitoring target ---` markers). An app-side robot→world
fixup would double-convert.

**N4 — The pinger world position is computed only inside the pinger message
callback**, using the pose concurrent with that message, and stays fixed
between messages. Re-anchoring a stale body vector to each new odom pose makes
the marker drag along behind the robot.

**N5 — The map view never rotates.** North-up is achieved by placing items in
the ENU frame, not by rotating the view; rotating the view rotates the satellite
tiles with it. Only the robot glyph rotates.

**N6 — The robot glyph's heading must come from an absolute source.** Raw
`/blueboat/odom` yaw is launch-zeroed and makes the arrow point screen-right at
every mission start regardless of true heading.

**N7 — Mission-path preview uses the `/path_request` service directly, never
`path_publisher`.** `path_publisher.py` is only started by `Sim_launch.py`, so
depending on it breaks every real-robot run.

**N8 — A GPS-anchored mission is never deployed with a non-heading-aligned
georeference.** `io_yaml.deploy_mission()` raises in that case; the placeholder
`theta = 0` would rotate the whole mission. Deferred deployment is real-robot
only — `_start_gps_deployment` returns immediately in simulation, because the
georeference can never converge without GPS and the boat would hold position at
(0,0) forever.

**N9 — Keep `mcs/` free of robot-side code.** Robot-side changes belong in the
`BlueBoat-Control` submodule (§6); nothing under `mcs/` imports robot-side
modules at runtime.

**N10 — Never weaken a comparison baseline or overstate evidence** in anything
this module produces for the thesis. See the project-level
`project_synthesis.md` §4/§8.6: the two-pass orthogonal baseline must be tuned
seriously, and simulation-derived results are stated as model-conditional.

---

## 4. Map & georeferencing (current model)

Two scene regimes, switched once, in `mcs/gui/map/map_view.py`:

- **Before heading alignment** — scene is the raw robot world frame (world-up).
  A `world-up (north unknown)` notice is shown.
- **After heading alignment** — scene is **local east/north (ENU)**. Every
  world-frame quantity (glyph, trails, pinger, targets, mission path, tiles) is
  converted with `GeoFit.world_to_enu()` at placement, so north is up with the
  view unrotated. An `N` + arrow badge appears.

`core/geo.py::GeoReferencer` has two fit stages:
- **Translation-only**, emitted from the *first* GPS fix — `is_valid` becomes
  true immediately so satellite tiles work right away; `theta` is a placeholder
  `0` and `heading_aligned` is `False`.
- **Rotated Kabsch fit**, once enough motion (`min_spread_m`) makes rotation
  observable — `heading_aligned` becomes `True` and `theta` is trustworthy.

Consumers must check `heading_aligned`, not just `is_valid`, before trusting
rotation.

`MapView._to_world()` is the exact inverse of `_to_scene()`, under the same
regime test, and every mouse position read back out of the view goes through
it: the published manual target, the click inspector (read-out, GPS and
distance to robot), the measure endpoints. `center_on_robot()` is the mirror
case and converts world→scene before `centerOn`. Item *placement* stays in
scene coordinates throughout, and the measure distance is computed there
because world↔scene is rigid. Anything holding a bare scene coordinate across
the one-time switch to ENU — an in-progress measurement, the inspector dot —
is dropped by `_update_scene_mode()`; the manual-target crosshair is re-placed
from the stored world target on every refresh tick instead.

**The mission path is not drawn until the fit is heading-aligned.**
`MapView.refresh()` gates the mission-path item on `store.world_frame_ready()`,
and `MainWindow._on_tick` holds the pending `/path_request` preview until the
same flag turns true. An empty map early in a run is expected, not a fault.

---

## 5. Survey Pattern Designer & trajectory files

`mcs/designer/` is layered so the logic is testable without Qt widgets:
`model.py`, `interpolation.py`, `patterns.py`, `sampling.py`, `io_yaml.py` are
widget-free; `designer_map.py`, `panels.py`, `designer_window.py` are UI. The
window is titled "Survey Pattern Designer".

Extension points are registries — add an `Interpolation` subclass or a `Pattern`
subclass and register it in `REGISTRY`; the parameter forms are generated from
each class's `schema`. The runtime never learns about new types, because
interpolation is resolved at export time.

Undo is snapshot-based: **any new mutating entry point must call
`DesignerWindow._push_undo()` first.**

### Files produced (in `~/.config/blueboat_mcs/trajectories/`)

| File | Role |
|---|---|
| `<name>.yaml` | **Runtime.** Format tag `blueboat_trajectory/1`. Dense `[t, x, y, yaw]` samples + `speed`, `loop`, `length_m`, `duration_s`. Consumed by the robot. |
| `<name>.meta.yaml` | **Editor only.** Groups, locks, per-segment interpolation + speed, comments. The robot never reads it. |
| `.deployed/<name>.yaml` | **Generated per run**, for GPS-anchored missions. Regenerated every launch. |

Why dense samples: `path_generation.single_pose(t)` evaluates at arbitrary
times, so the runtime only ever does a binary search plus linear interpolation
(yaw wrap-aware). With `loop: false` evaluation past the end **clamps to the
final pose**, so a mission ends in station-keeping; with `loop: true` time wraps
modulo `duration`.

Optional `geo_anchor: {lat0, lon0, theta_deg}` georeferences the design frame.
Anchored missions are relocated into the current run's world frame at deploy
time; non-GPS missions are normally *start-aligned* (first sample `(0,0)`,
first tangent `+x`) so the boat starts at the mission and moves forward —
offered on save by `_maybe_offer_alignment`, and available any time from
Edit ▸ Align to Start. The launch dialog badges a **non-GPS** custom path whose
runtime file does not start that way as `(not start-aligned)`; GPS-anchored
missions are exempt (they are geographically fixed and the boat turns toward
them). It is informational — the launch path never rewrites a mission file.
`sampling.start_misalignment` is the single predicate behind both the designer
action and the badge, with its tolerance in `DesignerConfig`
(`start_align_tol_m` / `start_align_tol_deg`), so the two cannot disagree about
the same file. `mcs/designer/{sampling,io_yaml,interpolation,patterns}.py`
import no Qt at runtime; only `model.py` and the UI modules do.

**Do not hand-edit files under `.deployed/`** — they are regenerated and carry
`deployed_from` / `deployed_fit_rms_m` provenance.

Other persistent state: `~/.config/blueboat_mcs/config.json` (user overrides,
hand-editable) and `~/.config/blueboat_mcs/tile_cache/` (disposable).

---

## 6. Robot-side files — in the `BlueBoat-Control` submodule

The four nodes the station depends on behaving a particular way are **committed
to `blueboat_control`** and installed by its `CMakeLists.txt`. There is no copy
step from this repo: change them in the `BlueBoat-Control` submodule, commit
there, and `colcon build`.

| File (path under `BlueBoat-Control/blueboat_control/`) | What the station depends on |
|---|---|
| `src/_custom_libraries/path_generation.py` | The `from_yaml` branch + a file **watcher** that reloads when the file appears/changes, holding a station-keeping pose until it does (this is what makes deferred GPS deployment possible). `generate_path()` and every hardcoded trajectory are unchanged. |
| `src/_custom_libraries/yaml_trajectory.py` | Loads `blueboat_trajectory/1` and evaluates at time `t`. Depends only on PyYAML + numpy. |
| `src/master_control.py` | Captures the **world-frame** target before the `inRobotFrame()` conversion so `/monitoring_data` is uniform across branches (`# --- world-frame monitoring target ---`, five capture sites: manual, the MPC / PID / LoS path branches, and the pinger branch). 20 Hz loop — `self.dt = dbl('control_dt', 0.05)` (`master_control.py:263`), a **declared ROS parameter** in the working tree, so the rate the station's `DiagnosticsConfig.expected_hz` assumes is settable per launch there (see the ⚠ below). `/controller_target` unchanged. |
| `src/robot_interaction/robot_interface.py` | CSV logging: important columns first (names unchanged), rows filled **by column name**, `[right, left]` thruster order, and the no-pinger target logged from `/monitoring_data`. Also the producer of `/blueboat/odom`, `/blueboat/pinger_coordinates` and `/blueboat/controller_ready`. |
| `src/_custom_libraries/robot_log_schema.py` | The position-CSV column layouts themselves (`COLUMNS_PINGER` / `COLUMNS_NO_PINGER`, selected by `columns_for(use_UWgps)`). ROS-free data module — read this, not the node, to learn the CSV format offline analysis consumes. |

Trajectory selection needs **no launch-file change** — the path rides inside the
existing argument: `trajectory:=from_yaml:/abs/path.yaml`.

### Robot-side behaviours the station does not compensate for
Point-LoS arrival checking is **off by default** — `safety_distance = -1.0`, a
declared ROS parameter on `master_control` (`:314`). If it is enabled,
`stopping_sequence` latches on arrival (set `:478`, guarded `:472`, initialised
`False` at `:180`) and is never reset, so every later manual or pinger target
yields zero thrust with nothing on screen saying why. Pinger-marker lag is
inherent to the source — the filtered seed plus dead reckoning described in §2
(`robot_interface.py:529-536`); N4 is what keeps it from *also* dragging behind
the robot. Neither is corrected in `mcs/` — see
`.claude/specs/robot-side-limitations-watchlist.SPEC.md`.

**⚠ Line numbers into `BlueBoat-Control` are volatile, and which version the
boat runs cannot be settled from this repository.** Every citation above is
against `BlueBoat-Control`'s **working tree**, which carries uncommitted
modifications to `master_control.py`, `robot_interface.py` and
`path_generation.py`. At the commit the superproject records for that submodule
the same code reads differently — `self.dt` is a hardcoded `0.05` at `:106` and
`safety_distance` a hardcoded `-1.` at `:208`, neither a declared parameter — so
`control_dt` and `safety_distance` are launch-settable only on the working-tree
version. Anchor on the symbol name, not the line, and treat the deployment
question as `TODO.md` A3 (needs the boat's workspace).

### CSV logs (written by `robot_interface.py` on the robot)
Two layouts. With pinger: date, `relative_*`, `corrected_pinger_*`, GPS, pinger
GPS, thrusters, then raw USBL/IMU. Without pinger: date, `relative_*`,
`target_*`, GPS, thrusters, then raw IMU. `target_*` exists **only** in the
no-pinger layout — in pinger mode it duplicated `corrected_pinger_*`.

---

## 7. Commands known to work

```bash
# run (basestation workspace is ~/ros2_ws; env.sh activates .venv + sources
# ROS2 and the overlay). /blueboat_ws is the BOAT's workspace, not this one.
cd ~/ros2_ws && source env.sh
cd src/BlueBoat-SideScanSonar/BlueBoat-MCS
pip install -r requirements.txt   # first time only; NOT --user inside the venv
python3 run.py
python3 run.py --config /path/to/config.json --verbose

# build.sh does the whole sequence (colcon build in ~/ros2_ws, then run.py)

# regression script — headless, no ROS needed
QT_QPA_PLATFORM=offscreen python3 smoke_test.py

# lint — no ROS and no venv needed; ruff resolves no imports
pip install -r requirements-dev.txt   # first time only
ruff check .

# launching a mission from the CLI (what the launch dialog builds)
ros2 launch blueboat_control BlueBoat_launch.py \
    enable_motors:=True controller_type:=LoS use_pinger:=True
ros2 launch blueboat_control BlueBoat_launch.py \
    controller_type:=LoS trajectory:=from_yaml:/abs/path/mission.yaml
ros2 launch blueboat_control Sim_launch.py \
    robot_file:=thrusters_ur controller_type:=MPC trajectory:=circle

# field debugging
ros2 topic echo /mavros/global_position/compass_hdg
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: default"
```

The boat's own workspace is `/blueboat_ws` (with a `.venv`), which is where
`BlueBoat-Control` is built and run — distinct from the basestation's
`~/ros2_ws`.

`ruff check .` is the lint gate, configured in `ruff.toml`. It exits clean on
this tree in any environment — sourced, venv-only, or bare system Python — and
is pinned to an exact version (`ruff==0.16.5`) in `requirements-dev.txt`,
because the gate is ruff's *default* rule selection rather than an explicit
`select` list. Both files are gitignored (§1), so the gate travels with this
working tree, not with a clone. Two
suppressions are configured rather than fixed: `RUF046` project-wide (the
`int(math.floor(...))` casts in the tile and pattern geometry are correct as
written) and `RUF012` for `mcs/designer/{interpolation,patterns}.py` plus
`smoke_test.py` (the `schema` class attribute is the §5 extension point). The
eight broad `except Exception` clauses in the ROS-facing and logging paths carry
inline `# noqa: BLE001` with a reason each and stay broad.

**No type-checker is configured, deliberately.** A mypy survey of `mcs/` found
45 errors and *no* real defect: 8 were untyped-import declarations for the ROS
message packages, and the remaining 37 were PySide6 stub-narrowing noise
concentrated in `SchemaForm._make_editor` and the `schema` attribute — the two
documented extension points. Making it clean would mean annotating around the
Qt stubs across the GUI, which is restructuring for no defect caught. Re-open
this only with new evidence, not from scratch.

`smoke_test.py` is a linear script — not a test framework — that imports every
module, builds the full window offscreen and drives synthetic telemetry through
the `SignalBus`.
It prints fourteen checkpoints (`TimeSeries ok`, `GeoReferencer ok`,
`LoS predictor ok`, `start alignment ok`, `designer ok`, `deploy guard ok`,
`store ok`, `stats ok`, `pinger anchor ok`, `map frame ok`, `sentinel ok`,
`safe shutdown ok`, `window ok`, `SMOKE TEST PASSED`) and aborts on the first
failed assertion. It passes on this tree, exit status 0, and runs identically
across all three environment shapes: no `rclpy` (GUI-only); `rclpy` importable
but the overlay unsourced, so `blueboat_interfaces` is missing and the bridge
node comes up with its `/path_request` client disabled; and a fully sourced
workspace. Whenever a real bridge node exists, the blocks that assert what was
published substitute their own node stub.

Four of the §3 non-negotiables are asserted there: `pinger anchor ok` (N4)
holds the pinger world position fixed across odom updates, `sentinel ok` (N2)
proves an origin click is nudged to `1e-3` and that only
`_on_continue_mission()` emits `[0, 0]`, `deploy guard ok` (N8) proves
`io_yaml.deploy_mission` raises — and writes nothing — against a
non-heading-aligned fit, and `safe shutdown ok` (N1) asserts the
`command_center` *ordering*: publish before any terminate, on the echo path,
the timeout path with its single T/2 republish, all three operator doors and
the no-ROS degraded path. `map frame ok` drives a real `QMouseEvent` through
`MapView` under a synthetic rotated `GeoFit` and asserts the scene↔world round
trip at every input site. `designer ok` covers the Qt-free designer layer —
sampling invariants, the save/load/resample round trip, every stock pattern and
interpolation from its own `schema` defaults, and both extension registries.

Still uncovered: the designer UI (`designer_map.py`, `panels.py`,
`designer_window.py`, including the `_push_undo` contract). And because the
script runs offscreen on synthetic telemetry, a green run is not field
readiness — it cannot catch QoS mismatches, real message-type drift or
launch-file argument errors.

Simulation runs (`Sim_launch.py`) have no MAVROS, no `robot_interface`, no
`param_set` and no pinger; code paths gated on those must degrade quietly.

---

## 8. Documentation set

`README.md` and `docs/01`–`08` + `HEADING_AND_MAP_ALIGNMENT.md` were audited
against the tree and are current: the two scene regimes of §4, the compass
heading source of §2, the 20 Hz `master_control` loop, and the robot-side file
locations of §6. `docs/03_ros_integration.md` §Observations is the running list
of robot-side findings, each marked open or fixed against
`BlueBoat-Control`; §6 above owns the two limitations carried deliberately.
`docs/HEADING_AND_MAP_ALIGNMENT.md` is written to stand on its own for reuse in
the other application.

Line citations into `BlueBoat-Control` have been re-anchored against its
current working tree and verified to land on the code they describe. They drift
whenever that module is edited; `BlueBoat-Control` owns them, and its own
`CLAUDE.md` §2.1.1 records the section layout they point into. Prefer the symbol
name over the line number when following one.

This file stays authoritative where it and `docs/` disagree — `docs/` is the
long-form explanation, not a second source of truth.
