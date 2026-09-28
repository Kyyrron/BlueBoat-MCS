# BlueBoat-MCS — Mission Control Station

Operator ground-station GUI for a BlueRobotics BlueBoat USV used in a master's
thesis on aspect-aware side-scan sonar survey. Runs on the operator laptop
(`~/ros2_ws`), like every node of the project; the boat carries only its hardware
and its own firmware, reached through the BlueBoat Base Station WiFi.

**The station supervises and commands. It performs no control computation and
duplicates no logic that already exists on a ROS topic.** Every number it shows
comes from a topic or service published by the existing `blueboat_control`
stack. When something looks wrong on screen, the first question is whether the
producing node is right, not whether the GUI should compensate.

Build system: standalone Python (no colcon, no `setup.py`). The package is run
from the repo root; ROS2 comes from the sourced workspace, never from pip.

---

## 1. Layout & internals

```
<repo root>/
├── mcs/                   the application (importable package)
│   ├── main.py            bootstrap; `run.py` is a thin launcher
│   ├── config/settings.py ALL topics, thresholds, gains, paths (dataclasses)
│   ├── core/              signals.py (thread boundary), series.py
│   │                      (append-only growable series, never wrapped —
│   │                      recording is unbounded), geo.py (odom↔GPS),
│   │                      los_predictor.py, sim_gps.py, sea.py, worlds.py
│   │                      (Gazebo world folders ~/worlds/<path>/<world>/:
│   │                      list, filter by limits, duplicate — reimplements
│   │                      the SSS-Sim contract locally, never imports it,
│   │                      per CM-3)
│   ├── models/store.py    DataStore: the single in-memory state snapshot
│   ├── ros/               ros_manager (rclpy thread), bridge_node (all
│   │                      subs/pubs/service), launch_manager, command_center
│   ├── gui/               main_window, left/right/bottom panels, console,
│   │                      mission_stats, sea_state_box, theme, widgets,
│   │                      map/, plot/, dialogs/
│   └── designer/          Survey Pattern Designer (widget-free logic + Qt UI)
├── docs/                  03, 04, 05 + GPS_MAP_ARCHITECTURE.md (see §8)
├── smoke_test.py          headless regression script (see §7)
├── build.sh               colcon build in ~/ros2_ws, then `python3 run.py`
├── ruff.toml              lint config (see §7)
├── requirements.txt       runtime deps only
├── requirements-dev.txt   ruff, pinned; dev-only, kept out of requirements.txt
└── README.md
```

`ruff.toml` and `requirements-dev.txt` are listed in `.gitignore` and are
**untracked**. They exist in this working tree; a fresh clone has neither, so it
has no lint config and no pinned ruff until they are recreated. Everything else
in the tree above is tracked.

Control-stack code (`robot_interface`, `master_control`, `path_generation`, …) lives in the
**`BlueBoat-Control` submodule**, not here — see §6.

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

### Layer diagram

```
┌────────────────────────────  GUI thread  ────────────────────────────┐
│  gui/            main_window · left_panel · right_panel · toolbar    │
│                  map (view, items, tiles) · plot · dialogs · designer│
│        reads on 10 Hz tick                 emits user intents        │
│  models/store    DataStore: live states + full-experiment histories  │
│  core/           SignalBus · TimeSeries · GeoReferencer · predictor  │
├───────────────────────  Qt queued signals  ──────────────────────────┤
│  ros/            RosManager (thread) → BridgeNode (subs/pubs/service)│
│                  LaunchManager (ros2 launch subprocess)              │
│                  CommandCenter (stop actions, mode toggle)           │
└──────────────────────────────────────────────────────────────────────┘
```

`core/` and `models/` import no widgets and no rclpy (only `QObject` for
signals) — that is what makes the offline smoke test possible.

### Module inventory

| Module | Responsibility |
|---|---|
| `main.py` | bootstrap: logging, `AppConfig.load`, `QApplication`, `MainWindow` |
| `config/settings.py` | every topic name, threshold, gain; JSON overrides |
| `core/signals.py` | the thread boundary |
| `core/series.py` | append-only growable series (never wrapped) |
| `core/geo.py` | odom↔GPS translation fit, equirectangular helpers |
| `core/los_predictor.py` | display-only LoS path sketch |
| `core/sim_gps.py` | synthetic NavSatFix model for GPS-anchored sim runs |
| `core/sea.py` | sea-state presets, timelines, readback parsing (sim) |
| `core/worlds.py` | `~/worlds/` enumeration, limits filter, duplication |
| `models/store.py` | states, histories, derived stats |
| `designer/` | Survey Pattern Designer (Qt-free model + Qt UI) |
| `ros/ros_manager.py` | rclpy lifecycle |
| `ros/bridge_node.py` | subs, pubs, path service, topic stats — **the only file that knows message types** |
| `ros/launch_manager.py` | `ros2 launch` subprocess + sea companion |
| `ros/command_center.py` | E-STOP / safe-shutdown sequences, mode toggle |
| `gui/main_window.py` | the 10 Hz tick, signal wiring, launch orchestration |
| `gui/{left,right}_panel.py`, `bottom_toolbar.py`, `console.py` | panels and toolbar |
| `gui/map/` | `map_view` (scene + modes), `map_items` (drawing), `tile_layer` |
| `gui/sea_state_box.py`, `gui/mission_stats.py` | the two floating map overlays |
| `gui/dialogs/` | `launch_dialog`, `world_dialog`, `sea_state_dialog` |
| `gui/plot/distance_plot.py` | robot↔target distance plot |

### Data flow, end to end

Odometry, the canonical example:

1. `BridgeNode._on_odom` (ROS thread) converts the message to plain Python
   lists, stamps `time.monotonic()`, updates its `TopicStats` via `_mark`, and
   emits `SignalBus.odom_received`.
2. Qt queues the signal into the GUI thread; `DataStore.on_odom` updates
   `RobotState`, appends to `robot_track` / `speed_hist`, integrates travelled
   distance, feeds the `GeoReferencer`, refreshes the pinger world position.
3. Nothing repaints yet. At the next 10 Hz tick `MainWindow._on_tick` calls
   `left_panel.refresh()`, `right_panel.refresh()`, `map_view.refresh()`, which
   *pull* from the store and repaint once.

User intents flow the other way: widget → signal → `MainWindow` slot →
`CommandCenter` → `BridgeNode.publish_*` (thread-safe) → ROS graph.

### Clocks

All history timestamps are `time.monotonic()` at **reception**, one clock for
everything. Experiment-relative time (what the timeline shows) is `t − store.t0`,
`t0` being the first odometry sample. The controller's own `t` inside
`/monitoring_data` is **never** used as a clock — reception time is, which keeps
every series mutually consistent even if a node restarts.

### Repaint bounds

`TimeSeries.decimated_window` bounds every repaint: at most
`map.trajectory_max_points_drawn` (20 000) vertices per polyline and 2000 per
plot. `PolylineItem.set_points` rebuilds a `QPainterPath` per tick from the
decimated window — measured adequate at 10 Hz well past 10⁵ stored points. The
tile layer refuses to populate more than 64 tiles per viewport instead of
hammering the tile server.

### Map rendering

Constant-pixel glyphs (`ItemIgnoresTransformations`) for the boat and markers,
cosmetic pens for lines, an adaptive 1/2/5-decade metric grid painted in
`drawBackground`, and the satellite `TileLayer` at z = −100. Interaction modes
(`NORMAL` / `MANUAL_TARGET` / `MEASURE`) are a small state machine inside
`MapView`; the view emits intents (`target_clicked`, `point_inspected`) and
never publishes anything itself.

### How to add…

**A subscribed topic.** Name it in `TopicsConfig`; add thresholds to
`DiagnosticsConfig` (it then appears in the diagnostics panel automatically);
add a `Signal` to `SignalBus`; create the subscription + `_on_x` callback in
`BridgeNode` (call `self._mark(topic)` first); add a `DataStore.on_x` slot and
connect it in `MainWindow._connect_signals`. Display it from any `refresh()`.

**A published command.** Add a publisher in `BridgeNode.__init__` and a
`publish_x` wrapper (keep the `self._pub_lock` pattern and the `command_sent`
emission), expose a semantic method on `CommandCenter`, call it from the GUI.

**A map layer.** Create an item in `gui/map/map_items.py` (`_cosmetic_pen` for
constant-pixel lines, `ItemIgnoresTransformations` for constant-pixel glyphs),
add it to the scene in `MapView.__init__`, update it in `MapView.refresh()`,
register a key in `MapView.set_layer_visible`, add one entry to `_LAYERS` in
`left_panel.py`. That is the whole checklist.

**A plot.** Follow `DistancePlot`: a `QWidget` reading one `TimeSeries`, honouring
`set_time_window`, painting in `paintEvent`. Add it to a `CollapsibleSection` in
the right panel and call its `refresh()` from `RightPanel.refresh()`.

**A launch argument.** Four sites, all in `mcs/ros/launch_manager.py` plus the
dialog: `LaunchParameters.to_cli()` (the main argument list, which splices in
`params.sea_args()`), `companion_command()` (the empty-world sea companion),
`launch_target()` (which of the three launch files runs), and
`gui/dialogs/launch_dialog.py` for the widget.

---

## 2. ROS2 interface — the exact contract

Everything here is configured in `mcs/config/settings.py::TopicsConfig`; change
names there, never inline. Peer module for all of these is `blueboat_control`
(nodes `robot_interface.py`, `master_control.py`, `path_generation.py`,
`param_set`), plus MAVROS.

### Subscriptions

| Topic | Type | Produced by | Used for | Notes |
|---|---|---|---|---|
| `/blueboat/odom` | `nav_msgs/Odometry` | `robot_interface.py` | pose, yaw, speed, trail | **Local ENU**: position translated at that node's first callback (world origin = boat position at launch), axes East/North, yaw **absolute ENU** (0 = East, CCW+; NOT re-zeroed). Same frame kind as the simulator's Gazebo-bridged odom — only the origin differs. The pre-2026-08-31 hybrid (translated position, launch-relative yaw) is gone; see `docs/03_ros_integration.md` §Observations. |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | MAVROS (real) · **the station's own bridge node** (Gazebo run of a GPS-anchored mission: a 5 Hz timer synthesises fixes from the sim odom — world metres → lat/lon about a receiver origin placed `SimGpsConfig.offset_north_m` north of the mission's first point — received back through its own subscription, so the whole pipeline incl. diagnostics runs as on real water) | georeference, GPS read-out | BEST_EFFORT QoS. `lat==0 and lon==0` means no fix and is discarded. |
| `/mavros/global_position/compass_hdg` | `std_msgs/Float64` | MAVROS | **robot glyph heading (preferred)** | Degrees **clockwise from north** (0=N, 90=E). Converted to the app-wide convention (radians CCW from east) as `radians(90 - hdg)`. Fallback: the odom yaw directly (absolute ENU). Subscribed **BEST_EFFORT** — this, the GPS fix and the battery are the three sensor streams that are; every other subscription uses the default reliable depth-10 profile. |
| `/mavros/state` | `mavros_msgs/State` | MAVROS | FCU **connection** flag, one of the three inputs to the readiness count | Only subscribed when `mavros_msgs` imports; absent in simulation. `armed` and `mode` ride the same message but nothing displays them, so the store does not keep them. |
| `/mavros/battery` | `sensor_msgs/BatteryState` | MAVROS (`sys_status`) | battery charge read-out (left panel, ROBOT) | BEST_EFFORT QoS. `percentage` is a **0..1 fraction**, not a percent — MAVROS divides the MAVLink value by 100. A field the FCU does not report arrives NaN, is passed on as `None` and never overwrites the last good reading; the row greys out after 15 s of silence. No simulation graph publishes this topic. |
| `/blueboat/pinger_coordinates` | `std_msgs/Float32MultiArray` | `robot_interface.py` | USBL pinger marker | **ROBOT/BODY frame.** With `fixed_pinger=False` (the default) it is seeded from the Waterlinked *filtered* (`filaco`) position and dead-reckoned at odom rate between USBL updates. |
| `/uw_gps_data` | `std_msgs/Float32MultiArray` | `uwgps_log.py` | raw USBL freshness | 19 values: date(7), aco xyz, ant xyz, lat/lon/dep, filaco xyz. |
| `/monitoring_data` | `std_msgs/Float32MultiArray` | `master_control.py` | target display, distance plot | `[t, x, y, psi, x_d, y_d, psi_d, u1, u2]`, published at the controller's 20 Hz loop rate. `x_d/y_d/psi_d` are **WORLD frame for every controller branch** (§6, non-negotiable N3). |
| `/thruster_input` | `std_msgs/Float32MultiArray` | `master_control.py` | motor read-out | Order is **`[right, left]`** in Newtons. |
| `/blueboat/controller_ready` | `std_msgs/Bool` | `robot_interface.py` | readiness gating; **E-STOP acknowledgement** | Default QoS (depth 10, volatile). **Re-published every 1 s** rather than once, because a one-shot handshake races DDS discovery — that periodic republish, not a latched QoS, is what makes a late subscriber receive it. Since 2026-09-04 it also carries `False`, published the instant `full_stop()` latches, which is what confirms an E-STOP landed (N1b). |
| `/blueboat/param_mode` | `std_msgs/String` | `param_set` | safe-shutdown acknowledgement | `'default'` / `'override'`, or `''` for "alive but no mode locked". **Heartbeated at 1 Hz** since 2026-09-04, so a late station sees the mode without waiting for a transition — which is exactly why the shutdown sequence requires a transition rather than any matching value (N1). |

**Sea state (sim only, 2026-09-03).** `/sim/sea_state` (`std_msgs/String`
JSON, subscribed **TRANSIENT_LOCAL depth 1** — the one latched reader in
the station) feeds `DataStore.sea` (`mcs.core.sea.SeaReadout`) and the
floating SEA STATE box (`gui/sea_state_box.py`, parented to the
map view at its top-left — the mirror of the mission-stats box), visible
only while a Gazebo mission runs;
`/sim/sea_state/command` (String JSON) is published by "Modify
situation…" through `CommandCenter.send_sea_state`. Presets come from the
simulator's installed `share/blueboat_sss_sim/config/sea_states.yaml`,
read as a file (`mcs.core.sea.SeaCatalog`, never-raises, CM-3); the
choice is `LaunchParameters.sea` (`SeaChoice`) — `sea_*` arguments in
world mode, a companion `ros2 launch blueboat_sss_sim sea_state_launch.py
world_name:=ocean …` (`launch_manager.companion_command`, its own session,
best effort) in the empty world, nothing when the choice is null.
Timelines (`blueboat_sea_schedule/1`) live under
`~/.config/blueboat_mcs/sea_states/`; a **Custom** wave choice
(`SeaChoice.custom_waves`: Hs, Tp, γ, events) is written there as
`custom_waves.yaml` (one keyframe, explicit fields) and passed as
`sea_schedule:=`, or sent as explicit fields in a live command. The dialog
(`gui/dialogs/sea_state_dialog.py`) is shown by the bottom toolbar after
the world dialog for **every** simulation launch and again, live, from
the floating box's button.

### Publications

| Topic | Type | Consumed by | Contract |
|---|---|---|---|
| `/blueboat/input_str` | `std_msgs/String` | `robot_interface.py` (dispatch), `param_set` | Values: `enable`, `disable`, `default`, `override`, `stop`, `arm`, `disarm`, `move <l> <r> <s>`. Any unrecognised token falls through to `move_callback`; an empty message is ignored. The station publishes `default`, `override` and — from either E-STOP button — `stop`. `enable` is the only thing that clears `robot_interface`'s E-STOP latch. |
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
- Odometry yaw is **absolute ENU** (0 = East). `DataStore.robot_true_heading()`
  prefers the compass and falls back to the odom yaw directly — there is no
  rotation correction anywhere (see `docs/GPS_MAP_ARCHITECTURE.md`).

---

## 3. NON-NEGOTIABLE constraints

Violating any of these breaks another module, hardware safety, or field data.

**N1 — Safe shutdown before terminating a launch.** **Stop Mission** and **app
exit** — the only two paths that terminate anything — must run the
`command_center` sequence: publish `default` on `/blueboat/input_str` → verify a
matched subscriber via `get_subscription_count()` → wait for the
`/blueboat/param_mode` echo (one republish at T/2) → flush delay → *only then*
terminate the process. Killing the launch first can leave the motors in
override. In a simulation graph (`CommandCenter.set_simulation_mode(True)`) the
*acknowledgement wait* is skipped — `Sim_launch.py` starts neither
`robot_interface` nor `param_set`, so no echo can arrive — but the publish still
happens before the flush and the terminate, so the ordering guarantee is
unchanged.

Since `param_set` heartbeats `/blueboat/param_mode` at 1 Hz, the echo that
confirms *this* command must be a **transition** into `default`: the sequence
snapshots the last known mode before publishing and ignores repeats. A boat
already in `default` is reported as its own confirmation level
(`already-default`) rather than being credited with an echo it never sent.

**N1b — E-STOP cuts the motors and does nothing else.** The three concerns are
separate operator actions and must never be re-nested (they were until
2026-09-04, which is why an operator could not stop the boat without also
surrendering the servo mapping, nor surrender the mapping without ending the
mission):

| Button | Publishes | Acknowledged by | Terminates? |
|---|---|---|---|
| **E-STOP** | `stop` | `/blueboat/controller_ready` → `False` | no |
| **E-STOP + Stop Override** | `stop`, then `default` | the above, then the `param_mode` transition | no |
| **Stop Mission** / app exit | `default` | the `param_mode` transition | yes, after confirmation |

`stop` is the real emergency primitive: `robot_interface.full_stop()` zeroes the
thrust, closes the motor gate, disarms and **latches** until an explicit
`enable`, all without leaving override. Leaving override is a *transfer* of
authority — the RC channels go back to whatever else is transmitting — so it is
never part of the panic button. The E-STOP acknowledgement runs in its own state
machine so it can neither block nor be blocked by an in-flight `default`
sequence.

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

**N6 — The robot glyph's heading must come from an absolute source.** The
compass is preferred; the odom yaw is an acceptable fallback because it is
absolute ENU (0 = East, not launch-zeroed). Never re-apply a rotation
correction to either — the old `theta` machinery is gone and re-adding one
would rotate a correct heading wrong.

**N7 — Mission-path preview uses the `/path_request` service directly, never
`path_publisher`.** `path_publisher.py` is only started by `Sim_launch.py`, so
depending on it breaks every real-robot run.

**N8 — A GPS-anchored mission is never deployed without a valid translation
anchor.** `io_yaml.deploy_mission()` raises when `current_fit` is None; the
watcher polls `geo.is_valid`, which converges from the first few GPS fixes
with **no vehicle motion** (the world frame is local ENU, so only a
translation is estimated — the old heading-alignment requirement and its
station-keeping deadlock are gone). Deferred deployment runs on real water
**and** in Gazebo runs of GPS-anchored missions (`gps_simulated`: the
station synthesises the fixes itself and spawns the boat at the **fixed**
`LaunchConfig.sim_spawn_yaw_deg` via `spawn_yaw:=`) — only a sim launch of a
*non-anchored* mission
skips it and executes design-frame points directly. A GPS-anchored sim
launch may instead target a **personalized world**
(`LaunchParameters.world_dir` set → `blueboat_sss_sim
full_mission_launch.py`): the deferred deploy still runs, but the fixes
come from the simulator's mavros shim (the world's own anchor) and the
station's SimGps stays **disarmed** — arming it too would put a second
publisher on `/mavros/global_position/global` with an arbitrary origin and
deploy the path into the wrong frame relative to the world geometry. That
launch file declares no `spawn_yaw`; the boat spawns at world (0, 0)
heading east.

**N8b — A simulated run is reproducible: same spawn pose, same fix noise.**
The spawn heading is a configured constant (`LaunchConfig.sim_spawn_yaw_deg`,
0° = east; the position is always Gazebo (0, 0)) and `SimGpsModel` is
constructed with `SimGpsConfig.noise_seed`, never an unseeded generator.
Both used to be random — the heading drawn per launch, the noise per
process — and since the fix noise feeds the anchor estimate, the *deployed*
path landed a little differently every run, so two sim runs of one mission
could not be compared and a difference could not be attributed to the change
under test. Rehearsing another orientation is a *choice*: `spawn_yaw:=` in
Extra args (appended last, so it wins) or the config value.

**N9 — Keep `mcs/` free of control-stack code.** Control-stack changes belong in
the `BlueBoat-Control` submodule (§6); nothing under `mcs/` imports
`blueboat_control` modules at runtime.

**N10 — Never weaken a comparison baseline or overstate evidence** in anything
this module produces for the thesis. See the project-level
`project_synthesis.md` §4/§8.6: the two-pass orthogonal baseline must be tuned
seriously, and simulation-derived results are stated as model-conditional.

---

## 4. Map & georeferencing (current model)

**One scene regime, GPS-frame-only** (`mcs/gui/map/map_view.py`; full
rationale and the portable recipe in `docs/GPS_MAP_ARCHITECTURE.md`):

- The scene is **local east/north metres about the latched GPS origin**
  `(lat0, lon0)` — the first accepted fix. North-up, east-right, view never
  rotates; only the glyph rotates, to its true heading. There is no second
  regime and no switch.
- The only estimated quantity is a **translation** `t = EN(world origin)`:
  `EN = world + t`, `world = EN − t` (`core/geo.py::GeoFit`, defined once).
  `GeoReferencer` pairs each GPS fix with the concurrent odom pose (paired in
  `store.on_gps`, at GPS rate, with a < 0.5 s odom-freshness guard), takes
  the per-axis **median** over a rolling window, and reports a MAD-robust
  residual as `rms_m`. `is_valid` = `n_pairs ≥ min_pairs` (5) and
  `rms_m ≤ max_residual_m` — true within ~1 s of GPS, **no motion needed**.
  There is no rotation estimation: with an ENU odom frame the true rotation
  is zero by construction (the old Kabsch fit / `heading_aligned` /
  `world_yaw_to_true` machinery is deleted).
- **Nothing is drawn before `store.map_frame_ready()`** — `geo.is_valid or
  (mission.simulation and not mission.gps_simulated)`. Until then
  `refresh()` hides every item and shows a "waiting for GPS fix" notice,
  and manual-target clicks are **refused**. Two simulation modes:
  *non-GPS sim* (non-anchored mission) uses the identity anchor (fit stays
  None), draws immediately, tiles off; *GPS-sim* (`gps_simulated`, anchored
  mission) runs the full real-water pipeline — the bridge synthesises the
  NavSatFix feed (§2), the anchor gates the map, and tiles show the real
  imagery of the location the mission was planned at.
- **The georeferencer is reset on every mission launch**
  (`store.reset_georeference()` in `_on_mission_launched`): each launch
  restarts the control stack with a NEW world origin, so pairs from the
  previous run are wrong by construction. The map re-anchors from fresh
  fixes within seconds; a relaunch briefly showing "waiting for GPS fix"
  is correct, not a regression.
- Satellite tiles are placed **axis-aligned** from `(lat0, lon0)`
  (`tile_layer._place_tile`): NW corner at its east/north metres, per-tile
  metres-per-pixel scale, no rotation ever.

`MapView._to_world()` is the exact inverse of `_to_scene()` (a pure
translation both ways), and every mouse position read back out of the view
goes through it: the published manual target, the click inspector (read-out,
GPS and distance to robot), the measure endpoints. `center_on_robot()` is the
mirror case and converts world→scene before `centerOn`. Item *placement*
stays in scene coordinates throughout, and the measure distance is computed
there because world↔scene is rigid. The manual-target crosshair is re-placed
from the stored world target on every refresh tick.

`MainWindow._on_tick` holds the pending `/path_request` preview until
`store.map_frame_ready()` — immediate in simulation, a few seconds on real
water. `_on_launch_state("idle")` clears `store.mission_path` so a finished
run's path never re-anchors onto the next run's frame.

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

### The design anchor is explicit, and never live (2026-09-09)

**The designer reads no live telemetry.** Its georeference — `_manual_fit`,
returned by `_active_fit()` and set only through `_set_anchor()` — comes from
exactly four places, all of them deliberate: typed coordinates, a Gazebo world,
a one-off snapshot of the robot's current fix (**Set GPS Origin ▸ From the
Robot's Current Position**), or the `geo_anchor` of the mission being opened.
`New` and `Open` reset it, so the anchor is a property of the mission rather
than of the session, and an un-anchored mission can never inherit an origin it
was not drawn at. A permanent status-bar read-out names the anchor in force and
where it came from.

*Why this is a non-negotiable and not a preference:* `_active_fit()` used to
prefer `store.geo.fit` whenever the station held a GPS lock, deriving the design
origin from the boat's launch point and discarding whatever anchor the operator
or the file had set. Waypoint `x/y` were untouched, so satellite tiles, the
per-waypoint GPS read-outs and the world-limits rectangle all translated
rigidly by the distance between the mission's real anchor and the boat — the
"every path is shifted, restart the app and it's fine" symptom (a fresh app has
no lock, so the real anchor won). Worse, `_write()` re-derived `geo_anchor` from
it, so `Ctrl+S` on an opened mission silently repointed it at the boat on disk
and `deploy_mission` then ran the survey that far off. It also broke
`worlds.duplicate_world_for_path`'s documented "`design_offset` is [0,0] when
the anchor came from the world" invariant, and made `_maybe_offer_alignment`
treat every mission as GPS-anchored. `smoke_test.py`'s `designer anchor ok`
block pins all of it against a store with a *valid* live fit.

Consequently the designer draws **no robot and no pinger overlay** — a live
glyph placed at raw world `(x, y)` is only correct while the design frame is
the robot's world frame, which is precisely the assumption that was wrong. The
250 ms overlay timer that re-pushed the live fit is gone with them;
`DesignerMapView.resizeEvent` now carries the tile refresh that tick provided.

**Set GPS Origin ▸ From a Gazebo World** anchors the design frame on a
generated world's `metadata.yaml` (`core/worlds.py::read_world_meta`): the
world's `geo_anchor` becomes the design origin (design frame == world local
frame) and its limit rectangle is drawn as a read-only dashed overlay
(`DesignerMapView.set_world_limits`, corners kept in GPS and re-projected
through the active fit). The reference is transient window state
(`_world_ref`) — never on the model, never in undo, never in the runtime
YAML. Saving with it set copies the world folder to
`~/worlds/<mission name>/<world name>/` with `metadata.yaml`'s
`source_path` and `builder_state.yaml`'s `path.name`/`path.file` re-pointed
at the new mission (`core/worlds.py::duplicate_world_for_path`); an
existing target is never overwritten (write-once, CM-7).

**Do not hand-edit files under `.deployed/`** — they are regenerated and carry
`deployed_from` / `deployed_fit_rms_m` provenance.

Other persistent state: `~/.config/blueboat_mcs/config.json` (user overrides,
hand-editable) and `~/.config/blueboat_mcs/tile_cache/` (disposable).

---

## 6. Control-stack files — in the `BlueBoat-Control` submodule

The four nodes the station depends on behaving a particular way are **committed
to `blueboat_control`** and installed by its `CMakeLists.txt`. There is no copy
step from this repo: change them in the `BlueBoat-Control` submodule, commit
there, and `colcon build`.

| File (path under `BlueBoat-Control/blueboat_control/`) | What the station depends on |
|---|---|
| `src/_custom_libraries/path_generation.py` | The `from_yaml` branch + a file **watcher** that reloads when the file appears/changes, holding a station-keeping pose until it does (this is what makes deferred GPS deployment possible). `generate_path()` and every hardcoded trajectory are unchanged. |
| `src/_custom_libraries/yaml_trajectory.py` | Loads `blueboat_trajectory/1` and evaluates at time `t`. Depends only on PyYAML + numpy. |
| `src/master_control.py` | Captures the **world-frame** target before the `inRobotFrame()` conversion so `/monitoring_data` is uniform across branches (`# --- world-frame monitoring target ---`, five capture sites: manual, the MPC / PID / LoS path branches, and the pinger branch). 20 Hz loop — `self.dt = dbl('control_dt', 0.05)`, a committed **declared ROS parameter**, so the rate the station's `DiagnosticsConfig.expected_hz` assumes is settable per launch (see the ⚠ below). `/controller_target` unchanged. |
| `src/robot_interaction/robot_interface.py` | CSV logging: important columns first (names unchanged), rows filled **by column name**, `[right, left]` thruster order, and the no-pinger target logged from `/monitoring_data`. Also the producer of `/blueboat/odom`, `/blueboat/pinger_coordinates` and `/blueboat/controller_ready`. |
| `src/_custom_libraries/robot_log_schema.py` | The position-CSV column layouts themselves (`COLUMNS_PINGER` / `COLUMNS_NO_PINGER`, selected by `columns_for(use_UWgps)`). ROS-free data module — read this, not the node, to learn the CSV format offline analysis consumes. |

Trajectory selection needs **no launch-file change** — the path rides inside the
existing argument: `trajectory:=from_yaml:/abs/path.yaml`.

### Control-stack behaviours the station does not compensate for
**Point-LoS arrival checking is the pinger branch's alone.** `safety_distance`
is a declared ROS parameter on `master_control`, defaulting to `-1.0` on the
real boat (arrival check off) and `+1.0` in simulation. When it is positive,
`stopping_sequence` latches on arrival and the boat brakes to zero thrust with
nothing on screen saying why. A **manual** target no longer goes through either:
it has its own arrival state (`manual_keep_location`, hold / reacquire radii),
`stopping_sequence` is not consulted for it, and `manual_target_callback` clears
the latch on every new target — so the old "one reached target mutes every later
one" failure mode is gone from that path.

Pinger-marker lag is inherent to the source — the filtered seed plus dead
reckoning described in §2 (`robot_interface.odom_callback`); N4 is what keeps it
from *also* dragging behind the robot. Neither is corrected in `mcs/`: the
station supervises, and a station-side workaround would hide a control-stack defect
behind a display that looks right.

**⚠ Line numbers into `BlueBoat-Control` are volatile, and the installed build
can lag the source.** `control_dt` (`self.dt = dbl('control_dt', 0.05)`) and
`safety_distance` are **committed** declared parameters now — the old
working-tree-only caveat no longer applies — but what runs is the copy in
`~/ros2_ws/install/`, which only `colcon build` refreshes. That matters doubly
since the 2026-08-31 local-ENU odom fix (`docs/03_ros_integration.md` §Observations): a
stale install publishes the old hybrid frame (launch-relative yaw over ENU axes)
and silently breaks the GPS map and trajectory following from non-East headings.
Anchor on the symbol name, not the line, and rebuild in `~/ros2_ws` before a field
session so the install matches the SHA the superproject records.

### CSV logs (written by `robot_interface.py` in real-robot runs)
Two layouts. With pinger: date, `relative_*`, `corrected_pinger_*`, GPS, pinger
GPS, thrusters, then raw USBL/IMU. Without pinger: date, `relative_*`,
`target_*`, GPS, thrusters, then raw IMU. `target_*` exists **only** in the
no-pinger layout — in pinger mode it duplicated `corrected_pinger_*`.

---

## 7. Commands known to work

```bash
# run (the one workspace is ~/ros2_ws; env.sh activates .venv + sources
# ROS2 and the overlay)
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
    robot_file:=thrusters_ur controller_type:=MPC trajectory:=circle \
    note:=sim-tuning   # note:= names the poslog CSV, as on the real boat
ros2 launch blueboat_sss_sim full_mission_launch.py \
    world_dir:=$HOME/worlds/<path>/<world> with_control:=true \
    trajectory_file:=$HOME/.config/blueboat_mcs/trajectories/.deployed/<name>.yaml \
    controller_type:=PID note:=sim-dam   # personalized-world launch (needs blueboat_sss_sim built)

# field debugging
ros2 topic echo /mavros/global_position/compass_hdg
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: default"
```

`~/ros2_ws` is the project's only workspace: `BlueBoat-Control`, this station
and every other node are built and run there, on this laptop (`/blueboat_ws`
never existed in this project — do not reintroduce it).

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
ten broad `except Exception` clauses in the ROS-facing and logging paths carry
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
It prints twenty-four checkpoints (`TimeSeries ok`, `GeoReferencer ok`,
`sim gps model ok`, `LoS predictor ok`, `start alignment ok`, `designer ok`,
`deploy guard ok`, `worlds ok`, `launch dialog sim-gps ok`,
`world launch ok`, `sea state ok`, `battery ok`, `store ok`, `stats ok`,
`pinger anchor ok`, `map frame ok`, `sentinel ok`, `georef reset ok`,
`designer anchor ok`, `safe shutdown ok`, `path preview ok`,
`launch crash ok`, `sea box ok`, `window ok`, `SMOKE TEST PASSED`) and aborts on the first failed assertion. It passes on this tree, exit status 0, and runs identically
across all three environment shapes: no `rclpy` (GUI-only); `rclpy` importable
but the overlay unsourced, so `blueboat_interfaces` is missing and the bridge
node comes up with its `/path_request` client disabled; and a fully sourced
workspace. Whenever a real bridge node exists, the blocks that assert what was
published substitute their own node stub.

Four of the §3 non-negotiables are asserted there: `pinger anchor ok` (N4)
holds the pinger world position fixed across odom updates, `sentinel ok` (N2)
proves an origin click is nudged to `1e-3` and that only
`_on_continue_mission()` emits `[0, 0]`, `deploy guard ok` (N8) proves
`io_yaml.deploy_mission` raises — and writes nothing — with no fit or no
`geo_anchor`, deploys correctly through a plain translation fit (including a
legacy `theta_deg ≠ 0` anchor, rotated inline), and `safe shutdown ok`
(N1 / N1b) asserts both the *separation* and the *ordering*: E-STOP publishes
`stop`, is acknowledged by `controller_ready` going `False`, terminates nothing
and leaves the Default/Override label alone; E-STOP + Stop Override publishes
`stop` then `default` and still terminates nothing; only `safe_stop_mission`
and `safe_app_exit` reach `terminate`, and only after the echo. It also pins
the anti-false-confirmation rule — a repeated `param_mode` (param_set's 1 Hz
heartbeat) never counts as an acknowledgement, the already-`default` case
takes its own labelled path, and the timeout path still republishes once at
T/2 and still terminates — plus the supersede rule (a terminating request
upgrades a non-terminating one in flight, never the reverse), the empty-graph
warning, the simulation short-circuit and the no-ROS degraded path. `map frame ok` drives a real
`QMouseEvent` through `MapView` under a synthetic translation `GeoFit`
(`|t| ≈ 44 m`, non-vacuous) and asserts the scene↔world round trip at every
input site, the pre-anchor gate (nothing drawn, clicks refused), the
simulation identity anchor (drawing and clicks immediate) and the glyph
heading source (compass first, else absolute odom yaw). The `GeoReferencer`
block additionally asserts stationary anchoring at `min_pairs`, glitch
robustness of the median/MAD estimate, and that a sustained odom/GPS
inconsistency flips `is_valid` off. `sim gps model ok` pins the simulated
receiver's translation mapping, first-call origin latch and seeded-noise
reproducibility; `launch dialog sim-gps ok` proves a sim launch of an
anchored mission takes the deferred-deploy branch with `gps_simulated` and
a **fixed, repeated** `spawn_yaw:=` equal to the configured heading (N8b),
plus the log note on every branch — raw text kept in `LaunchParameters.note`,
`wire_note()` sanitising it into one CLI token and tagging a simulated run
`sim`/`sim-<note>`, an empty real-water note emitting no argument at all, and
the field visible in both modes (and that non-anchored/real launches are
untouched, and that `parameters()` never sets `world_dir`); the map-frame
block's 0c section asserts the anchor gate holds
in GPS-sim until synthetic fixes arrive; `georef reset ok` proves every
launch starts from a fresh georeferencer. `worlds ok` covers the Qt-free
`core/worlds.py` layer on a synthetic `~/worlds` tree: enumeration (dir
names, newest first, invalid folders skipped), the one-point-inside limits
filter including a legacy `theta_deg` anchor rotated with `deploy_mission`'s
convention, and duplication (provenance patched, geometry byte-identical,
an existing target never touched). `world launch ok` pins the
`full_mission_launch.py` CLI branch (exactly `world_dir` / `with_control` /
`trajectory_file` / `controller_type` / `note`; no `robot_file`/`trajectory`/
`spawn_yaw` leak), the three-way `launch_target` choice, the headless
`WorldChoiceDialog` (world vs Empty Gazebo), and the designer overlay
(rectangle only under a fit, hidden on clear); the `path preview ok` block
additionally proves a world-mode launch leaves SimGps disarmed while the
deferred-deploy poll still arms. `path preview ok` covers the
/path_request lifecycle at both ends: GUI side (the tick issues the pending
request, a failure re-arms it bounded and spaced, mission end cancels and
clears the retry state, a deferred-GPS launch leaves the preview to the
deployment poll) and bridge side, on the real `_poll_path_future` code with
the ROS machinery stubbed (a hung in-flight call is dropped at
`path_request_timeout_s` instead of blocking every later request for the
session, an empty path is a failure rather than a silent blank map, cancel
suppresses even a completed reply from a dead run). `launch crash ok` proves
a launch process that dies on its own returns the manager to `idle` (the
exit watch started by `start()`), instead of wedging the Launch button
forever, and that `stop()`'s SIGTERM/SIGKILL escalation is bound to the
process it was armed for: it is disarmed when the tree exits, and firing a
dead launch's escalation leaves a relaunch untouched (it used to read
`self._proc`/`self._sea_proc` at fire time and killed the *next* run's sea
companion). Its children are started `start_new_session` and handshake on
stdout before the test signals them — a child sharing this script's process
group would SIGINT the smoke test itself, and one still starting up dies on
the SIGINT instead of surviving to be escalated against. `sea box ok` covers the floating SEA STATE overlay:
parented to the map view and hidden outside a running simulation, at
`(8, 8)` against the left panel while the stats box holds the mirror
position at the right edge, the read-out reflecting a parsed readback,
each row measuring its own wrapped height so a longer wave string grows
the box downwards rather than sideways, and the "Modify situation…"
button still wired to `MainWindow._on_modify_sea`. It also asserts the
read-out is *not* duplicated back into the left panel. `designer ok` covers the Qt-free designer
layer — sampling invariants, the save/load/resample round trip, every stock
pattern and interpolation from its own `schema` defaults, and both extension
registries.

`designer anchor ok` builds a real `DesignerWindow` offscreen against a store
whose live fit **is** valid — the exact condition of the old shift bug — and
asserts that a fresh window is un-anchored, that an explicit anchor survives
twenty further GPS fixes, that `_write` serialises *that* anchor and not a
live-derived one, that `New` clears it, that the robot snapshot is one-off and
refuses (visibly) with no fix, and that no overlay item exists on the designer
map. `battery ok` pins the 0..1 → percent conversion, the "no data" state
before any message, that a `None` field does not wipe the last good reading,
and the charge/staleness colour bands.

Still uncovered: the rest of the designer UI (`designer_map.py`, `panels.py`,
`designer_window.py`, including the `_push_undo` contract). And because the
script runs offscreen on synthetic telemetry, a green run is not field
readiness — it cannot catch QoS mismatches, real message-type drift or
launch-file argument errors.

Simulation runs (`Sim_launch.py`) have no MAVROS, no `robot_interface`, no
`param_set` and no pinger; code paths gated on those must degrade quietly.
Personalized-world runs (`full_mission_launch.py`) still have no
`robot_interface`/`param_set`/pinger, but the simulator's mavros shim DOES
publish `/mavros/global_position/global`, `compass_hdg` and `imu/data`.

---

## 8. Documentation set

Four documents, all under `docs/`. Architecture, data flow, the module
inventory and the "how to add X" checklists live in §1 of this file, not in
`docs/` — there is no separate architecture or developer guide.

| Doc | Content |
|---|---|
| `03_ros_integration.md` | every topic / command / service, the launch targets, the stop actions and the safe-shutdown sequence, open control-stack observations |
| `04_user_guide.md` | every screen and control, including the Survey Pattern Designer |
| `05_trajectory_format.md` | the `blueboat_trajectory/1` YAML contract and deferred GPS deployment |
| `GPS_MAP_ARCHITECTURE.md` | the authoritative, self-contained description of the GPS-only map model (§4), written for reuse in any application that draws a GPS vehicle on a real-world map |

`03_ros_integration.md` §Observations carries what is still open or carried
deliberately in the control stack, plus the table of `BlueBoat-Control` fixes that
a stale `~/ros2_ws/install` build silently reintroduces. §6 above owns the station-side half.

Line citations into `BlueBoat-Control` drift whenever that module is edited;
`BlueBoat-Control` owns them, and its own `CLAUDE.md` §2.1.1 records the
section layout they point into. Prefer the symbol name over the line number.

This file stays authoritative where it and `docs/` disagree — `docs/` is the
long-form explanation, not a second source of truth. There is no `TODO.md`:
the module is feature-complete, and what remains is field verification, which
this repository cannot settle.
