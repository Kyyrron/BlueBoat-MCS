# 1 — Architecture Documentation

BlueBoat Mission Control Station (MCS) — the operator interface used during field
experiments for robot supervision, mission control, controller monitoring, USBL
pinger tracking and live experiment monitoring.

## Guiding principles

The station **supervises** the running ROS2 system; it performs **no control
computation**. Whenever a quantity already exists on a ROS2 topic, the station
subscribes to it instead of recomputing it. The only computations performed
locally are pure display transforms: the pinger body→world rotation, distances,
travelled-distance integration, an online odom↔GPS georeference for map clicks
and satellite tiles, and an explicitly-approximate sketch of the LoS future
path. The application must remain responsive and flicker-free across
multi-hour experiments, and every module must be replaceable without touching
its neighbours.

## Layered structure

```
┌────────────────────────────  GUI thread  ────────────────────────────┐
│  gui/            main_window · left_panel · right_panel · toolbar    │
│                  map (view, items, tiles, tools) · plot              │
│        reads on 10 Hz tick                 emits user intents        │
│  models/store    DataStore: live states + full-experiment histories  │
│  core/           SignalBus · TimeSeries · GeoReferencer · predictor  │
├───────────────────────  Qt queued signals  ──────────────────────────┤
│  ros/            RosManager (thread) → BridgeNode (subs/pubs/service)│
│                  LaunchManager (ros2 launch subprocess)              │
│                  CommandCenter (E-STOP sequence, mode toggle)        │
└────────────────────────────  ROS thread  ────────────────────────────┘
```

### The thread boundary: `SignalBus`

`RosManager` spins a single `BridgeNode` in a `SingleThreadedExecutor` on a
dedicated thread. Every ROS callback ends in a signal emission on the
`SignalBus`; Qt queues cross-thread signals, so all slots on the GUI side run
in the GUI thread. No widget imports rclpy, and no ROS callback touches a
widget. Commands travel the other way through thread-safe publisher wrappers
on the bridge node (`publish()` is safe from foreign threads).

If `rclpy` or `mavros_msgs` is not importable (development laptop without a
sourced ROS environment) the station starts in a degraded GUI-only mode and
says so in the status bar — nothing crashes.

### The refresh tick

Telemetry arrives at 20–50 Hz across several topics. Repainting per message
causes flicker and event-queue backlog. Instead, signals update the
`DataStore` immediately (cheap numpy appends), and one `QTimer` at 10 Hz asks
the three panels and the map to *pull* the current state and repaint once.
This single decision is what keeps the UI smooth during long experiments.

### Recording model

Every quantity is stored in a `TimeSeries` — a capacity-doubling numpy buffer
keyed by `time.monotonic()` reception time — so recording is unbounded and
uniform across topics. The timeline range slider selects a *display window*;
windowed reads are binary-searched numpy views with stride decimation capped
at `map.trajectory_max_points_drawn` points per repaint. Recording never
stops, whatever the window shows; snapping the high handle to the end of the
range restores live following.

### Map

A `QGraphicsView` in metres with a y-flip. **The view never rotates**: north-up
is achieved by choosing what frame the scene is in, not by rotating the camera —
rotating the view would spin the satellite tiles with it. There is exactly
**one scene frame**: local east/north metres about the latched GPS origin
`(lat0, lon0)`. Every world-frame quantity (glyph, trails, pinger, targets,
mission path) is placed through the pure translation `GeoFit.world_to_enu()`
(`EN = world + t`); satellite tiles are axis-aligned from the origin. Nothing
is drawn — and manual-target clicks are refused — until the anchor exists
(`store.map_frame_ready()`): a "waiting for GPS fix" notice shows instead. In
simulation the anchor is the identity and drawing starts immediately, tiles
off.

Only the robot glyph rotates, and its heading comes from an absolute source
(compass, else the absolute ENU odom yaw — see `GPS_MAP_ARCHITECTURE.md` at
the repo root). `MapView._to_world()` is the exact inverse of `_to_scene()`,
and every mouse position read back out of the view goes through it.

Constant-pixel-size glyphs (`ItemIgnoresTransformations`) for the boat and
markers, cosmetic pens for lines, an adaptive 1/2/5-decade metric grid painted
in `drawBackground`, and a satellite `TileLayer` at z = −100. Interaction modes
(`NORMAL` / `MANUAL_TARGET` / `MEASURE`) are a small state machine inside the
view; the view emits intents (`target_clicked`, `point_inspected`) and never
publishes anything itself.

### Georeferencing

`/blueboat/odom` is **local ENU** (origin = launch point, axes East/North,
yaw absolute), so the only unknown between the world frame and GPS is a
**translation** `t = EN(world origin)` — never published by the robot, but
trivially estimable. `GeoReferencer` pairs each GPS fix with the concurrent
odom pose (GPS rate, odom-freshness guard), takes the per-axis **median**
over a rolling window, and reports a MAD-robust residual as the health
figure. `is_valid` needs only `min_pairs` (~1 s of GPS) under the residual
threshold — **no vehicle motion**, so a station-keeping boat anchors too
(what makes deferred GPS-anchored deployment work). There is deliberately no
rotation estimation: with an ENU odom frame the true rotation is zero by
construction — the old two-stage Kabsch model is deleted (see
`GPS_MAP_ARCHITECTURE.md` §7 for why it could never work). The fit quality
(robust RMS + pair count) is shown in the status bar.

### Mission lifecycle

`LaunchManager` runs `ros2 launch blueboat_control BlueBoat_launch.py …` in
its own process session, streams output to the toolbar console, and stops it
SIGINT-first (exactly what Ctrl-C does in a terminal, which `ros2 launch`
propagates to every node), escalating to SIGTERM/SIGKILL only on timeout.
`CommandCenter` sequences the Emergency Stop: publish `default` on
`/blueboat/input_str` → wait for the `/blueboat/param_mode` echo (timeout +
DDS flush delay fallback) → only then terminate nodes if requested. Nodes are
never killed before the emergency command is transmitted.

## Module inventory

| Module | Responsibility | Depends on |
|---|---|---|
| `config/settings.py` | every topic name, threshold, gain; JSON overrides | — |
| `core/signals.py` | thread boundary | Qt Core |
| `core/series.py` | timestamped ring buffers | numpy |
| `core/geo.py` | odom↔GPS fit, mercator helpers | numpy |
| `core/los_predictor.py` | display-only LoS path sketch | — |
| `models/store.py` | states, histories, derived stats | core |
| `designer/` | Survey Pattern Designer (Qt-free model + Qt UI) | core, PyYAML |
| `ros/ros_manager.py` | rclpy lifecycle | rclpy |
| `ros/bridge_node.py` | subs, pubs, path service, topic stats | rclpy, msgs |
| `ros/launch_manager.py` | ros2 launch subprocess | Qt Core |
| `ros/command_center.py` | E-STOP sequence, mode toggle | above |
| `gui/*` | presentation only | models, core |

`core/` and `models/` are Qt-widget-free and ROS-free (except `QObject` for
signals), which is what makes the offline smoke test possible.
