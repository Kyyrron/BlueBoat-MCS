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

A `QGraphicsView` whose scene coordinates are ROS world metres with a y-flip
(north-ish up). Constant-pixel-size glyphs (`ItemIgnoresTransformations`) for
the boat and markers, cosmetic pens for lines, an adaptive 1/2/5-decade metric
grid painted in `drawBackground`, and a satellite `TileLayer` at z = −100.
Interaction modes (`NORMAL` / `MANUAL_TARGET` / `MEASURE`) are a small state
machine inside the view; the view emits intents (`target_clicked`,
`point_inspected`) and never publishes anything itself.

### Georeferencing

The world frame is defined inside `robot_interface.py` by an origin and yaw
offset that are never published. The station estimates the identical
similarity transform online: it pairs odometry positions with GPS fixes
projected to local east/north metres, and Kabsch-fits rotation + translation
(scale fixed to 1) over a sliding window once the boat has moved a few metres.
The fit quality (RMS residual) is shown in the status bar; GPS read-outs and
the satellite layer only activate when the fit is trustworthy.

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
| `ros/ros_manager.py` | rclpy lifecycle | rclpy |
| `ros/bridge_node.py` | subs, pubs, path service, topic stats | rclpy, msgs |
| `ros/launch_manager.py` | ros2 launch subprocess | Qt Core |
| `ros/command_center.py` | E-STOP sequence, mode toggle | above |
| `gui/*` | presentation only | models, core |

`core/` and `models/` are Qt-widget-free and ROS-free (except `QObject` for
signals), which is what makes the offline smoke test possible.
