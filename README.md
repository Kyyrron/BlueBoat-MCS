# BlueBoat Mission Control Station

Operator station for BlueRobotics BlueBoat field experiments: mission
launching, live monitoring, robot & controller supervision, USBL pinger
tracking, manual targeting and emergency actions — in the spirit of
QGroundControl / Mission Planner, built on the lab's existing ROS2 stack
(`blueboat_control`). The station supervises and commands; it performs no
control computation and duplicates no logic already available on a topic.

```
cd ~/ros2_ws && source env.sh          # basestation workspace (ROS2 + overlay)
cd src/BlueBoat-SideScanSonar/BlueBoat-MCS
pip install -r requirements.txt        # first time only; env.sh activates .venv
python3 run.py
```

The station runs on the **basestation**, whose workspace is `~/ros2_ws`. The
boat has a separate workspace, `/blueboat_ws` (with its own `.venv`), where
`BlueBoat-Control` is built and run — the two are easy to confuse and are not
interchangeable.

## Highlights

* Interactive world-frame map: robot + trajectory, mission path (fetched from
  the existing `/path_request` service), USBL pinger position/trajectory,
  robot→target line, LoS-approximation preview, adaptive metric grid,
  optional satellite imagery via an online odom↔GPS georeference.
* Manual Target mode publishing `/blueboat/manual_target` (with the `[0,0]`
  resume protocol of `master_control.py`), distance/measure tools, click
  inspector with GPS read-out.
* Live panels: robot / pinger / target information, per-topic ROS diagnostics
  with rate/age LEDs, robot↔target distance plot, dual-handle mission
  timeline (freeze display, keep recording), windowed mission statistics.
* Mission lifecycle: configurable `ros2 launch` dialog, readiness gating,
  SIGINT-first graceful stop, and a sequenced Emergency Stop that publishes
  `default` on `/blueboat/input_str` and confirms transmission **before** any
  node is terminated.

## Documentation

| Doc | Content |
|---|---|
| `docs/01_architecture.md` | design, threading model, decisions |
| `docs/02_developer_guide.md` | conventions, data flow, how-to checklists |
| `docs/03_ros_integration.md` | every topic/command/service + flagged robot-side issues |
| `docs/04_user_guide.md` | every interface feature |
| `docs/05_handover.md` | extension points and cautions |
| `docs/06_installation.md` | setup and troubleshooting |
| `docs/07_getting_started.md` | zero-to-operating in five minutes |
| `docs/08_trajectory_format.md` | the `blueboat_trajectory/1` YAML contract |
| `docs/HEADING_AND_MAP_ALIGNMENT.md` | heading sources and map frames (reusable note) |

Verified headless via `QT_QPA_PLATFORM=offscreen python3 smoke_test.py`
(no ROS, no display required).

Dependencies: PySide6, numpy, scipy (pip) + rclpy / mavros_msgs /
blueboat_interfaces from the sourced ROS2 workspace.
