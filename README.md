# BlueBoat Mission Control Station

Operator station for BlueRobotics BlueBoat field experiments: mission
launching, live monitoring, robot & controller supervision, USBL pinger
tracking, manual targeting and emergency actions — in the spirit of
QGroundControl / Mission Planner, built on the lab's existing ROS2 stack
(`blueboat_control`). The station supervises and commands; it performs no
control computation and duplicates no logic already available on a topic.

```
source /opt/ros/<distro>/setup.bash && source ~/blueboat_ws/install/setup.bash
pip install --user -r requirements.txt
python3 run.py
```

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

Verified headless via `python3 smoke_test.py` (no ROS, no display required).

Dependencies: PySide6, numpy, scipy (pip) + rclpy / mavros_msgs /
blueboat_interfaces from the sourced ROS2 workspace.
