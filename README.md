# BlueBoat Mission Control Station (MCS)

Version 1.1

The MCS is the operator's desktop app for the BlueRobotics
[BlueBoat](https://bluerobotics.com/store/boat/blueboat/) surveying with side-scan sonar.
It launches, watches and stops missions on the real boat or in Gazebo. It shows the boat
and its path on a north-up satellite map, with live telemetry. It also holds the
**Survey Pattern Designer**, where you draw survey paths and save them as trajectory files.

The station does **no control computation**. Every number it shows comes from a ROS 2
topic of the control stack (`BlueBoat-Control`). If a value looks wrong, check the node
that publishes it first.

## Where it fits

```
Operator laptop (~/ros2_ws, runs ALL project software)
  MCS  ──ros2 launch──▶  BlueBoat-Control (or BlueBoat-SSS-Sim for Gazebo worlds)
   ▲                        │
   └──── ROS 2 topics ◀─────┘ ◀── MAVROS ◀── WiFi (BlueBoat Base Station) ◀── boat
```

The boat carries only its hardware and its own firmware (ArduPilot / BlueOS).
Nothing of this project runs on the boat.

## Requirements

| Needed | Why |
|---|---|
| Ubuntu 24.04, Python 3.10 or newer | the operator laptop |
| [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html) | `rclpy`, `ros2 launch` |
| `BlueBoat-Control` (built in `~/ros2_ws`) | the launch files and nodes the station drives, and `blueboat_interfaces` |
| [MAVROS](https://github.com/mavlink/mavros/blob/ros2/docs/installation.md#binary-package-deb) | `mavros_msgs` and the link to the boat's autopilot |
| PySide6, numpy, scipy, PyYAML | `requirements.txt` |
| WiFi route to the boat; internet for satellite tiles | tiles are optional, the map works without them |
| `BlueBoat-SSS-Sim` (optional) | simulated missions in generated Gazebo worlds, with sonar and sea state |

Missing pieces disable features instead of crashing: without `rclpy` the app starts as
GUI only.

## Install

The MCS is plain Python, not a colcon package. It ships inside the superproject.

```bash
# 1. Clone the superproject with its submodules into the workspace, then build
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/Kyyrron/BlueBoat-SideScanSonar.git
cd ~/ros2_ws && colcon build

# 2. Source the workspace (and the venv) - every time, before starting the station
cd ~/ros2_ws && source env.sh        # or: source install/setup.bash

# 3. Python dependencies, once
cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-MCS
pip install -r requirements.txt
```

`~/ros2_ws/env.sh` is a small helper you create once, sourced from `~/ros2_ws`:

```bash
#!/usr/bin/env bash
source .venv/bin/activate
source install/setup.bash
```

Notes:
- `rclpy` always comes from ROS, never from pip.
- Inside the workspace venv, do not pass `--user` to pip. A venv you make yourself needs
  `python3 -m venv --system-site-packages`.

## Run

```bash
cd ~/ros2_ws && source env.sh
cd src/BlueBoat-SideScanSonar/BlueBoat-MCS
python3 run.py                       # options: --config <file.json>  --verbose
```

The status bar should read **ROS: connected**. `bash build.sh` does all of it in one go:
`colcon build`, `source env.sh`, then `python3 run.py`.

Check the install without a boat:

```bash
QT_QPA_PLATFORM=offscreen python3 smoke_test.py    # must end with SMOKE TEST PASSED
```

## Main features

- **Launch Mission** in one of three ways: *Real robot*, *Gazebo simulation* (empty
  world, or a generated world from `BlueBoat-SSS-Sim`), or **From a ros2bag**, which
  replays a recorded real run in Gazebo.
- **Run sessions on the real boat.** You connect once. The boat goes into override and
  waits. **START RUN** / **STOP RUN** start and stop each logged run.
  **Update Mission** picks the next run's path and controller.
- **Map**: satellite tiles, boat at its true heading, trail, mission path with its
  start **S**, USBL pinger, the controller's target. Pan, zoom, click to read GPS.
- **Manual Target**: click a point on the map and the boat goes there.
  **Continue Original Mission** resumes the path.
- **Arrow control**: drive the boat with the arrow keys (ARROW CONTROL box on the map).
- **Live panels**: robot, pinger and target state, battery, ROS diagnostics per topic,
  distance plot, mission timeline (freeze the view while recording continues),
  statistics, launch console with filters.
- **Link guardian box** (real robot): shows and controls the autopilot script that brings
  the boat home when the laptop goes silent. It is ON by default in every real session.
- **Simulation options**: sea state (current, waves, timed plans), **Sonar model**
  (Geometric or Ray-cast) and **Slow-down** for the slow ray-cast sonar.
- **Survey Pattern Designer** (**Create Survey Pattern**): draw or generate patterns
  (lawnmower, circle, rectangle, square, figure eight, spiral, expanding square,
  station keeping, polygon), anchor them to GPS, export them.

## Stop buttons and safe shutdown

| Button | What it does | Ends the launch? |
|---|---|---|
| **E-STOP** | zeroes thrust, closes the motor gate, disarms, latches. Stays in override | no |
| **E-STOP + Stop Override** | the same, then hands the servos back to RC / QGroundControl | no |
| **Release E-STOP** | clears the latch (real session launched with motors enabled) | no |
| **Stop Mission** (reads **Turn Off Connection** in a real session) | safe shutdown, then stops every node | yes |

**Safe shutdown.** Stop Mission and closing the app first publish `default` on
`/blueboat/input_str` and wait until the boat confirms it left override. Only then are
the nodes stopped. If no confirmation comes, the station asks you before stopping
anything. Never kill a real-robot launch from the terminal instead: the motors can stay
in override. To do it by hand:

```bash
ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: default"
```

## Files and configuration

| Path | Content |
|---|---|
| `~/.config/blueboat_mcs/config.json` | optional settings override (JSON). Every topic name, threshold and folder is in `mcs/config/settings.py`; put only the keys you change, grouped by section |
| `~/.config/blueboat_mcs/trajectories/` | designer paths: `<name>.yaml` (what the boat follows) and `<name>.meta.yaml` (editor data) |
| `…/trajectories/.deployed/`, `…/trajectories/.session/run.yaml` | written by the station at each launch / run. Never edit by hand |
| `~/.config/blueboat_mcs/sea_states/` | saved sea-state plans and the last launch's sea choice |
| `~/worlds/<path>/<world>/` | generated Gazebo worlds (made with the SSS-Sim World Builder) |
| `~/ros2_ws/data/Robot_data/` | real-robot run logs and their ros2bags (used by the replay) |

## Good to know

- **On real water the map stays empty for about a second**, until GPS fixes anchor the
  frame. No driving is needed. Map clicks are refused until then.
- **`[0, 0]` on `/blueboat/manual_target` means "resume the mission"**, not a position.
  A click exactly on the origin is moved by 1 mm.
- **A stale build breaks things silently.** After pulling `BlueBoat-Control`, run
  `colcon build` again in `~/ros2_ws`.
- **Simulated results for the thesis** use Sonar model *Ray-cast — harbour, fast*.
  Geometric is the dialog default but is a legacy sonar. The allowed settings are in
  `../SIM_VALIDITY_CONDITIONS.md`.
- Control-stack changes belong in `BlueBoat-Control`, never in this repository.

## Directory layout

```
BlueBoat-MCS/
├── run.py              start the app
├── build.sh            colcon build + source env.sh + run.py
├── smoke_test.py       offline regression test
├── requirements.txt
├── docs/               user and developer documentation
└── mcs/
    ├── main.py         app bootstrap (command-line options)
    ├── config/         settings.py: all topics, thresholds, paths
    ├── core/           logic without widgets or ROS (geo, sea state, replay, ...)
    ├── models/         the data store the GUI reads
    ├── ros/            ROS thread, topic bridge, launch manager, commands and shutdown
    ├── gui/            main window, panels, map, plots, dialogs
    └── designer/       Survey Pattern Designer
```

## Documentation

| Read | For |
|---|---|
| [`docs/04_user_guide.md`](docs/04_user_guide.md) | every screen, button and dialog |
| [`docs/05_trajectory_format.md`](docs/05_trajectory_format.md) | the trajectory file format and GPS deployment |
| [`docs/03_ros_integration.md`](docs/03_ros_integration.md) | every topic, service and launch command; the shutdown sequence |
| [`docs/GPS_MAP_ARCHITECTURE.md`](docs/GPS_MAP_ARCHITECTURE.md) | how the map places things from GPS |

Part of the [BlueBoat-SideScanSonar](https://github.com/Kyyrron/BlueBoat-SideScanSonar)
project.

---

**Author** — BERTRAND Killian, Kyushu Institute of Technology, 2026.
