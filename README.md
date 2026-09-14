# BlueBoat Mission Control Station

v1.0 - September 14th - 2026

Operator ground station for autonomous side-scan-sonar surveys with a
BlueRobotics **BlueBoat** — an unmanned surface vessel (USV) for shallow-water
survey work ([bluerobotics.com](https://bluerobotics.com/store/boat/blueboat/)).

The station runs on the basestation laptop, not on the boat. It launches and
supervises missions, draws the boat and its mission on a north-up satellite map,
plots live telemetry, and carries the emergency-stop and safe-shutdown sequences.
It performs **no control computation**: every number on screen comes from a ROS 2
topic published by the boat's own stack. It also contains the **Survey Pattern
Designer**, an editor for drawing survey paths and exporting them as trajectory
files the boat executes.

Part of the [BlueBoat-SideScanSonar](https://github.com/Kyyrron/BlueBoat-SideScanSonar)
project.

---

## Dependencies

| Needed | Why |
|---|---|
| Ubuntu 22.04 / 24.04, Python ≥ 3.10 | the basestation laptop (24.04)|
| [ROS 2](https://docs.ros.org/en/jazzy/Installation.html) (Jazzy on the reference machine) | `rclpy`, `ros2 launch` |
| [BlueBoat-Control](https://github.com/Kyyrron/BlueBoat-Control) | `blueboat_control` (the launch files and nodes the station drives) and `blueboat_interfaces` (the `/path_request` service type) |
| [mavros](https://github.com/mavlink/mavros/blob/ros2/docs/installation.md#binary-package-deb) | `mavros_msgs`, and the MAVLink link to the boat's ArduPilot |
| PySide6, numpy, scipy, PyYAML | `requirements.txt` |
| Network route to the boat; internet for satellite tiles | tiles are optional — the map works without them |

Optional, for simulated missions in personalized Gazebo worlds:
[BlueBoat-SSS-Sim](https://github.com/Kyyrron/BlueBoat-SSS-Sim).

Missing pieces degrade rather than crash: without `rclpy` the app starts
GUI-only; without `mavros_msgs` or `blueboat_interfaces` the features that need
them disable themselves.

## Installation

1. Install ROS 2 and mavros, and create the **basestation** workspace + src folder `~/ros2_ws/src`.

2. Clone the superproject into it, with submodules, and build:

   ```bash
   cd ~/ros2_ws/src
   git clone --recurse-submodules https://github.com/Kyyrron/BlueBoat-SideScanSonar.git
   cd ~/ros2_ws && colcon build
   ```

3. Source the workspace **before** starting the station — that is what puts
   `rclpy` and `ros2 launch` on the path. Create also a Python virtual environment (venv). 
   
   Tips: create this file as `~/ros2_ws/env.sh`
   ```bash
    #!/usr/bin/env bash
    source .venv/bin/activate
    source install/setup.bash
   ```
   So that you can do this 
   ```bash
   cd ~/ros2_ws && source env.sh          # or: source install/setup.bash otherwise
   ```
   It will source both your ros2 workspace _and_ the Python venv (when named `.venv`). 

4. Install the Python dependencies into that environment, once:

   ```bash
   cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-MCS
   pip install -r requirements.txt
   ```

   `rclpy` always comes from ROS, never from pip. Inside the workspace venv do
   not pass `--user`; against a bare system Python use `pip install --user`. A
   hand-made venv must be created with `python3 -m venv --system-site-packages`.

## Starting the MCS App

When in `~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-MCS`, run:

   ```bash
   python3 run.py
   ```

   The status bar should read **ROS: connected**. Verify without a boat:
   `QT_QPA_PLATFORM=offscreen python3 smoke_test.py` must print
   `SMOKE TEST PASSED`.

__`build.sh` does ros2 colcon build -> sources env.sh -> starts MCS App in one go.__

## Features

* Launch, supervise and stop ROS 2 missions — real boat, Gazebo simulation, or a
  personalized Gazebo world with simulated sonar and sea state (when [BlueBoat-SSS-Sim](https://github.com/Kyyrron/BlueBoat-SSS-Sim) is installed).
* North-up satellite map with the boat at its true heading, its trail, the
  mission path, the USBL pinger and the controller's current target.
* Manual targeting by clicking the map, distance measurement, click inspector
  with GPS read-out.
* Live panels: robot / pinger / target state, battery, per-topic ROS diagnostics
  with rate and staleness LEDs.
* Distance plot, mission timeline (freeze the display while recording
  continues), windowed mission statistics, filterable launch console.
* Three separate stop actions: E-STOP, E-STOP + Stop Override, Stop Mission.
* Survey Pattern Designer: draw or generate survey patterns, anchor them to real
  GPS coordinates, export them as trajectory files the boat runs.

## Usage

Start the station, press **Launch Mission**, configure the run, and watch it.
The window has four regions:

| Region | What it holds |
|---|---|
| **Left panel** | map layer checkboxes; robot / pinger / target read-outs; ROS diagnostics, one row per topic |
| **Centre — the map** | satellite imagery, boat glyph, trails, mission path, targets. Drag to pan, wheel to zoom, click to inspect a point |
| **Right panel** | zoom and centre buttons, robot↔target distance plot, mission timeline, statistics, launch console |
| **Bottom toolbar** | Launch Mission · Stop Mission · E-STOP · E-STOP + Stop Override · Default/Override toggle · Manual Target · Measure · Create Survey Pattern |

The buttons that matter:

* **Launch Mission** — a dialog for controller, trajectory, pinger and motors
  (motor enable is always re-confirmed). Simulation runs add a world dialog and a
  sea-state dialog.
* **Manual Target** — press it, then click the map once; that point is published
  as the boat's target and the button disarms itself. **Continue Original
  Mission** hands control back to the mission.
* **E-STOP** — zeroes the thrust and latches it. It does *not* leave override and
  does *not* end the mission. Clear it by publishing `enable`:
  `ros2 topic pub --once /blueboat/input_str std_msgs/msg/String "data: enable"`.
* **E-STOP + Stop Override** — the same stop, then hands the servo mapping back
  to the RC transmitter / QGroundControl. Still does not end the mission.
* **Stop Mission** — the only button that terminates the launch, and only after
  the boat has confirmed it left override.
* **Create Survey Pattern** — opens the Survey Pattern Designer.

Details: [`docs/04_user_guide.md`](docs/04_user_guide.md) for every screen and
control, [`docs/05_trajectory_format.md`](docs/05_trajectory_format.md) for the
trajectory file format. Working on the code: `.claude/CLAUDE.md` is the
authoritative reference (architecture, ROS contract, non-negotiables), with
[`docs/03_ros_integration.md`](docs/03_ros_integration.md) and
[`docs/GPS_MAP_ARCHITECTURE.md`](docs/GPS_MAP_ARCHITECTURE.md) alongside it.

## Worth knowing

* **On real water the map is empty for the first few seconds.** Nothing is drawn
  until GPS anchors the frame — an empty map is honest, a wrongly-placed one is
  not. It resolves in about a second, with no driving needed. Manual clicks are
  refused until then.
* **`[0, 0]` on `/blueboat/manual_target` is a protocol sentinel** meaning
  "resume the mission", so a click exactly on the world origin is nudged by 1 mm.
  Never publish it as a position.
* **The station never compensates for the boat.** If a number looks wrong, the
  producing node is the first suspect. A boat running a stale `BlueBoat-Control`
  build silently breaks the map and trajectory following.
* **Robot-side changes belong in `BlueBoat-Control`**, never here. Nothing is
  copied out of this repo onto the boat.

---

**Author** — BERTRAND Killian, Kyushu Institute of Technology, 2026.
