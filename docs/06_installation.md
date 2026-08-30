# 6 — Installation Guide

## Requirements

* Ubuntu 22.04 / 24.04 (the basestation laptop used for experiments)
* Python ≥ 3.10
* ROS2 (the distribution your BlueBoat workspace is built against), with the
  workspace providing `blueboat_control`, `blueboat_interfaces` and
  `mavros_msgs` built and sourceable
* Network route to the boat (the usual MAVROS `udp://:14550@192.168.2.2:14550`
  link) and, for the satellite layer only, internet access to the tile server

## Steps

1. Get the code onto the basestation. The station is a submodule of the
   `BlueBoat-SideScanSonar` superproject, checked out inside the basestation
   ROS2 workspace `~/ros2_ws`:

   ```bash
   cd ~/ros2_ws/src
   git clone --recurse-submodules <superproject-url> BlueBoat-SideScanSonar
   cd BlueBoat-SideScanSonar/BlueBoat-MCS
   ```

   (`--recurse-submodules` matters: without it the module directories are
   empty. On an existing clone, `git submodule update --init --recursive`.)

2. Source the basestation workspace **before** anything else (this is what
   makes `rclpy`, `mavros_msgs`, `blueboat_interfaces` and the `ros2 launch`
   executable available to the station). `~/ros2_ws/env.sh` does both halves —
   it activates the workspace virtual environment and sources
   `install/setup.bash`:

   ```bash
   cd ~/ros2_ws && source env.sh
   ```

   The equivalent by hand, if you are not using `env.sh`:

   ```bash
   source /opt/ros/<distro>/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

   **Which workspace:** `~/ros2_ws` is the **basestation's** — the station and
   the SSS applications run from it. `/blueboat_ws`, with its own `.venv`, is
   the **boat's**, where `BlueBoat-Control` is built and launched. They are not
   interchangeable, and this is the most common setup mistake.

3. Install the Python dependencies into that environment (first time only):

   ```bash
   pip install -r requirements.txt
   ```

   Do **not** add `--user` while the workspace `.venv` is active — pip refuses
   it. If you are running against a bare system Python with no venv,
   `pip install --user -r requirements.txt` is the right form. Either way
   `rclpy` comes from ROS, never from pip; a hand-made venv must therefore be
   created with `python3 -m venv --system-site-packages`.

4. Run, from the module root:

   ```bash
   cd ~/ros2_ws/src/BlueBoat-SideScanSonar/BlueBoat-MCS
   python3 run.py
   ```

   `build.sh` does the whole sequence (a `colcon build` in `~/ros2_ws`, then
   `source env.sh`, then `run.py`).

## Configuration (optional)

Defaults match the stack as provided. To override anything (topic names,
diagnostic thresholds, tile server, LoS approximation gains…), create
`~/.config/blueboat_mcs/config.json` containing only the fields you change,
mirroring the dataclass structure in `mcs/config/settings.py`:

```json
{
  "topics": { "monitoring": "/blueboat/monitoring_data" },
  "map":    { "ui_refresh_hz": 15 }
}
```

or pass a file explicitly: `python3 run.py --config my_config.json`.

## Verifying the installation

Without the boat: `QT_QPA_PLATFORM=offscreen python3 smoke_test.py` must
print `SMOKE TEST PASSED` (runs headless; works with or without ROS sourced). With ROS sourced but no boat, start the
station: the status bar shows "ROS: connected" and the diagnostics panel shows
every topic grey ("never") — that is the expected idle state.

## Troubleshooting

* *"ROS: unavailable" in the status bar* — the environment wasn't sourced in
  the shell that started the station.
* *"'ros2' not found" when launching a mission* — same cause; the launch
  subprocess inherits the station's environment.
* *Satellite checkbox stays disabled* — no GPS fix yet, or no internet route
  to the tile server. The tiles need only the **first** GPS fix: the
  georeference emits a translation-only fit immediately, which is enough to
  place them. (Motion is needed for something else — see the next entry.)
* *Tiles are up but the mission path never draws, and the `N↑` badge does not
  appear* — the georeference is not heading-aligned yet. Rotation is
  unobservable while the boat is stationary; drive a few metres and it
  resolves. Expected early in a run, not a fault.
* *mavros_msgs / blueboat_interfaces warnings at startup* — those packages are
  missing from the sourced workspace; the corresponding features (FCU state,
  mission-path display) disable themselves and everything else keeps working.
