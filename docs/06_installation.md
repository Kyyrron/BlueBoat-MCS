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

1. Get the code onto the basestation:

   ```bash
   git clone <your-repo-url> blueboat_mcs        # or copy the folder
   cd blueboat_mcs
   ```

2. Install the Python dependencies. Either system-wide/user:

   ```bash
   pip install --user -r requirements.txt
   ```

   or in a virtual environment — in that case create it with access to the
   ROS site-packages, since `rclpy` comes from ROS, not pip:

   ```bash
   python3 -m venv --system-site-packages .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

3. Source ROS2 and your workspace **before** starting the station (this is
   what makes `rclpy`, `mavros_msgs`, `blueboat_interfaces` and the
   `ros2 launch` executable available to it):

   ```bash
   source /opt/ros/<distro>/setup.bash
   source ~/blueboat_ws/install/setup.bash
   ```

4. Run:

   ```bash
   python3 run.py
   ```

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

Without the boat: `python3 smoke_test.py` must print `SMOKE TEST PASSED`
(runs headless, no ROS needed). With ROS sourced but no boat, start the
station: the status bar shows "ROS: connected" and the diagnostics panel shows
every topic grey ("never") — that is the expected idle state.

## Troubleshooting

* *"ROS: unavailable" in the status bar* — the environment wasn't sourced in
  the shell that started the station.
* *"'ros2' not found" when launching a mission* — same cause; the launch
  subprocess inherits the station's environment.
* *Satellite checkbox stays disabled* — no GPS fix yet, or the boat hasn't
  moved the few metres needed for the georeference (watch "georef" in the
  status bar), or no internet route to the tile server.
* *mavros_msgs / blueboat_interfaces warnings at startup* — those packages are
  missing from the sourced workspace; the corresponding features (FCU state,
  mission-path display) disable themselves and everything else keeps working.
