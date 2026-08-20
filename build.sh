#!/bin/bash

# Stop if fails
set -e

cd ~/ros2_ws/
colcon build

source env.sh

cd src/BlueBoat-SideScanSonar/BlueBoat-MCS
python3 run.py