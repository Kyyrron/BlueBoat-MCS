#!/bin/bash

# Stop if fails
set -e

cd ~/ros2_ws/
colcon build

source env.sh

cd src/BlueBoat-SideScanSonar/blueboat_mcs
python3 run.py