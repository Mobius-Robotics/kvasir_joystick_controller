#!/usr/bin/env bash

# Script to be ran as in the systemd service file that starts this node on bootup.

set -e

source /opt/ros/humble/setup.bash

source ~/ros2_ws/install/setup.bash

ros2 launch loki_joystick_controller joystick.launch.py
