#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "$0")"
if [ ! -f /opt/ros/jazzy/setup.bash ]; then
  echo "ROS 2 Jazzy is not installed. See README.md."
  exit 1
fi
source /opt/ros/jazzy/setup.bash
if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
  echo "GUI display is unavailable. Check WSLg."
  exit 1
fi
exec ros2 launch ./demo.launch.py

