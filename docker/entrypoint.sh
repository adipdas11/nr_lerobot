#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /opt/micro_ros_humble/setup.bash ]; then
  source /opt/micro_ros_humble/setup.bash
fi

if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi

exec "$@"
