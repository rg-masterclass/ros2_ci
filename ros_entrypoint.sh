#!/bin/bash
set -e

# setup ros environment
source /opt/ros/$ROS_DISTRO/setup.bash
source /galactic_ws/install/setup.bash

# sleep 5s

$@
