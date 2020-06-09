#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
source "/root/catkin_ws/devel/setup.bash"


echo "Compile ros project"
cd /root/catkin_ws
rosdep install -y rgbdslam
catkin_make

exec "$@"
