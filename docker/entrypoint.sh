#!/bin/bash
set -e

echo "entrypoint"

# setup ros environment
source "/opt/ros/kinetic/setup.bash"
source "/root/catkin_ws/devel/setup.bash"

apt update

echo "Compile ros project"
cd /root/catkin_ws
rosdep install -y rgbdslam --skip-keys=\"nvidia-cuda-dev nvidia-cuda\"
catkin_make -j2

exec "$@"
