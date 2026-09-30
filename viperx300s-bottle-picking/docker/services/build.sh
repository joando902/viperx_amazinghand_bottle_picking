#!/bin/bash

apt update
rosdep update

# 1. build april_tag ws
cd /home/apriltag_ws
# rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w"

# 2. build interbotix_ws
cd /home/interbotix_ws
# rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_CXX_FLAGS="-w"