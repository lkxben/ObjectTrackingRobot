#!/bin/bash

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --packages-select robot_msgs
source install/setup.bash

colcon build --symlink-install --packages-select robot
source install/setup.bash

echo "ROS workspace ready!"