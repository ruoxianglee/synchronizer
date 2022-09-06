#!/bin/bash

# Enter this project after cloning
cd synchronizer

# Prepare files for our implementation of Approximate Time Model and Latest-policy Model
cp message_filters/sync_policies/* ~/ros2_dashing/src/ros2/message_filters/include/message_filters/sync_policies
cp message_filters/test/* ~/ros2_dashing/src/ros2/message_filters/test
cp message_filters/CMakeLists.txt ~/ros2_dashing/src/ros2/message_filters/CMakeLists.txt

# Enter ROS2 Dashing workspace
cd ~/ros2_dashing

# Build message_filters package, add header files in `sync_policies`
colcon build --packages-select message_filters --symlink-install
source ~/ros2_dashing/install/setup.bash

# Enter your own workspace. Change to the name of your own workspace. Default ros2_dashing is used.
# cd ros2_ws

# Build this package
colcon build --packages-select synchronizer --symlink-install