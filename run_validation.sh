#!/bin/bash

source ~/ros2_dashing/install/setup.bash

#########################################################
###################  Validation 2  ######################
#########################################################
# Validation using rosbag data
ros2 run synchronizer validation_rosbag /home/paper164/ros2_dashing/src/synchronizer/results/validation_rosbag
# gnome-terminal -x bash -c "source ~/ros2_dashing/install/setup.bash; ros2 bag play /home/paper164/ros2_dashing/src/synchronizer/rosbag/data1_lidar1_cam2_imu100"

