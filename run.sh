#!/bin/bash

source ~/ros2_dashing/install/setup.bash

# Number of test group
num_group=2

#########################################################
###################  Validation 1  ######################
#########################################################
# Validation with different number of channels
ros2 run synchronizer validation_timer 2 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/channel_num/channel_2 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 3 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/channel_num/channel_3 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 4 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/channel_num/channel_4 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 5 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/channel_num/channel_5 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/channel_num/channel_6 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 7 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/channel_num/channel_7 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 8 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/channel_num/channel_8 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 9 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/channel_num/channel_9 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml

# Validation with changed periods
ros2 run synchronizer validation_timer 6 10 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_periods/varied_period_lower_10 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 6 20 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_periods/varied_period_lower_20 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 6 30 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_periods/varied_period_lower_30 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 6 40 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_periods/varied_period_lower_40 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer validation_timer 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_periods/varied_period_lower_50 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml

# Validation with varied timestamp separation 
ros2 run synchronizer validation_timer 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_period_factor/varied_period_factor_1.0 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_2.yaml
ros2 run synchronizer validation_timer 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_period_factor/varied_period_factor_1.2 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_3.yaml
ros2 run synchronizer validation_timer 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_period_factor/varied_period_factor_1.4 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_4.yaml
ros2 run synchronizer validation_timer 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_period_factor/varied_period_factor_1.6 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_5.yaml
ros2 run synchronizer validation_timer 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/validation_timer/varied_period_factor/varied_period_factor_1.8 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_6.yaml

#########################################################
###################  Evaluation  ########################
#########################################################
# Evaluation with different number of channels
ros2 run synchronizer evaluation 3 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/channel_num/channel_3 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 4 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/channel_num/channel_4 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 5 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/channel_num/channel_5 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/channel_num/channel_6 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 7 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/channel_num/channel_7 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 8 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/channel_num/channel_8 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 9 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/channel_num/channel_9 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml

# Evaluation with changed periods
ros2 run synchronizer evaluation 6 10 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_periods/varied_period_lower_10 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 6 20 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_periods/varied_period_lower_20 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 6 30 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_periods/varied_period_lower_30 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 6 40 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_periods/varied_period_lower_40 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_periods/varied_period_lower_50 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_1.yaml

# Evaluation with varied timestamp separation 
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_period_factor/varied_period_factor_1.0 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_2.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_period_factor/varied_period_factor_1.2 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_3.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_period_factor/varied_period_factor_1.4 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_4.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_period_factor/varied_period_factor_1.6 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_5.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_period_factor/varied_period_factor_1.8 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_6.yaml

# Evaluation with varied delay
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_delay/random_delay_10 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_7.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_delay/random_delay_20 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_8.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_delay/random_delay_30 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_9.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_delay/random_delay_40 __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_10.yaml
ros2 run synchronizer evaluation 6 50 $num_group /home/paper164/ros2_dashing/src/synchronizer/results/evaluation/varied_delay/random_nodelay __params:=/home/paper164/ros2_dashing/src/synchronizer/config/config_11.yaml

clear

# Compare the validation result
python3 /home/paper164/ros2_dashing/src/synchronizer/scripts/compare_1.py

# Generate the evalution illustrations
python3 /home/paper164/ros2_dashing/src/synchronizer/scripts/histogram_channel_num.py
python3 /home/paper164/ros2_dashing/src/synchronizer/scripts/histogram_period_lower.py
python3 /home/paper164/ros2_dashing/src/synchronizer/scripts/histogram_period_factor.py
python3 /home/paper164/ros2_dashing/src/synchronizer/scripts/histogram_varied_delay.py