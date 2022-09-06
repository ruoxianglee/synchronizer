# Worst-Case Time Disparity Analysis of Message Synchronization in ROS 
This is a ROS2 Dashing package providing the evaluation test about the worst-case time disparity analysis for [ROS Approximate Time policy](https://github.com/ros2/message_filters/blob/master/include/message_filters/sync_policies/approximate_time.h) and [CyberRT latest policy](https://github.com/ApolloAuto/apollo/blob/master/cyber/data/fusion/all_latest.h).

## Paper
If you use this project in your research, please cite our paper:
```
Ruoxiang Li, Nan Guan, Xu Jiang, Zhishan Guo, Zheng Dong and Mingsong Lv. "Worst-Case Time Disparity Analysis of Message Synchronization in ROS.” 2022 IEEE Real-Time Systems Symposium (RTSS). IEEE, 2022.
```
## Package Structure
```
> tree .
|-- config
    |-- config.yaml                           # Test parameters
|-- include         
    |-- helper.h                              # Some helper functions    
    |-- publisher.h                           # Message publishing node using timer
    |-- subscriber_evaluation.h               # Message subscription node for comparing the latest model and our model
    |-- subscriber_validation.h               # Message subscription node for comparing original ROS2 and our model
|-- message_filters                           # Files in this directory should be put into `message_filters` package
    |-- sync_policies
        |-- approximate_time_model.h          # Our implementation for modeling Approximate Time policy
        |-- latest_policy.h                   # Our implementation for modeling Latest policy
    |-- test
        |-- test_approximate_time_model.cpp   # Google test for our implementation of Approximate Time policy
        |-- test_latest_policy.cpp            # Google test for our implementation of Latest policy
    |-- CMakeLists.txt                        # The build file with the added instructions for message_filters package
|-- results
|-- scripts
    |-- bound_with_delay.py                   # Time disparity upper bound calculation
    |-- compare.py                            # A helper file for comparing the validation results
    |-- histogram_channel_num.py              # Histogram Fig.8 (a) generation 
    |-- histogram_period_factor.py            # Histogram Fig.8 (c) generation 
    |-- histogram_period_lower.py             # Histogram Fig.8 (b) generation  
    |-- histogram_varied_delay.py             # Histogram Fig.8 (d) generation 
|-- src
    |-- evaluation.cpp                        # Evaluation for comparing latest and our model (Sec VI.B in paper)
    |-- validation_rosbag.cpp                 # Validation using ROS bag data (Sec VI.A in paper)
    |-- validation_timer.cpp                  # Validation for comparing ROS2's and our model (Sec VI.A in paper)
|-- prepare.sh                                # The script for automatically adding files and building packages
```

## Before building

### [Build ROS2 Dashing in `ros2_dashing` workspace](https://docs.ros.org/en/dashing/Installation/Ubuntu-Development-Setup.html)
### Clone this project into your workspace
```sh
~/ros2_ws/src
git clone github.com/ruoxianglee/synchronizer
```
### Add files

- Copy files from `message_filters/sync_policies` to `ros2_dashing/src/ros2/message_filters/include/message_filters/sync_policies`

- Copy files from `message_filters/test` to `ros2_dashing/src/ros2/message_filters/test`

### Modify CMakeLists.txt in `message_filters` package

Add the following instructions into CMakeLists.txt:

```
  ament_add_gtest(${PROJECT_NAME}-test_latest_policy test/test_latest_policy.cpp)
  if(TARGET ${PROJECT_NAME}-test_latest_policy)
    target_link_libraries(${PROJECT_NAME}-test_latest_policy ${PROJECT_NAME})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_approximate_time_model test/test_approximate_time_model.cpp)
  if(TARGET ${PROJECT_NAME}-test_approximate_time_model)
    target_link_libraries(${PROJECT_NAME}-test_approximate_time_model ${PROJECT_NAME})
  endif()
```
## Build

### Build `message_filters` package

 ```sh
cd ros2_dashing
colcon build --packages-select message_filters --symlink-install
source ./install/setup.bash
 ```
 
### Build this project
```sh
source ~/ros2_dashing/install/setup.bash
cd ros2_ws
colcon build --packages-select synchronizer --symlink-install
```

## Option
After cloning this project, you can also run the script `prepare.sh` to automatically add files and build the two packages.

**Note**: change `ros2_ws` to the name of your own workspace! The default workspace `ros2_dashing` is used.

## Run
### Run nodes of this project
### Synchronize sensor data generated from SVL simulator comparing our model and original approximate time implementation
Run the synchronizer node:
```sh
ros2 run synchronizer synchronizer_node ./src/synchronizer/results/
```
- Parameter: `./src/synchronizer/results/` is the directory for saving results.

Replay the recorded sensor data:
```sh
ros2 bag play your_data_including_image_points_imu
```

### Synchronize period messages generated using ROS timer comparing our model and original approximate time implementation
```sh
ros2 run synchronizer verification_node 6 50 100 ./src/synchronizer/results/ __params:=./src/synchronizer/config/config.yaml
```
- Parameter 1: the number of message channel ([2,9])
- Parameter 2: the lower limit for random message period generation (10, 20, 30, 40, 50 as used in paper)
- Parameter 3: the number of iteration for each generated message period
- Parameter 4: the directory for saving results
- Parameter 5: other configuration parameters in config.yaml

### Synchronize period messages generated using ROS timer comparing our model and latest policy
```sh
ros2 run synchronizer test_node 6 50 100 ./src/synchronizer/results/ __params:=./src/synchronizer/config/config.yaml
```
- Parameter 1: the number of message channel ([2,9])
- Parameter 2: the lower limit for random message period generation (10, 20, 30, 40, 50 as used in paper)
- Parameter 3: the number of iteration for each generated message period
- Parameter 4: the directory for saving results
- Parameter 5: other configuration parameters in config.yaml

### Run google test
```sh
colcon test --packages-select message_filters
```

or

```
cd ~/ros2_dashing/build/message_filters/test
ctest -V -R message_filters-test_approximate_time_model
```
