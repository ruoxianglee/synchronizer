cmake_minimum_required(VERSION 3.5)
project(synchronizer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(builtin_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

set(dependencies
  rclcpp
  message_filters
  sensor_msgs
)

add_executable(validation_rosbag src/validation_rosbag.cpp)
target_include_directories(validation_rosbag PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(validation_rosbag
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

add_executable(evaluation src/evaluation.cpp)
target_include_directories(evaluation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(evaluation
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

add_executable(validation_timer src/validation_timer.cpp)
target_include_directories(validation_timer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(validation_timer
  "rclcpp"
  "message_filters"
  "sensor_msgs"
  )

install(TARGETS 
  validation_rosbag
  evaluation
  validation_timer
  DESTINATION lib/${PROJECT_NAME})


# Install files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
