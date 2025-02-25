cmake_minimum_required(VERSION 2.8.3)
project(livox_filter_2)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  livox_ros_driver
  livox_ros_driver2
  sensor_msgs
)

## Declare a catkin package
catkin_package()

## Build
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(livox_filter_2 src/livox_filter_2.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(livox_filter_2
  ${catkin_LIBRARIES}
)
