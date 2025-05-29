cmake_minimum_required(VERSION 3.0.2)
project(perception)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  rosbag
  roscpp
  rospy
  sensor_msgs
  tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
)

catkin_python_setup()

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable for save_cloud_main
add_executable(perception_save_cloud_main src/save_cloud_main.cpp)

## Rename C++ executable without prefix
set_target_properties(perception_save_cloud_main PROPERTIES OUTPUT_NAME save_cloud PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(perception_save_cloud_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(perception_save_cloud_main
  ${catkin_LIBRARIES}
)

## Declare a C++ executable for point_cloud_demo
add_executable(perception_point_cloud_demo 
  src/point_cloud_demo.cpp
  src/crop.cpp
  src/downsample.cpp
)

## Rename C++ executable without prefix
set_target_properties(perception_point_cloud_demo PROPERTIES OUTPUT_NAME point_cloud_demo PREFIX "")

## Add cmake target dependencies of the executable
add_dependencies(perception_point_cloud_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(perception_point_cloud_demo
  ${catkin_LIBRARIES}
)