cmake_minimum_required(VERSION 2.8.3)
project(imu_low_pass_filter)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_low_pass_filter src/imu_low_pass_filter_node.cpp)
target_link_libraries(imu_low_pass_filter ${catkin_LIBRARIES})

add_executable(imu_2low_pass_filter src/imu_2low_pass_filter_node.cpp)
target_link_libraries(imu_2low_pass_filter ${catkin_LIBRARIES})
