cmake_minimum_required(VERSION 3.0.2)
project(ieskf_slam)

SET(CMAKE_CXX_FLAGS "-std=c++17 -O3")
add_compile_options(-std=c++17)


add_definitions(-DPROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
)

find_package(PCL REQUIRED)
# YAML CPP
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieskf_slam
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
)


SET(IESKF_SLAM_FILE 
  src/modules/frontend/frontend.cc
  src/ieskf/ieskf.cc
  src/map/local_map.cc
  src/modules/propagate.cc
)

SET(FRONTEDN_ROS_WRAPPER
  src/ros_wrapper/frontend_ros_wrapper.cc
)

add_executable(ieskf_node 
  src/app/ieskf_node.cc
  ${IESKF_SLAM_FILE}
  ${FRONTEDN_ROS_WRAPPER}
)

target_link_libraries( ieskf_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)