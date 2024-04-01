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