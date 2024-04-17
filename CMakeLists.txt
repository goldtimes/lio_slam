cmake_minimum_required(VERSION 3.0.2)
project(ieskf_slam)

SET(CMAKE_CXX_FLAGS "-std=c++17 -O3")
set(CMAKE_BUILD_TYPE Release)
ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")
add_definitions(-DPROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  livox_ros_driver  ## for livox
  eigen_conversions
  message_generation
  rosbag
  std_msgs
  tf
  geometry_msgs
  nav_msgs
)

find_package(PCL REQUIRED)
# YAML CPP
find_package(PkgConfig REQUIRED)
find_package(Ceres REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES ieskf_slam
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs
  # DEPENDS EIGEN3 PCL Ceres
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDEDIR}
  # src/ceres_pose_graph
)

include(cmake/packages.cmake)

include_directories(
  include 
)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)

add_library(${PROJECT_NAME}.common
  src/common/timer.cc)

target_link_libraries(${PROJECT_NAME}.common
)

add_library(${PROJECT_NAME}.lidarProcess
  src/lidar_process/lidar_process.cc)

target_link_libraries(${PROJECT_NAME}.lidarProcess
  ${third_party_libs}
)

