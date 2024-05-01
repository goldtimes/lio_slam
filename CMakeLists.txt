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
# find_package(Ceres REQUIRED)
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
  ${Sophus_INCLUDE_DIR}
  # src/ceres_pose_graph
)

include(cmake/packages.cmake)

include_directories(
  include 
)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/src)


set(ctlio_source_file
  src/common/timer.cc
  src/lidar_process/lidar_process.cc
  src/lidar_factor/lidar_factor.cc
  src/lidar_factor/analytical_factor.cc
  src/lidar_factor/pose_parameterization.cc
  src/lio/lio_utils.cc
  src/lio/lidar_odom.cc
  src/state/static_imu_init.cc
)

add_library(ctlio_lib 
  ${ctlio_source_file}
)
  
target_link_libraries(ctlio_lib
    ${third_party_libs}
    ${Sophus_LIBRARIES}
    Sophus
)

add_executable(main_node
  src/app/main_node.cc
)
target_link_libraries(main_node
  ctlio_lib
)

# add_executable(
#   lidar_process_test
#   test/lidar_process_test.cc)

# target_link_libraries(lidar_process_test
#   ctlio_lib
# )

# add_executable(
#   static_imu_init_test
#   test/static_imu_test.cc
#   src/state/static_imu_init.cc
# )

# target_link_libraries(static_imu_init_test
#   ctlio_lib
# )

