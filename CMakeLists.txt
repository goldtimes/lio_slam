cmake_minimum_required(VERSION 3.0.2)
project(lio_slam)

SET(CMAKE_CXX_FLAGS "-std=c++17 -O3")
add_compile_options(-std=c++17)

message("=========${PROJECT_SOURCE_DIR}=============")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)


add_definitions(-DPROJECT_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  livox_ros_driver
  message_generation
  std_msgs
)
set(CMAKE_BUILD_TYPE Release)

find_package(PCL REQUIRED)
# YAML CPP
find_package(PkgConfig REQUIRED)
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED)
find_package(Sophus REQUIRED)
find_package(GTSAM REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

add_service_files(
  FILES 
  SaveMap.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


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
  ${GLOG_INCLOUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
  src
  # sophus
  # ${PROJECT_SOURCE_DIR}/thirdparty/sophus
)




list(APPEND lio_source_files 
  src/voxel_map/voxel_map.cc
  src/mapping/map_builder_ros.cc
  src/lio/imu_process.cc
  src/lio/iglio_builder.cc
  src/ieskf_fastlio/ieskf_fastlio.cc
  src/loopclosure/loopclosure.cc
)

add_library(lio_lib SHARED ${lio_source_files})
target_link_libraries(lio_lib
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  glog
  gflags
  gtsam
)


add_executable(map_builder_node  
  src/map_builder_node.cc
)

target_link_libraries(map_builder_node
  lio_lib
)