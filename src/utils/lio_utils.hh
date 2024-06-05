#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <deque>
#include "sensors/imu.hh"
#include "sensors/point_types.hh"

namespace lio {

#define NUM_MATCH_POINTS (5)
#define NUM_MAX_POINTS (10000)

struct LivoxData {
    std::deque<sensors::PointNormalCloud::Ptr> clouds_buff;
    std::deque<double> time_buffer;
    double blind = 0.5;
    int filter_num = 3;
};

struct MeasureGroup {
    double lidar_begin_time;
    double lidar_end_time;
    std::deque<sensors::IMU> imudatas;
    sensors::CloudPtr lidar_cloud;
};

}  // namespace lio