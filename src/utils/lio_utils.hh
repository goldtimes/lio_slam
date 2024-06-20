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
    bool lidar_pushed;
    double lidar_begin_time;
    double lidar_end_time;
    std::deque<sensors::IMU> imudatas;
    sensors::PointNormalCloud::Ptr lidar_cloud;
};

enum LIO_STATUS { INITIALIZE, RELOCALIZATION, LOCALIZATION, MAPPING };

inline bool esti_plane(Eigen::Vector4d& plane_coeffs, const std::vector<Eigen::Vector3d>& points, const double& thresh,
                       bool none) {
    Eigen::Matrix<double, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<double, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0;
    for (int i = 0; i < NUM_MATCH_POINTS; ++i) {
        A(i, 0) = points[i](0);
        A(i, 1) = points[i](1);
        A(i, 2) = points[i](2);
    }
    Eigen::Vector3d norm_vec = A.colPivHouseholderQr().solve(b);
    double norm = norm_vec.norm();
    plane_coeffs[0] = norm_vec(0) / norm;
    plane_coeffs[1] = norm_vec(1) / norm;
    plane_coeffs[2] = norm_vec(2) / norm;
    plane_coeffs[3] = 1.0 / norm;
    // check 点到平面的距离 < thresh
    for (int j = 0; j < points.size(); ++j) {
        if (std::fabs(plane_coeffs[0] * points[j](0) + plane_coeffs[1] * points[j](1) +
                      plane_coeffs[2] * points[j](2)) > thresh) {
            return false;
        }
    }
    return true;
}

}  // namespace lio