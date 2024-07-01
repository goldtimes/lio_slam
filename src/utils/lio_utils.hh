#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
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
    Eigen::MatrixXd A(points.size(), 3);
    Eigen::VectorXd b(points.size());
    A.setZero();
    b.setOnes();
    b *= -1.0;
    for (int i = 0; i < points.size(); ++i) {
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
        if (std::fabs(plane_coeffs[0] * points[j](0) + plane_coeffs[1] * points[j](1) + plane_coeffs[2] * points[j](2) +
                      plane_coeffs[3]) > thresh) {
            return false;
        }
    }
    return true;
}

inline geometry_msgs::TransformStamped eigen2Tf(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos,
                                                std::string parent_frame_id, std::string child_frame_id,
                                                double timestamped) {
    geometry_msgs::TransformStamped tf;
    tf.header.frame_id = parent_frame_id;
    tf.header.stamp = ros::Time().fromSec(timestamped);
    tf.child_frame_id = child_frame_id;
    Eigen::Quaterniond quad = Eigen::Quaterniond(rot);

    tf.transform.rotation.w = quad.w();
    tf.transform.rotation.x = quad.x();
    tf.transform.rotation.y = quad.y();
    tf.transform.rotation.z = quad.z();

    tf.transform.translation.x = pos(0);
    tf.transform.translation.y = pos(1);
    tf.transform.translation.z = pos(2);
    return tf;
}

inline nav_msgs::Odometry eigen2odom(const Eigen::Matrix3d& rot, const Eigen::Vector3d& pos,
                                     std::string parent_frame_id, std::string child_frame_id, double timestamped) {
    nav_msgs::Odometry odom;
    odom.header.frame_id = parent_frame_id;
    odom.header.stamp = ros::Time().fromSec(timestamped);
    odom.child_frame_id = child_frame_id;
    Eigen::Quaterniond quad = Eigen::Quaterniond(rot);

    odom.pose.pose.orientation.w = quad.w();
    odom.pose.pose.orientation.x = quad.x();
    odom.pose.pose.orientation.y = quad.y();
    odom.pose.pose.orientation.z = quad.z();

    odom.pose.pose.position.x = pos(0);
    odom.pose.pose.position.y = pos(1);
    odom.pose.pose.position.z = pos(2);
    return odom;
}
inline sensor_msgs::PointCloud2 pcl2msg(const sensors::PointNormalCloud::Ptr cloud_pcl, std::string frame_id,
                                        double timestamp) {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_pcl, msg);
    if (timestamp < 0) {
        msg.header.stamp = ros::Time::now();
    } else {
        msg.header.stamp = ros::Time().fromSec(timestamp);
    }
    msg.header.frame_id = frame_id;
    return msg;
}
inline Eigen::Vector3d rotate2rpy(const Eigen::Matrix3d& rotate) {
    double roll = std::atan2(rotate(2, 1), rotate(2, 2));
    double pitch = std::asin(-rotate(2, 0));
    double yaw = std::atan2(rotate(1, 0), rotate(0, 0));
    return Eigen::Vector3d(roll, pitch, yaw);
}
}  // namespace lio