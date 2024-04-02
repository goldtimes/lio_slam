/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-02 23:09:45
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-02 23:33:35
 * @FilePath: /lio_ws/src/ieskf_slam/include/lidar/avia_process.hh
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
#include "lidar_process_interface.hh"
namespace avia_ros {
// avia雷达的点定义
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D
    float intensity;
    std::uint32_t offset_time;
    std::uint8_t line;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace avia_ros

// 注册点云格式
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(avia_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::uint8_t, line, line)
)
// clang-format on

namespace IESKF_SLAM {
class AviaProcess : public CommonLidarProcessInterface {
   public:
    AviaProcess() = default;
    ~AviaProcess() = default;
    bool process(const sensor_msgs::PointCloud2& msg, PointCloud& cloud) override {
        pcl::PointCloud<avia_ros::Point> avia_cloud;
        pcl::fromROSMsg(msg, avia_cloud);
        cloud.cloud_ptr->clear();
        // omp加速
        for (const auto& pt : avia_cloud.points) {
            Point p;
            p.x = pt.x;
            p.y = pt.y;
            p.z = pt.z;
            p.intensity = pt.intensity;
            p.offset_time = pt.offset_time;
            p.ring = pt.line;
            cloud.cloud_ptr->push_back(p);
        }
        cloud.time_stamp.fromNsec(msg.header.stamp.toNSec());
        return true;
    }
};
}  // namespace IESKF_SLAM

// clang-format on
