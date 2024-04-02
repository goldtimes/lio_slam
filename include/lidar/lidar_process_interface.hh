/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-02 23:05:46
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-02 23:08:04
 * @FilePath: /lio_ws/src/ieskf_slam/include/ros/lidar_process_interface.hh
 * @Description: 将ros的sensor_msgs转到自定义pointcloud接口
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */

#pragma once

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include "utils/lidar_utils.hh"

namespace IESKF_SLAM {
class CommonLidarProcessInterface {
   public:
    virtual bool process(const sensor_msgs::PointCloud2& msg, PointCloud& cloud) = 0;
};
}  // namespace IESKF_SLAM