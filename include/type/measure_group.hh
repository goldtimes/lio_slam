/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 23:08:28
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-03 23:10:37
 * @FilePath: /lio_ws/src/ieskf_slam/include/type/measure_group.hh
 * @Description: 将时间同步的传感器信息放到结构体中
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
#include <deque>
#include "imu.hh"
#include "pointcloud.hh"

namespace IESKF_SLAM {
struct MeasureGroup {
    double lidar_begin_time;
    double lidar_end_time;
    std::deque<IMU> imus;
    PointCloud cloud;
};
}  // namespace IESKF_SLAM