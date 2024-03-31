/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-03-31 22:55:50
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-01 00:23:38
 * @FilePath: /lio_ws/src/ieskf_slam/include/type/pose.hh
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */

#pragma once
#include "timestamped.hh"
#include <Eigen/Dense>

namespace IESKF_SLAM {
    struct Pose {
        TimeStamp time_stamp;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
    };
}