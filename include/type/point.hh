/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-03-31 22:52:37
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-01 00:18:21
 * @FilePath: /lio_ws/src/ieskf_slam/include/type/point.hh
 * @Description: 点云的格式定义
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */

#pragma once

#ifndef PCL_NO_PRECOMPILE
#define PCL_NO_PRECOMPILE
#endif

#include <pcl/impl/pcl_base.hpp>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace IESKF_SLAM{
    struct EIGEN_ALIGN16 Point{
        PCL_ADD_POINT4D;
        float intensity;
        std::uint32_t offset_time;
        std::int32_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
    IESKF_SLAM::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, offset_time, offset_time)
    (std::int32_t, ring, ring)
);


