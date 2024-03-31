/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-03-31 22:55:45
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-01 00:20:12
 * @FilePath: /lio_ws/src/ieskf_slam/include/type/pointcloud.hh
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */

#pragma once
#include "point.hh"
#include "timestamped.hh"
#include <memory>

namespace IESKF_SLAM{
    using PCLPointCloud = pcl::PointCloud<Point>;
    using PCLPointCloudPtr = pcl::PointCloud<Point>::Ptr;

    struct PointCloud{
        using Ptr = std::shared_ptr<PointCloud>;
        TimeStamp time_stamp;
        PCLPointCloudPtr cloud_ptr;
        PointCloud(){
            cloud_ptr.reset(new PCLPointCloud());
        } 
    };
}
