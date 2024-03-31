/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-01 00:24:30
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-01 00:28:06
 * @FilePath: /lio_ws/src/ieskf_slam/include/utils/lidar_utils.hh
 * @Description: pcl的一些工具类
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */

#pragma once

#include "type/pointcloud.hh"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace IESKF_SLAM {
    using VoxelFilter = pcl::VoxelGrid<Point>;
    using KDTree = pcl::KdTreeFLANN<Point>;
    using KDTreePtr = KDTree::Ptr;
    // 重力常量
    const double GRAVITY = 9.81;
}
