/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-16 23:10:10
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-17 23:54:56
 * @FilePath: /lio_ws/src/ieskf_slam/src/tools/lidar_utils.hpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once

#include <pcl/filters/voxel_grid.h>
#include "sensors/point_types.hh"

namespace ctlio::slam {

FullCloudPtr ConvertVecs3DtoFullCloud(const std::vector<point3D>& input) {
    FullCloudPtr cloud(new FullPointCloudType());
    for (const auto& pt : input) {
        FullPointType p;
        p.x = pt.raw_point[0];
        p.y = pt.raw_point[1];
        p.z = pt.raw_point[2];
        p.intensity = pt.intensity;
        cloud->points.emplace_back(p);
    }
    cloud->width = input.size();
    return cloud;
}

/**
 * @brief 将全量点云转换成xyzi点云
 */
template <typename PointT = PointType>
CloudPtr ConvertToCloud(typename pcl::PointCloud<PointT>::Ptr input) {
    CloudPtr cloud(new PointCloudType());
    for (auto& pt : input->points) {
        PointType p;
        p.x = pt.x;
        p.y = pt.y;
        p.z = pt.z;
        p.intensity = pt.intensity;
        cloud->points.emplace_back(p);
    }
    cloud->width = input->width;
    return cloud;
}

/**
 * @brief 滤波
 */
inline CloudPtr FilteCloud(CloudPtr cloud, float voxel_size = 0.1) {
    pcl::VoxelGrid<PointType> voxel_grid_;
    voxel_grid_.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid_.setInputCloud(cloud);

    CloudPtr output(new PointCloudType);
    voxel_grid_.filter(*output);
    return output;
}
}  // namespace ctlio::slam