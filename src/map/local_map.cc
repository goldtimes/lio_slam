/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 23:52:21
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-04 00:29:15
 * @FilePath: /lio_ws/src/ieskf_slam/src/map/local_map.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "map/local_map.hh"
#include <pcl/common/transforms.h>
#include "math/math.hpp"

namespace IESKF_SLAM {
LocalMap::LocalMap(const std::string& config_path, const std::string& prefix)
    : ModuleBase(config_path, prefix, "LocalMapManager") {
    local_map_ptr_ = pcl::make_shared<PCLPointCloud>();
    kdtree_ptr_ = pcl::make_shared<KDTree>();
    filter_.setLeafSize(0.5, 0.5, 0.5);
}

void LocalMap::AddScan(PCLPointCloudPtr curr_scan_ptr, const Eigen::Quaterniond& rotation,
                       const Eigen::Vector3d& positon) {
    PCLPointCloud scan;
    pcl::transformPointCloud(*curr_scan_ptr, scan, compositeTransform(rotation, positon).cast<float>());
    *local_map_ptr_ += scan;

    filter_.setInputCloud(local_map_ptr_);
    filter_.filter(*local_map_ptr_);
    kdtree_ptr_->setInputCloud(local_map_ptr_);
}

void LocalMap::reset() {
    local_map_ptr_->clear();
}

};  // namespace IESKF_SLAM