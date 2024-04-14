/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 23:52:21
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-13 15:12:56
 * @FilePath: /lio_ws/src/ieskf_slam/src/map/local_map.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "map/local_map.hh"
#include <pcl/common/transforms.h>
#include "math/math.hpp"

namespace IESKF_SLAM {
LocalMap::LocalMap(const std::string& config_path, const std::string& prefix) : ModuleBase(config_path, prefix) {
    local_map_ptr_ = pcl::make_shared<PCLPointCloud>();
    kdtree_ptr_ = pcl::make_shared<KDTree>();
    filter_.setLeafSize(0.5, 0.5, 0.5);
    readParam<float>("map_side_length_2", map_side_length_2_, 500);
    readParam<float>("map_resolution", map_resolution_, 0.5);
}

void LocalMap::AddScan(PCLPointCloudPtr curr_scan_ptr, const Eigen::Quaterniond& rotation,
                       const Eigen::Vector3d& positon) {
    PCLPointCloud scan;
    pcl::transformPointCloud(*curr_scan_ptr, scan, compositeTransform(rotation, positon).cast<float>());
    // 这里很粗暴的构建kdtree,在大的场景会比较影响性能

    // *local_map_ptr_ += scan;
    // filter_.setInputCloud(local_map_ptr_);
    // filter_.filter(*local_map_ptr_);
    // kdtree_ptr_->setInputCloud(local_map_ptr_);
    if (local_map_ptr_->empty()) {
        *local_map_ptr_ = scan;
    } else {
        // 规定一个正方形大小,落在正方形大小的点来构建kdtree,否则删除
        // kdtree搜索>0.5m的点放到kdtree中
        for (const auto& point : scan.points) {
            std::vector<int> indexs;
            std::vector<float> dists;
            kdtree_ptr_->nearestKSearch(point, 5, indexs, dists);
            if (dists[0] > map_resolution_) {
                local_map_ptr_->push_back(point);
            }
        }
        // 删除超过指定范围的点
        int left = 0;
        int right = local_map_ptr_->size() - 1;
        while (left < right) {
            // 用local_map最后的点-当前的位置,但是最后的点一定是最大的点么?
            while (left < right && (std::fabs(local_map_ptr_->points[right].x - positon.x()) > map_side_length_2_) &&
                   (std::fabs(local_map_ptr_->points[right].y - positon.y()) > map_side_length_2_) &&
                   (std::fabs(local_map_ptr_->points[right].z - positon.z()) > map_side_length_2_)) {
                right--;
            }
            while (left < right && (std::fabs(local_map_ptr_->points[left].x - positon.x()) < map_side_length_2_) &&
                   (std::fabs(local_map_ptr_->points[left].y - positon.y()) < map_side_length_2_) &&
                   (std::fabs(local_map_ptr_->points[left].z - positon.z()) < map_side_length_2_)) {
                left++;
            }
            // 超过map_side_length_2_ 的点移动到后面
            std::swap(local_map_ptr_->points[left], local_map_ptr_->points[right]);
        }
        // 删除后面的点
        local_map_ptr_->resize(right + 1);
    }
    kdtree_ptr_->setInputCloud(local_map_ptr_);
}

void LocalMap::reset() {
    local_map_ptr_->clear();
}

};  // namespace IESKF_SLAM