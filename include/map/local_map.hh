/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 23:38:44
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-04 00:23:02
 * @FilePath: /lio_ws/src/ieskf_slam/include/map/local_map.hh
 * @Description: 局部子图的构建，ikdtree or 传统的submap
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */

#pragma once
#include <Eigen/Core>
#include <string>
#include "modules/module_base.hh"
#include "type/pointcloud.hh"
#include "utils/lidar_utils.hh"

namespace IESKF_SLAM {
class LocalMap : public ModuleBase {
   public:
    LocalMap(const std::string& config_path, const std::string& prefix);
    ~LocalMap() = default;

    void reset();
    void AddScan(PCLPointCloudPtr curr_scan_ptr, const Eigen::Quaterniond& rotation, const Eigen::Vector3d& positon);
    const PCLPointCloudPtr GetLocalMap() const {
        return local_map_ptr_;
    }

    KDTreeConstPtr GetKDTree() {
        return kdtree_ptr_;
    }

   private:
    PCLPointCloudPtr local_map_ptr_;
    KDTreePtr kdtree_ptr_;
    VoxelFilter filter_;
    float map_side_length_2_;
    float map_resolution_;
};
}  // namespace IESKF_SLAM