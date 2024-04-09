/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 22:18:25
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-06 22:39:19
 * @FilePath: /lio_ws/src/ieskf_slam/include/modules/frontend/frontend.hh
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
#include <deque>
#include "ieskf/ieskf.hh"
#include "map/local_map.hh"
#include "modules/frontend/lio_zh_model.hh"
#include "modules/module_base.hh"
#include "modules/propagate.hh"
#include "type/imu.hh"
#include "type/measure_group.hh"
#include "type/pointcloud.hh"
#include "type/pose.hh"
#include "utils/lidar_utils.hh"

namespace IESKF_SLAM {
class Frontend : public ModuleBase {
   public:
    using Ptr = std::shared_ptr<Frontend>;

   public:
    Frontend(const std::string& config_file_path, const std::string& prefix);
    ~Frontend() = default;

    // 传入数据给前端
    void AddIMU(const IMU& imu_data);
    void AddCloud(const PointCloud& cloud);
    void AddPose(const Pose& pose);

    // imu 初始化
    bool InitState(MeasureGroup& group);
    // 传感器时间同步
    bool SyncMeasureGroup(MeasureGroup& group);
    bool track();

    const PCLPointCloud& getCurrentCloud() const {
        return *filter_point_cloud_ptr;
    }

    const IESKF::State18d readState() const {
        return ieskf_ptr_->GetState();
    }

   private:
    std::deque<IMU> imu_queue_;
    std::deque<PointCloud> clouds_queue_;
    std::deque<Pose> pose_queue_;
    PCLPointCloud current_pointcloud_;
    PCLPointCloudPtr filter_point_cloud_ptr;
    std::shared_ptr<IESKF> ieskf_ptr_;
    std::shared_ptr<LocalMap> map_ptr_;
    std::shared_ptr<FrontBackPropagate> propagate_ptr_;
    std::shared_ptr<LIOZHModel> lio_zh_model_ptr_;
    VoxelFilter voxel_filter_;

    // 外参矩阵
    Eigen::Matrix3d extrinsic_r;
    Eigen::Vector3d extrinsic_t;

    // imu inited
    bool imu_inited_;
    double imu_scale_;
};
}  // namespace IESKF_SLAM