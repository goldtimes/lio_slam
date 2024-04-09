/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 22:18:25
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-06 23:21:55
 * @FilePath: /lio_ws/src/ieskf_slam/src/modules/frontend/frontend.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "modules/frontend/frontend.hh"
#include <pcl/common/transforms.h>
#include <ros/ros.h>

namespace IESKF_SLAM {
Frontend::Frontend(const std::string& config_file_path, const std::string& prefix)
    : ModuleBase(config_file_path, prefix, "Frontend Modules") {
    // 读取雷达和imu外参
    float leaf_size;
    readParam("filter_leaf_size", leaf_size, 0.5f);
    voxel_filter_.setLeafSize(leaf_size, leaf_size, leaf_size);
    std::vector<double> extrinsic_r_params;
    std::vector<double> extrinsic_t_params;
    readParam("extrinsic_r", extrinsic_r_params, std::vector<double>());
    extrinsic_r.setIdentity();
    extrinsic_t.setZero();
    if (extrinsic_r_params.size() == 9) {
        Eigen::Matrix3d r;
        r << extrinsic_r_params[0], extrinsic_r_params[1], extrinsic_r_params[2], extrinsic_r_params[3],
            extrinsic_r_params[4], extrinsic_r_params[5], extrinsic_r_params[6], extrinsic_r_params[7],
            extrinsic_r_params[8];
        extrinsic_r = r;
    } else if (extrinsic_r_params.size() == 4) {
        extrinsic_r.x() = extrinsic_r_params[0];
        extrinsic_r.y() = extrinsic_r_params[1];
        extrinsic_r.z() = extrinsic_r_params[2];
        extrinsic_r.w() = extrinsic_r_params[3];
    }

    readParam("extrinsic_t", extrinsic_t_params, std::vector<double>());
    if (extrinsic_r_params.size() == 3) {
        extrinsic_t << extrinsic_t_params[0], extrinsic_t_params[1], extrinsic_t_params[2];
    }

    ieskf_ptr_ = std::make_shared<IESKF>(config_file_path, "ieskf");
    map_ptr_ = std::make_shared<LocalMap>(config_file_path, "local_map");
    propagate_ptr_ = std::make_shared<FrontBackPropagate>();
    lio_zh_model_ptr_ = std::make_shared<LIOZHModel>();
    // to do将 lio的caculate函数给到ieskf
    lio_zh_model_ptr_->prepare(map_ptr_->GetKDTree(), filter_point_cloud_ptr, map_ptr_->GetLocalMap());
}

void Frontend::AddIMU(const IMU& imu) {
    imu_queue_.push_back(imu);
}

void Frontend::AddCloud(const PointCloud& cloud) {
    clouds_queue_.push_back(cloud);
}

void Frontend::AddPose(const Pose& pose) {
    pose_queue_.push_back(pose);
}

/**
 * @brief 时间同步
 */
bool Frontend::SyncMeasureGroup(MeasureGroup& group) {
    group.imus.clear();
    group.cloud.cloud_ptr->clear();
    // 如果传感器的队列为空
    if (imu_queue_.empty() || clouds_queue_.empty()) {
        return false;
    }
    // 等待imu
    double imu_end_time = imu_queue_.back().time_stamp.sec();
    double imu_start_time = imu_queue_.front().time_stamp.sec();
    // 一帧雷达开始时间
    double cloud_start_time = clouds_queue_.front().time_stamp.sec();
    // 一帧雷达扫描的结束时间
    double cloud_end_time = clouds_queue_.front().cloud_ptr->points.back().offset_time / 1e9 + cloud_start_time;
    // ROS_INFO("imu end time:{%f}, imu_start_time:{%f}, cloud_start_time:{%f}, cloud_end_time:{%f}", imu_end_time,
    //          imu_start_time, cloud_start_time, cloud_end_time);
    // 如果imu队列中最晚的时间戳小于雷达点云扫描的结束时间，则无法同步
    if (imu_end_time < cloud_end_time) {
        // ROS_ERROR("imu_end_time < cloud_end_time");
        return false;
    }
    if (cloud_end_time < imu_start_time) {
        clouds_queue_.pop_front();
        // ROS_ERROR("cloud_end_time < imu_start_time");

        return false;
    }
    // 取出雷达数据
    group.cloud = clouds_queue_.front();
    clouds_queue_.pop_front();
    group.lidar_begin_time = cloud_start_time;
    group.lidar_end_time = cloud_end_time;
    // imu队列非空情况,将一帧雷达之间的imu存起来
    while (!imu_queue_.empty()) {
        if (imu_queue_.front().time_stamp.sec() < group.lidar_end_time) {
            group.imus.push_back(imu_queue_.front());
            imu_queue_.pop_front();
        } else {
            break;
        }
    }
    if (group.imus.size() <= 5) {
        return false;
    }
    return true;
}

/**
 * @brief 只估计了重力的方向和bg，感觉不太严谨
 */
bool Frontend::InitState(MeasureGroup& group) {
    static int imu_count = 0;
    static Eigen::Vector3d mean_acc{0, 0, 0};
    auto& ieskf = *ieskf_ptr_;
    if (imu_inited_) {
        return true;
    }
    for (size_t i = 0; i < group.imus.size(); ++i) {
        imu_count++;
        IESKF::State18d state = ieskf.GetState();
        mean_acc += group.imus[i].acc;
        state.bg += group.imus[i].gyro;
        ieskf.SetState(state);
    }
    if (imu_count >= 5) {
        IESKF::State18d state = ieskf.GetState();
        mean_acc /= (double)imu_count;
        state.bg /= (double)imu_count;
        imu_scale_ = GRAVITY / mean_acc.norm();
        // std::cout << "mea_acc norm: " << mean_acc.norm() << std::endl;
        propagate_ptr_->imu_scale = imu_scale_;
        propagate_ptr_->last_imu = group.imus.back();
        state.gravity = -mean_acc / mean_acc.norm() * GRAVITY;
        ieskf.SetState(state);
        imu_inited_ = true;
    }
    return true;
}

/**
 * @brief 根据pose找到对应时刻的点云,纯展示的过程
 */
bool Frontend::track() {
    MeasureGroup group;
    // 时间同步
    if (SyncMeasureGroup(group)) {
        if (!imu_inited_) {
            map_ptr_->reset();
            map_ptr_->AddScan(group.cloud.cloud_ptr, Eigen::Quaterniond::Identity(), Eigen::Vector3d::Zero());
            InitState(group);
            return false;
        }
        // imu 初始化成功
        // imu向前传播
        propagate_ptr_->propagate(group, ieskf_ptr_);
        voxel_filter_.setInputCloud(group.cloud.cloud_ptr);
        voxel_filter_.filter(*filter_point_cloud_ptr);
        // 点云的紧耦合更新
        ieskf_ptr_->Update();
        IESKF::State18d x = ieskf_ptr_->GetState();
        // 更新submap
        map_ptr_->AddScan(filter_point_cloud_ptr, x.rotation, x.position);
        return true;
    }
    // ROS_ERROR("sync error");
    return false;
    // 都为空
    // if (pose_queue_.empty() || clouds_queue_.empty()) {
    //     return false;
    // }
    // // 将早于cloud_queue_.front()时刻的pose删除
    // while (!pose_queue_.empty() && pose_queue_.front().time_stamp.nsec() < clouds_queue_.front().time_stamp.nsec()) {
    //     ROS_INFO("delete pose time:{%d}", pose_queue_.front().time_stamp.nsec());
    //     pose_queue_.pop_front();
    // }

    // if (pose_queue_.empty()) {
    //     return false;
    // }

    // while (!clouds_queue_.empty() && clouds_queue_.front().time_stamp.nsec() < pose_queue_.front().time_stamp.nsec())
    // {
    //     clouds_queue_.pop_front();
    // }
    // if (clouds_queue_.empty()) {
    //     return false;
    // }
    // // 滤波
    // VoxelFilter filter;
    // filter.setLeafSize(0.5, 0.5, 0.5);
    // auto cloud = clouds_queue_.front().cloud_ptr;
    // filter.setInputCloud(cloud);
    // filter.filter(*cloud);

    // // 将odom变化转矩阵
    // Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    // tf.block<3, 3>(0, 0) = pose_queue_.front().rotation.toRotationMatrix().cast<float>();
    // tf.block<3, 1>(0, 3) = pose_queue_.front().position.cast<float>();
    // pcl::transformPointCloud(*cloud, current_pointcloud_, tf);

    // clouds_queue_.pop_front();
    // pose_queue_.pop_front();
    // return true;
}
}  // namespace IESKF_SLAM
