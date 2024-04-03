/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 22:18:25
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-04 01:22:32
 * @FilePath: /lio_ws/src/ieskf_slam/src/frontend/frontend.cc
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
        ROS_ERROR("imu or cloud queue is empty");
        return false;
    }
    // 等待imu
    double imu_end_time = imu_queue_.back().time_stamp.sec();
    double imu_start_time = imu_queue_.front().time_stamp.sec();
    // 一帧雷达开始时间
    double cloud_start_time = clouds_queue_.front().time_stamp.sec();
    // 一帧雷达扫描的结束时间
    double cloud_end_time = clouds_queue_.front().cloud_ptr->points.back().offset_time / 1e9 + cloud_start_time;
    // 如果imu队列中最晚的时间戳小于雷达点云扫描的结束时间，则无法同步
    if (imu_end_time < cloud_end_time) {
        return false;
    }
    if (cloud_end_time < imu_start_time) {
        clouds_queue_.pop_front();
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
