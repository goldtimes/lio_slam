/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-04 12:07:24
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-11 00:37:26
 * @FilePath: /lio_ws/src/ieskf_slam/src/modules/propagate.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "modules/propagate.hh"

namespace IESKF_SLAM {

/**
 * @brief 前向传播
 */
void FrontBackPropagate::propagate(MeasureGroup& group, IESKF::Ptr ieskf_ptr) {
    auto& cloud = group.cloud;
    std::sort(cloud.cloud_ptr->begin(), cloud.cloud_ptr->end(),
              [](const Point& point_1, const Point& point_2) { return point_1.offset_time < point_2.offset_time; });

    auto imus_temp = group.imus;
    imus_temp.push_front(last_imu);
    std::vector<IMUPose6d> IMUPoses;
    IMUPoses.clear();
    const double& imu_begin_time = imus_temp.front().time_stamp.sec();
    const double& imu_end_time = imus_temp.back().time_stamp.sec();
    const double& lidar_begin_time = group.lidar_begin_time;
    const double& lidar_end_time = group.lidar_end_time;
    auto& pcl_out = *group.cloud.cloud_ptr;
    IESKF::State18d imu_state = ieskf_ptr->GetState();
    IMUPoses.emplace_back(0.0, acc_vel_last, angular_vel_last, imu_state.velocity, imu_state.position,
                          imu_state.rotation);
    Eigen::Vector3d angular_avr, acc_avr, acc_imu, vel_imu, pos_imu;
    Eigen::Matrix3d R_imu;
    double dt = 0;
    IMU in;
    for (auto iter = imus_temp.begin(); iter < (imus_temp.end() - 1); ++iter) {
        auto&& head = *iter;
        auto&& tail = *(iter + 1);
        // 找到最尾部的imu数据
        if (tail.time_stamp.sec() < last_lidar_end_time) {
            continue;
        }
        angular_avr = 0.5 * (head.gyro + tail.gyro);
        acc_avr = 0.5 * (head.acc + tail.acc) * imu_scale;
        // imu.head -------last_lidar_end_time------imu.tail
        if (head.time_stamp.sec() < last_lidar_end_time) {
            dt = tail.time_stamp.sec() - last_lidar_end_time;
        } else {
            dt = tail.time_stamp.sec() - head.time_stamp.sec();
        }
        in.acc = acc_avr;
        in.gyro = angular_avr;
        ieskf_ptr->Predict(in, dt);
        angular_vel_last = angular_avr - imu_state.bg;
        acc_vel_last = imu_state.rotation * (acc_avr - imu_state.ba);
        for (int i = 0; i < 3; ++i) {
            acc_vel_last[i] += imu_state.gravity[i];
        }
        double&& offset_t = tail.time_stamp.sec() - lidar_begin_time;
        IMUPoses.emplace_back(offset_t, acc_vel_last, angular_vel_last, imu_state.velocity, imu_state.position,
                              imu_state.rotation);
    }
    dt = lidar_end_time - imu_end_time;
    // 对一帧雷达之间的Imu都进行了传播
    ieskf_ptr->Predict(in, dt);
    // 获得最后时刻的imu状态
    imu_state = ieskf_ptr->GetState();
    last_imu = group.imus.back();
    last_lidar_end_time = lidar_end_time;
    // fastlio后向传播逐个点去畸变
    if (pcl_out.points.begin() == pcl_out.points.end()) {
        return;
    }
    auto it_pcl = pcl_out.end() - 1;
    for (auto imu_iter = IMUPoses.end() - 1; imu_iter != IMUPoses.begin(); --imu_iter) {
        auto head = imu_iter - 1;
        auto tail = imu_iter;
        R_imu = head->rot.toRotationMatrix();
        vel_imu = head->vel;
        pos_imu = head->pos;
        acc_imu = tail->acc;
        angular_avr = tail->gyro;
        for (; it_pcl->offset_time / 1e9 > head->time; it_pcl--) {
            dt = it_pcl->offset_time / 1e9 - head->time;
            Eigen::Matrix3d R_i(R_imu * so3Exp(angular_avr * dt));
            Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
            Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.position);
            Eigen::Vector3d P_compensate = imu_state.rotation.conjugate() * (R_i * P_i + T_ei);
            it_pcl->x = P_compensate(0);
            it_pcl->y = P_compensate(1);
            it_pcl->z = P_compensate(2);
            if (it_pcl == pcl_out.points.begin()) {
                break;
            }
        }
    }
    return;
}
}  // namespace IESKF_SLAM