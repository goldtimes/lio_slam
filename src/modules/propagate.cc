/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-04 12:07:24
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-04 12:59:05
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
    group.imus.push_front(last_imu);
    double dt;
    IMU in;
    IESKF::State18d imu_state;
    for (auto iter = group.imus.begin(); iter < (group.imus.end() - 1); ++iter) {
        auto&& head = *iter;
        auto&& tail = *(iter + 1);

        Eigen::Vector3d gyro_ave = 0.5 * (head.gyro + tail.gyro);
        // 这里没有对imu_scale进行设置
        Eigen::Vector3d acc_ave = 0.5 * (head.acc + tail.gyro) * imu_scale;
        double dt = tail.time_stamp.sec() - head.time_stamp.sec();
        in.acc = acc_ave;
        in.gyro = gyro_ave;
        ieskf_ptr->Predict(in, dt);
    }
    // imu的时间戳小于最后一个点云
    dt = group.lidar_end_time - group.imus.back().time_stamp.sec();
    ieskf_ptr->Predict(in, dt);
    last_imu = group.imus.back();
    // 下一帧预测开始的时间戳
    last_imu.time_stamp.fromSec(group.lidar_end_time);
}
}  // namespace IESKF_SLAM