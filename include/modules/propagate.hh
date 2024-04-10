/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-04 12:02:14
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-11 00:03:37
 * @FilePath: /lio_ws/src/ieskf_slam/include/modules/propagate.hh
 * @Description: 前向传播
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
#include "ieskf/ieskf.hh"
#include "math/SO3.hpp"
#include "math/math.hpp"
#include "type/measure_group.hh"

namespace IESKF_SLAM {
// 前向后向传播
class FrontBackPropagate {
   public:
    FrontBackPropagate() = default;
    ~FrontBackPropagate() = default;
    void propagate(MeasureGroup& group, IESKF::Ptr ieskf_ptr);

   public:
    double imu_scale;
    IMU last_imu;

    double last_lidar_end_time;

   private:
    struct IMUPose6d {
        double time;
        Eigen::Vector3d acc;
        Eigen::Vector3d gyro;
        Eigen::Vector3d vel;
        Eigen::Vector3d pos;
        Eigen::Quaterniond rot;
        IMUPose6d(double time_ = 0, Eigen::Vector3d a_ = Eigen::Vector3d::Zero(),
                  Eigen::Vector3d av_ = Eigen::Vector3d::Zero(), Eigen::Vector3d v_ = Eigen::Vector3d::Zero(),
                  Eigen::Vector3d p_ = Eigen::Vector3d::Zero(),
                  Eigen::Quaterniond q_ = Eigen::Quaterniond::Identity()) {
            time = time_;
            acc = a_;
            gyro = av_;
            vel = v_;
            pos = p_;
            rot = q_;
        }
    };

    Eigen::Vector3d acc_vel_last;
    Eigen::Vector3d angular_vel_last;
};
}  // namespace IESKF_SLAM