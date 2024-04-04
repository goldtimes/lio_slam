/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 23:14:19
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-04 11:21:24
 * @FilePath: /lio_ws/src/ieskf_slam/include/ieskf/ieskf.hh
 * @Description: 迭代卡尔曼滤波器算法
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */

#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "modules/module_base.hh"
#include "type/imu.hh"

namespace IESKF_SLAM {
class IESKF : public ModuleBase {
   public:
    struct State18d {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        Eigen::Quaterniond rotation;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d bg;
        Eigen::Vector3d ba;
        Eigen::Vector3d gravity;

        State18d() {
            rotation = Eigen::Quaterniond::Identity();
            position = Eigen::Vector3d::Zero();
            velocity = Eigen::Vector3d::Zero();
            bg = Eigen::Vector3d::Zero();
            ba = Eigen::Vector3d::Zero();
            gravity = Eigen::Vector3d::Zero();
        }
    };

   public:
    using Ptr = std::shared_ptr<IESKF>;
    IESKF(const std::string& config_path, const std::string& prefix);
    ~IESKF() = default;

    // 预测
    bool Predict(IMU& imu, double dt);
    // 利用观测更新
    bool Update();
    const State18d GetState() const {
        return state_;
    }
    void SetState(const State18d& state_in) {
        state_ = state_in;
    }

   private:
    // 状态量
    // [r,p,v,bg,ba,g]
    State18d state_;
    // 协方差
    Eigen::Matrix<double, 18, 18> P;
    // 预测方程的系统噪声
    Eigen::Matrix<double, 12, 12> Q;
};
}  // namespace IESKF_SLAM