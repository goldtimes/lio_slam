/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 23:52:03
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-04 11:54:41
 * @FilePath: /lio_ws/src/ieskf_slam/src/ieskf/ieskf.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "ieskf/ieskf.hh"
#include "math/SO3.hpp"

namespace IESKF_SLAM {
IESKF::IESKF(const std::string& config_path, const std::string& prefix) : ModuleBase(config_path, prefix, "IESKF") {
    // 初始化协方差
    P.setIdentity();
    P(9, 9) = P(10, 10) = P(11, 11) = 0.0001;
    P(12, 12) = P(13, 13) = P(14, 14) = 0.001;
    P(15, 15) = P(16, 16) = P(17, 17) = 0.00001;
    // 初始化系统噪声Q
    double cov_gyro, cov_acc, cov_bias_acc, cov_bias_gyro;
    readParam("cov_gyro", cov_gyro, 0.1);
    readParam("cov_acc", cov_acc, 0.1);
    readParam("cov_bias_gyro", cov_bias_gyro, 0.1);
    readParam("cov_bias_acc", cov_bias_acc, 0.1);
    Q.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d{cov_gyro, cov_gyro, cov_gyro};
    Q.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d{cov_acc, cov_acc, cov_acc};
    Q.block<3, 3>(6, 6).diagonal() = Eigen::Vector3d{cov_bias_gyro, cov_bias_gyro, cov_bias_gyro};
    Q.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d{cov_bias_acc, cov_bias_acc, cov_bias_acc};
    state_.rotation.setIdentity();
    state_.position.setZero();
    state_.velocity.setZero();
    state_.bg.setZero();
    state_.ba.setZero();
    state_.gravity.setZero();
}

bool IESKF::Predict(IMU& imu, double dt) {
    // fastlio2 状态递推，这里的推导和高博书里面的不太一样，需要查看fastlio的论文
    // 误差状态的传播 x_误差 =  x_真实值 - x_估计值 计算Fx,Fw
    imu.acc -= state_.ba;
    imu.gyro -= state_.bg;
    auto rotation = state_.rotation.toRotationMatrix();
    state_.rotation = Eigen::Quaterniond(rotation * so3Exp(imu.gyro * dt));
    // 四元素归一化
    state_.rotation.normalize();
    state_.position += state_.velocity * dt;
    // 从这里看出，状态方程表示的是世界坐标系下的状态
    // (将imu读到的acc转换到世界坐标系+gravity) * dt = 世界坐标系下的所读
    state_.velocity += (rotation * imu.acc + state_.gravity) * dt;
    // -------------------------------
    // Fx 和 Fw的计算
    Eigen::Matrix<double, 18, 18> Fx;
    Eigen::Matrix<double, 18, 12> Fw;

    Fw.setZero();
    Fx.setIdentity();
    Fx.block<3, 3>(0, 0) = so3Exp(-imu.gyro * dt);
    Fx.block<3, 3>(0, 9) = -A_T(imu.gyro * dt) * dt;
    Fx.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    Fx.block<3, 3>(6, 0) = rotation * skew(imu.acc) * dt * -1;
    Fx.block<3, 3>(6, 12) = rotation * dt * -1;
    Fx.block<3, 3>(6, 15) = Eigen::Matrix3d::Identity() * dt;

    Fw.block<3, 3>(0, 0) = -1 * A_T(-imu.gyro * dt) * dt;
    Fw.block<3, 3>(6, 3) = rotation * dt;
    Fw.block<3, 3>(9, 6) = Fw.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;

    P = Fx * P * Fx.transpose() + Fw * Q * Fw.transpose();
    return true;
}

bool IESKF::Update() {
    return true;
}

}  // namespace IESKF_SLAM