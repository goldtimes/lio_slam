/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 23:52:03
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-13 13:36:31
 * @FilePath: /lio_ws/src/ieskf_slam/src/ieskf/ieskf.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "ieskf/ieskf.hh"
#include "math/SO3.hpp"

namespace IESKF_SLAM {
IESKF::IESKF(const std::string& config_path, const std::string& prefix) : ModuleBase(config_path, prefix) {
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

// lidar观测的更新
bool IESKF::Update() {
    static int cnt = 0;
    auto state_temp = state_;
    auto state_last = state_;
    // 开始迭代ESKF
    Eigen::MatrixXd K;
    Eigen::MatrixXd H_K;
    Eigen::Matrix<double, 18, 18> P_in_update;
    bool converge = true;
    for (int iter = 0; iter < iter_times; ++iter) {
        // 计算J
        // 获取误差状态
        Eigen::Matrix<double, 18, 1> error_state = GetErrorState(state_temp, state_);
        // Jacobian矩阵 18x18
        Eigen::Matrix<double, 18, 18> J_inv;
        J_inv.setIdentity();
        J_inv.block<3, 3>(0, 0) = A_T(error_state.block<3, 1>(0, 0));
        // 更新P, 第一次更新这里几乎为0
        P_in_update = J_inv * P * J_inv.transpose();
        Eigen::MatrixXd z_k;
        Eigen::MatrixXd R_inv;
        // 调用接口计算Z，H
        caculate_z_h(state_temp, z_k, H_K);
        Eigen::MatrixXd H_K_T = H_K.transpose();
        // R噪声写死
        K = (H_K_T * H_K + (P_in_update / 0.001).inverse()).inverse() * H_K_T;
        // 计算增量
        Eigen::MatrixXd left = -1 * K * z_k;
        Eigen::MatrixXd right = -1 * (Eigen::Matrix<double, 18, 18>::Identity() - K * H_K) * J_inv * error_state;
        Eigen::MatrixXd update_x = left + right;
        // 收敛判断
        converge = true;
        for (int i = 0; i < 18; ++i) {
            // 有更新量，退出判断继续迭代
            if (update_x(i, 0) > 0.001) {
                converge = false;
                break;
            }
        }
        // 更新state_temp;
        state_temp.rotation = state_temp.rotation.toRotationMatrix() * so3Exp(update_x.block<3, 1>(0, 0));
        state_temp.rotation.normalize();  //四元素归一化
        state_temp.position = state_temp.position + update_x.block<3, 1>(3, 0);
        state_temp.velocity = state_temp.velocity + update_x.block<3, 1>(6, 0);
        state_temp.bg = state_temp.bg + update_x.block<3, 1>(9, 0);
        state_temp.ba = state_temp.ba + update_x.block<3, 1>(12, 0);
        state_temp.gravity = state_temp.gravity + update_x.block<3, 1>(15, 0);
        if (converge) {
            break;
        }
    }
    cnt++;
    // 更新state
    state_ = state_temp;
    // 更新P
    P = (Eigen::Matrix<double, 18, 18>::Identity() - K * H_K) * P_in_update;
    return converge;
}

Eigen::Matrix<double, 18, 1> IESKF::GetErrorState(const State18d& s1, const State18d& s2) {
    Eigen::Matrix<double, 18, 1> error_state = Eigen::Matrix<double, 18, 1>::Zero();
    error_state.block<3, 1>(0, 0) = SO3Log(s2.rotation.toRotationMatrix().transpose() * s1.rotation.toRotationMatrix());
    error_state.block<3, 1>(3, 0) = s1.position - s2.position;
    error_state.block<3, 1>(6, 0) = s1.velocity - s2.velocity;
    error_state.block<3, 1>(9, 0) = s1.bg - s2.bg;
    error_state.block<3, 1>(12, 0) = s1.ba - s2.ba;
    error_state.block<3, 1>(15, 0) = s1.gravity - s2.gravity;
    return error_state;
}

}  // namespace IESKF_SLAM