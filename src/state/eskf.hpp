#pragma once

#include <glog/logging.h>
#include <iomanip>
#include "common/eigen_types.hh"
#include "common/math_utils.hh"
#include "navi_state.hh"
#include "sensors/imu.hh"
#include "sensors/odom.hh"

namespace ctlio::slam {
/**
 * @brief 18维的eskf
 *  p,v,R,bg,ba,grav
 */
template <typename S>
class ESKF {
   public:
    using MotionNoiseT = Eigen::Matrix<S, 18, 18>;
    using OdomNoiseT = Eigen::Matrix<S, 3, 3>;
    using Matrix18T = Eigen::Matrix<S, 18, 18>;

    struct Options {
        Options() = default;
        // imu 测量与零偏参数
        double imu_dt_ = 0.01;
        double gryo_var_ = 1e-5;  // 陀螺仪测量标准差
        double acc_var_ = 1e-2;   // 加速度计测量标准差
        double bias_gyro_var_ = 1e-6;
        double bias_acce_var_ = 1e-4;
        // odom 参数
        double odom_var_ = 0.5;
        double odom_span_ = 0.1;
        double wheel_radius_ = 0.155;
        double circle_pulse_ = 1024.0;

        bool update_bias_ = true;  // 更新bias
    };

   public:
    ESKF(Options options = Options()) : options_(options) {
        BuildNoise(options);
    }

    void SetInitailConditions(Options options, const Vec3d& init_ba, const Vec3d& init_bg,
                              const Vec3d& gravity = Vec3d(0, 0, -9.81)) {
        BuildNoise(options);
        options_ = options;
        bg_ = init_bg;
        ba_ = init_ba;
        gravity_ = gravity;
        // 协方差
        cov_ = Matrix18T::Identity() * 1e-4;
    }
    // imu递推
    bool Predict(const IMU& imu);
    // 轮速计对速度的观测
    bool ObserveWheelSpeed(const Odom& odom);
    /**
     * @brief 松耦合对位姿的观测
     */
    bool ObserveSE3(const SE3& pose, double trans_noise = 0.1, double ang_noise = 1.0 * math::kDEG2RAD);
    bool ObserveSE3(const SE3& pose, const Vec6d& noise);

    void SetState(const NavState& x, const Vec3d& gravity) {
        current_time_ = x.timestamped_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
        gravity_ = gravity;
    }

    const NavState GetState() const {
        return NavState(current_time_, R_, p_, v_, bg_, ba_);
    }

    const SE3 GetSE3() const {
        return SE3(R_, p_);
    }

    const Vec3d GetV() const {
        return v_;
    }

    void SetCov(const Mat18d& cov) {
        cov_ = cov;
    }

    const Vec3d GetGravity() const {
        return gravity_;
    }

   private:
    void BuildNoise(const Options& options) {
        double noise_sigma_acc = options.acc_var_;
        double noise_sigma_gyro = options.gryo_var_;
        double noise_sigma_ba = options.bias_acce_var_;
        double noise_sigma_bg = options.bias_gyro_var_;

        double sigma_acc_2 = noise_sigma_acc;
        double sigma_gyro_2 = noise_sigma_gyro;
        double sigma_ba_2 = noise_sigma_ba;
        double sigma_bg_2 = noise_sigma_bg;

        Q_.diagonal() << 0, 0, 0, sigma_acc_2, sigma_acc_2, sigma_acc_2, sigma_gyro_2, sigma_gyro_2, sigma_gyro_2,
            sigma_bg_2, sigma_bg_2, sigma_bg_2, sigma_ba_2, sigma_ba_2, sigma_ba_2;

        // odom噪声
        double encoder_noise = options.odom_var_ * options.odom_var_;
        odom_noise_.diagonal() << encoder_noise, encoder_noise, encoder_noise;
    }
    // 更新名义状态量,并且重置dx
    void UpdateAndReset() {
        p_ += dx_.block<3, 1>(0, 0);
        v_ += dx_.block<3, 1>(3, 0);
        R_ = R_ * Sophus::SO3::exp(dx_.block<3, 1>(6, 0));
        if (options_.update_bias_) {
            bg_ += dx_.block<3, 1>(9, 0);
            ba_ += dx_.block<3, 1>(12, 0);
        }
        gravity_ += dx_.block<3, 1>(15, 0);

        ProjectCov();
        dx_.setZero();
    }
    // 对P矩阵投影
    void ProjectCov() {
        Mat18d J = Mat18d::Identity();
        J.block<3, 3>(6, 6) = Mat3d::Identity() - 0.5 * Sophus::SO3::hat(dx_.block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }

   private:
    Options options_;
    MotionNoiseT Q_;  // 过程噪声
    OdomNoiseT odom_noise_;
    double current_time_;

    // 状态量
    SO3 R_;
    Vec3d p_ = Vec3d::Zero();
    Vec3d v_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    Vec3d gravity_{0.0, 0.0, -9.81};
    Matrix18T cov_;
    // 误差状态
    Vec18d dx_ = Vec18d::Zero();
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename S>
bool ESKF<S>::Predict(const IMU& imu) {
    assert(imu.timestamped_ >= current_time_);
    double dt = imu.timestamped_ - current_time_;
    if (dt > 5 * (options_.imu_dt_) || dt < 0) {
        // 时间间隔不对，可能是第一个IMU数据，没有历史信息
        LOG(INFO) << "skip this imu because dt_ = " << dt;
        current_time_ = imu.timestamped_;
        return false;
    }
    // 状态递推
    Vec3d new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acce_ - ba_)) * dt * dt + 0.5 * gravity_ * dt * dt;
    Vec3d new_v = v_ + R_ * (imu.acce_ - ba_) * dt + gravity_ * dt;
    SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

    R_ = new_R;
    p_ = new_p;
    v_ = new_v;
    // 误差状态递推
    Mat18d F = Mat18d::Identity();
    F.block<3, 3>(0, 3) = Mat3d::Identity() * dt;                         // p对v
    F.block<3, 3>(3, 6) = -R_.matrix() * SO3::hat(imu.acce_ - ba_) * dt;  // v 对 theta
    F.block<3, 3>(3, 12) = -R_.matrix() * dt;                             // v对ba
    F.block<3, 3>(3, 15) = Mat3d::Identity() * dt;                        // v对g
    F.block<3, 3>(6, 6) = SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();     // theta对theta
    F.block<3, 3>(6, 9) = -Mat3d::Identity() * dt;                        // theta对bg

    // dx_ = F * dx_;
    cov_ = F * cov_.eval() * F.transpose() + Q_;
    current_time_ = imu.timestamped_;
    return true;
}
/**
 * @brief 轮速计修正
 */
template <typename S>
bool ESKF<S>::ObserveWheelSpeed(const Odom& odom) {
    assert(odom.timestamped_ >= current_time_);
    Eigen::Matrix<double, 3, 18> H = Eigen::Matrix<double, 3, 18>::Zero();
    H.block<3, 3>(0, 3) = Mat3d::Identity();
    // 卡尔曼增益
    Eigen::Matrix<double, 18, 3> K = cov_ * H.transpose() * (H * cov_ * H.transpose + odom_noise_).inverse();
    // 车体速度
    double velo_l = options_.wheel_radius_ * odom.left_pluse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    double velo_r =
        options_.wheel_radius_ * odom.right_pluse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    double average_vel = 0.5 * (velo_l + velo_r);
    Vec3d vel_odom(average_vel, 0, 0);
    Vec3d vel_world = R_ * vel_odom;
    // k * delta_v;
    dx_ = K * (vel_world - v_);
    cov_ = (Eigen::Matrix<double, 18, 18>::Identity() - K * H) * cov_;
    UpdateAndReset();

    return true;
}
template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, double trans_noise, double ang_noise) {
    Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
    H.block<3, 3>(0, 0) = Mat3d::Identity();
    H.block<3, 3>(3, 6) = Mat3d::Identity();
    Vec6d noise;
    noise << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;
    Mat6d V = noise.asDiagonal();
    Eigen::Matrix<double, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();
    Vec6d residual = Vec6d::Zero();
    residual.head<3>() = (pose.translation() - p_);
    residual.tail<3>() = (R_.inverse() * pose.so3()).log();
    dx_ = K * residual;
    cov_ = (Mat18d::Identity() - K * H) * cov_;
    UpdateAndReset();
    return true;
}

template <typename S>
bool ESKF<S>::ObserveSE3(const SE3& pose, const Vec6d& noise) {
    Eigen::Matrix<double, 6, 18> H = Eigen::Matrix<double, 6, 18>::Zero();
    H.block<3, 3>(0, 0) = Mat3d::Identity();
    H.block<3, 3>(3, 6) = Mat3d::Identity();
    Mat6d V = noise.asDiagonal();
    Eigen::Matrix<double, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();
    Vec6d residual = Vec6d::Zero();
    residual.head<3>() = (pose.translation() - p_);
    residual.tail<3>() = (R_.inverse() * pose.so3()).log;
    dx_ = K * residual;
    cov_ = (Mat18d::Identity() - K * H) * cov_;
    UpdateAndReset();
    return true;
}
}  // namespace ctlio::slam