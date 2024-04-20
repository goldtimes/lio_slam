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
}  // namespace ctlio::slam