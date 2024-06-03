#pragma once

#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iomanip>
#include "common/math_utils.hh"
#include "common/navi_state.hh"
#include "sensors/gnss.hh"
#include "sensors/imu.hh"
#include "sensors/odom.hh"
#include "sophus/se3.hpp"

using namespace sensors;
namespace lio_slam {
template <typename T = double>
class ESKF {
   public:
    using SO3 = Sophus::SO3<T>;
    using Vec3T = Eigen::Matrix<T, 3, 1>;
    using Mat3T = Eigen::Matrix<T, 3, 3>;
    using Vec18T = Eigen::Matrix<T, 18, 1>;         // 状态量
    using Mat18T = Eigen::Matrix<T, 18, 18>;        // 协方差类型
    using OdomNoiseT = Eigen::Matrix<T, 3, 3>;      // 速度噪声
    using GnssNoiseT = Eigen::Matrix<T, 6, 6>;      // gnss pose的噪声，旋转平移
    using MotionNoiseT = Eigen::Matrix<T, 18, 18>;  // 系统运动模型的噪声
    using NaviStateT = NaviState<T>;                // 导航的状态
    using SE3 = Sophus::SE3<T>;
    // eskf的imu初始化参数以及各种噪声的参数的配置
    struct Options {
        Options() = default;

        double imu_dt_ = 0.1;
        double gyro_var_ = 1e-5;       // 陀螺测量标准差
        double acce_var_ = 1e-2;       // 加计测量标准差
        double bias_gyro_var_ = 1e-6;  // 陀螺零偏游走标准差
        double bias_acce_var_ = 1e-4;  // 加计零偏游走标准差

        /// 里程计参数
        double odom_var_ = 0.5;
        double odom_span_ = 0.1;        // 里程计测量间隔
        double wheel_radius_ = 0.155;   // 轮子半径
        double circle_pulse_ = 1024.0;  // 编码器每圈脉冲数

        /// RTK 观测参数
        double gnss_pos_noise_ = 0.1;                   // GNSS位置噪声
        double gnss_height_noise_ = 0.1;                // GNSS高度噪声
        double gnss_ang_noise_ = 1.0 * math::kDEG2RAD;  // GNSS旋转噪声
        /// 其他配置
        bool update_bias_gyro_ = true;  // 是否更新陀螺bias
        bool update_bias_acce_ = true;  // 是否更新加计bias
    };

   public:
    ESKF(Options options = Options()) : options_(options) {
        BuildNoise(options);
    }

    // eskf的预测步骤
    bool Predict(const IMU& imu);
    // 速度的观测
    bool ObserveWheelSpeed(const Odom& odom);
    // gps的观测
    bool ObserveGps(const GNSS& gnss);

    bool ObserveSE3(const SE3& pose, double trans_noise = 0.1, double ang_noise = 1.0 * kDEG2RAD);

    NaviStateT GetNominalState() const {
        return NaviStateT(current_time_, R_, p_, v_, bg_, ba_);
    }

    SE3 GetNominalSE3() const {
        return SE3(R_, p_);
    }

    void SetX(const NaviStateD& x, const Eigen::Vector3d& gravity) {
        current_time_ = x.timestamp_;
        R_ = x.R_;
        p_ = x.p_;
        v_ = x.v_;
        bg_ = x.bg_;
        ba_ = x.ba_;
        g_ = gravity;
    }
    /// 设置协方差
    void SetCov(const Mat18T& cov) {
        cov_ = cov;
    }

    /// 获取重力
    Vec3d GetGravity() const {
        return g_;
    }

   private:
    /**
     * @brief 更新状态量并且重置dx
     */
    void UpdateStateAndReset() {
        p_ = p_ + dx_.template block<3, 1>(0, 0);
        v_ = v_ + dx_.template block<3, 1>(3, 0);
        R_ = R_ * SO3::exp(dx_.template block<3, 1>(6, 0));

        bg_ = bg_ + dx_.template block<3, 1>(9, 0);
        ba_ = ba_ + dx_.template block<3, 1>(12, 0);
        g_ = g_ + dx_.template block<3, 1>(15, 0);
        // 协防差投影，这里为什么需要投影
        // 重置dx状态
        dx_.setZero();
    }

    void ProjectCov() {
        Mat18T J = Mat18T::Identity();
        J.template block<3, 3>(6, 6) = Mat3T::Identity() - 0.5 * SO3::hat(dx_.template block<3, 1>(6, 0));
        cov_ = J * cov_ * J.transpose();
    }

    void BuildNoise(const Options& options) {
        double ev = options.acce_var_;
        double et = options.gyro_var_;
        double eg = options.bias_gyro_var_;
        double ea = options.bias_acce_var_;

        double ev2 = ev;  // * ev;
        double et2 = et;  // * et;
        double eg2 = eg;  // * eg;
        double ea2 = ea;  // * ea;

        // 设置过程噪声
        Q_.diagonal() << 0, 0, 0, ev2, ev2, ev2, et2, et2, et2, eg2, eg2, eg2, ea2, ea2, ea2, 0, 0, 0;

        // 设置里程计噪声
        double o2 = options_.odom_var_ * options_.odom_var_;
        odom_noise_.diagonal() << o2, o2, o2;

        // 设置GNSS状态
        double gp2 = options.gnss_pos_noise_ * options.gnss_pos_noise_;
        double gh2 = options.gnss_height_noise_ * options.gnss_height_noise_;
        double ga2 = options.gnss_ang_noise_ * options.gnss_ang_noise_;
        gnss_noise_.diagonal() << gp2, gp2, gh2, ga2, ga2, ga2;
    }

   private:
    double current_time_ = 0.0;
    // 名义状态量
    Vec3T p_ = Vec3T::Zero();
    Vec3T v_ = Vec3T::Zero();
    SO3 R_;
    Vec3T bg_ = Vec3T::Zero();
    Vec3T ba_ = Vec3T::Zero();
    Vec3T g_{0, 0, -9.8};
    // 误差状态量
    Vec18T dx_ = Vec18T::Zero();
    // 协方差
    Mat18T cov_ = Mat18T::Identity();
    // 系统噪声
    MotionNoiseT Q_ = MotionNoiseT::Zero();
    OdomNoiseT odom_noise_ = OdomNoiseT::Zero();
    GnssNoiseT gnss_noise_ = GnssNoiseT::Zero();

    bool first_gnss_ = true;

    Options options_;
};

using ESKFD = ESKF<double>;
using ESKFF = ESKF<float>;

template <typename T>
bool ESKF<T>::Predict(const IMU& imu) {
    assert(imu.timestamp_ >= current_time_);
    double dt = imu.timestamp_ - current_time_;
    if (dt > (5 * options_.imu_dt_) || dt < 0) {
        LOG(INFO) << "skip this imu, because dt_ = " << dt;
        current_time_ = imu.timestamp_;
        return false;
    }
    // 名义状态量的递推，不需要考虑噪声

    Vec3T new_p = p_ + v_ * dt + 0.5 * (R_ * (imu.acc_ - ba_)) * dt * dt + 0.5 * g_ * dt * dt;
    Vec3T new_v = v_ + R_ * (imu.acc_ - ba_) * dt + g_ * dt;
    SO3 new_R = R_ * SO3::exp((imu.gyro_ - bg_) * dt);

    R_ = new_R;
    p_ = new_p;
    v_ = new_v;

    // error 状态递推
    // 计算误差的运动方程对误差状态量的jacobian
    Mat18T F = Mat18T::Idetity();
    F.template block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
    F.template block<3, 3>(3, 6) = -R_.matrix() * SO3::hat((imu.acc_ - ba_)) * dt;
    F.template block<3, 3>(3, 12) = -R_.matrix() * dt;
    F.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
    F.template block<3, 3>(6, 6) = SO3::exp(-(imu.gyro_ - bg_) * dt).matrix();
    F.template block<3, 3>(6, 9) = -Eigen::Matrix3d::Identity() * dt;

    // 协方差的递推
    cov_ = F * cov_.eval() * F.transpose() + Q_;
    current_time_ = imu.timestamp_;
    return true;
}

template <typename T>
bool ESKF<T>::ObserveWheelSpeed(const Odom& odom) {
    assert(odom.timestamp_ >= current_time_);
    // 速度对误差状态量的jacobian
    Eigen::Matrix<T, 3, 18> H = Eigen::Matrix<T, 3, 18>::Zero();
    H.template block<3, 3>(0, 3) = Eigen::Matrix<T, 3, 3>::Identity();
    // 计算K 增益
    Eigen::Matrix<T, 18, 3> K = cov_ * H.transpose() * (H * cov_.eval() * H.transpose() + odom_noise_).inverse();
    // 计算残差
    // velocity obs
    double velo_l = options_.wheel_radius_ * odom.left_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    double velo_r =
        options_.wheel_radius_ * odom.right_pulse_ / options_.circle_pulse_ * 2 * M_PI / options_.odom_span_;
    double average_vel = 0.5 * (velo_l + velo_r);

    Vec3T vel_odom(average_vel, 0.0, 0.0);
    Vec3T vel_world = R_ * vel_odom;
    Vec3T res = vel_world - V;
    // 计算dx
    dx_ = K * res;
    // 更新协方差
    cov_ = (Mat18T::Identity() - K * H) * cov_;
    UpdateStateAndReset();
    return true;
}

template <typename T>
bool ESKF<T>::ObserveGps(const GNSS& gnss) {
    assert(gnss.unix_time_ >= current_time_);
    if (first_gnss_) {
        R_ = gnss.utm_pose_.so3();
        p_ = gnss.utm_pose_.translation();
        first_gnss_ = false;
        current_time_ = gnss.unix_time_;
        return false;
    }
    assert(gnss.heading_valid_);
    ObserveSE3(gnss.utm_pose_, options_.gnss_pos_noise_, options_.gnss_ang_noise_);
    current_time_ = gnss.unix_time_;
    return true;
}

template <typename T>
bool ESKF<T>::ObserveSE3(const SE3& pose, double trans_noise, double ang_noise) {
    Eigen::Matrix<T, 6, 18> H = Eigen::Matrix<T, 6, 18>::Zero();
    H.template block<3, 3>(0, 0) = Eigen::Matrix<T, 3, 3>::Identity();
    H.template block<3, 3>(3, 6) = Eigen::Matrix<T, 3, 3>::Identity();
    Eigen::Matrix<T, 6, 1> noise_vec;
    noise_vec << trans_noise, trans_noise, trans_noise, ang_noise, ang_noise, ang_noise;
    Eigen::Matrix<T, 6, 6> V = noise_vec.asDiagonal();
    // 卡尔曼增益
    Eigen::Matrix<T, 18, 6> K = cov_ * H.transpose() * (H * cov_ * H.transpose() + V).inverse();
    // 计算残差
    Eigen::Matrix<T, 6, 1> res = Eigen::Matrix<T, 6, 1>::Zero();
    res.template block<3, 1>(0, 0) = (pose.translation() - p_);
    res.template block<3, 1>(3, 0) = (R_.inverse() * pose.so3()).log();

    // 计算dx
    dx_ = K * res;
    cov_ = (Mat18T::Identity() - K * H) * cov_;
    UpdateStateAndReset();
    return true;
}

}  // namespace lio_slam
