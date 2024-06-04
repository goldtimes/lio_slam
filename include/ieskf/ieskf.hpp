#pragma once
#include <Eigen/Core>
#include <functional>
#include <sophus/so3.hpp>

namespace kf {

const double GRAVITY = 9.81;
using Vec3d = Eigen::Vector3d;
using Mat3d = Eigen::Matrix3d;

using Vec24d = Eigen::Matrix<double, 24, 1>;
using Mat24d = Eigen::Matrix<double, 24, 24>;

Eigen::Matrix3d rightJacobian(const Eigen::Vector3d& values) {
    return Sophus::SO3d::jl(values);
}

// 这里是仿造fastlio2复杂的ieskf框架
// 这里定义的结构体以及维度是存放观测对p,r,rot_ext,pos_ext的jacobian
struct SharedState {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, 12, 12> H;
    Eigen::Matrix<double, 12, 1> b;
};

/**
 * @brief 状态量的定义
 */
struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3d p_ = Vec3d::Zero();
    Mat3d R_ = Mat3d::Identity();
    Mat3d rot_ext = Mat3d::Identity();  // 外参
    Vec3d pos_ext = Vec3d::Zero();
    Vec3d v_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
    Vec3d g_{0, 0, -9.81};
    void initGravity(const Vec3d& gravity) {
        g_ = gravity.normalized() * GRAVITY;
    }

    void operator+=(const Vec24d& delta);
    Vec24d operator-(const State& other);
};

struct InputData {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec3d acc_;
    Vec3d gyro_;
    InputData() = default;
    InputData(const Vec3d& acc, const Vec3d& gyro) : acc_(acc), gyro_(gyro) {
    }
};

using MeasureFunc = std::function<void(State&, SharedState&)>;

class IESKF {
   public:
    IESKF() = default;
    IESKF(int max_iters) : max_iter(max_iters) {
    }

    bool Predict(const InputData& input, double dt, const Eigen::Matrix<double, 12, 12>& Q);
    bool Update();

    void SetP(const Mat24d& p) {
        P_ = p;
    }

    void set_mearsure_fun(std::function<void(State&, SharedState&)> func) {
        measure_func_ = func;
    }

    State GetState() const {
        return x_;
    }

   private:
    int max_iter;
    // 迭代退出的阈值
    double eps_ = 0.001;
    // 状态量
    State x_;
    // 协方差
    Mat24d P_;
    // jabobian矩阵
    Mat24d H_;
    Vec24d b_;
    // 误差运动方程对误差状态的jacobian
    Mat24d F_;
    Eigen::Matrix<double, 24, 12> Fw_;
    MeasureFunc measure_func_;
};

void State::operator+=(const Vec24d& delta) {
    p_ += delta.segment<3>(0);
    R_ *= Sophus::SO3d::exp(delta.segment<3>(3)).matrix();
    rot_ext *= Sophus::SO3d::exp(delta.segment<3>(6)).matrix();
    pos_ext *= delta.segment<3>(9);
    v_ += delta.segment<3>(12);
    bg_ += delta.segment<3>(15);
    ba_ += delta.segment<3>(18);
    // 这里的加法和高博的ieskf不一样
    g_ += delta.segment<3>(21);
}

bool IESKF::Predict(const InputData& input, double dt, const Eigen::Matrix<double, 12, 12>& Q) {
    Vec24d dx = Vec24d::Zero();
    dx.segment<3>(0) = x_.v_ * dt;
    dx.segment<3>(3) = (input.gyro_ - x_.bg_) * dt;
    dx.segment<3>(12) = (x_.R_ * (input.acc_ - x_.ba_) + x_.g_) * dt;
    // fx, fw
    F_.setIdentity();
    // F_d&p_d&v
    F_.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity() * dt;
    // f_d&r_d&r
    F_.block<3, 3>(3, 3) = Sophus::SO3d::exp(-(input.gyro_ - x_.bg_) * dt).matrix();
    // f_d&r_d&bg
    F_.block<3, 3>(3, 15) = -rightJacobian((input.gyro_ - x_.bg_) * dt) * dt;
    //  F_d&v_d&r
    F_.block<3, 3>(12, 3) = -x_.R_ * Sophus::SO3d::hat(input.acc_ - x_.ba_) * dt;
    // F_d&v_d&bg
    F_.block<3, 3>(12, 18) = -x_.R_ * dt;
    // F_d&v_d&g
    F_.block<3, 3>(12, 21) = Eigen::Matrix3d::Identity() * dt;

    Fw_.setZero();
    Fw_.block<3, 3>(3, 0) = -rightJacobian((input.gyro_ - x_.bg_) * dt) * dt;
    Fw_.block<3, 3>(12, 3) = -x_.R_ * dt;
    Fw_.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity() * dt;
    Fw_.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity() * dt;

    x_ += dx;
    P_ = F_ * P_ * F_.transpose() + Fw_ * Q * Fw_.transpose();
}
bool IESKF::Update() {
}
}  // namespace kf