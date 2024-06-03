#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <sophus/so3.hpp>

/**
 * @brief 这个类，slam状态中的状态量,描述pvq姿态的
 */

namespace lio_slam {
template <typename T>
class NaviState {
   public:
    using Vec3 = Eigen::Matrix<T, 3, 1>;
    using SO3 = Sophus::SO3<T>;
    // from time, R, p, v, bg, ba
    explicit NavState(double time, const SO3& R = SO3(), const Vec3& t = Vec3::Zero(), const Vec3& v = Vec3::Zero(),
                      const Vec3& bg = Vec3::Zero(), const Vec3& ba = Vec3::Zero())
        : timestamp_(time), R_(R), p_(t), v_(v), bg_(bg), ba_(ba) {
    }

    // from pose and vel
    NavState(double time, const SE3& pose, const Vec3& vel = Vec3::Zero())
        : timestamp_(time), R_(pose.so3()), p_(pose.translation()), v_(vel) {
    }

    /// 转换到Sophus
    Sophus::SE3<T> GetSE3() const {
        return SE3(R_, p_);
    }

    // 重载<<符号打印，方便调试
    friend std::ostream& operator<<(std::ostream& os, const NavState<T>& s) {
        os << "p: " << s.p_.transpose() << ", v: " << s.v_.transpose()
           << ", q: " << s.R_.unit_quaternion().coeffs().transpose() << ", bg: " << s.bg_.transpose()
           << ", ba: " << s.ba_.transpose();
        return os;
    }

   private:
    double timestamp_ = 0.0;
    Vec3 p_ = Vec3::Zero();
    Vec3 v_ = Vec3::Zero();
    SO3 R_;
    Vec3 bg_ = Vec3::Zero();
    Vec3 ba_ = Vec3::Zero();
};

using NaviStateD = lio_slam::NaviState<double>;
using NaviStateF = lio_slam::NaviState<float>;
}  // namespace lio_slam
