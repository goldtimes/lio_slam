#include <sophus/so3.h>
#include "common/eigen_types.hh"

namespace ctlio::slam {
/**
 * @brief 导航用的状态变量
 */
struct NavState {
    NavState() = default;

    explicit NavState(double time, const SO3& R = SO3(), const Vec3d& p = Vec3d::Zero(), const Vec3d& v = Vec3d::Zero(),
                      const Vec3d& bg = Vec3d::Zero(), const Vec3d& ba = Vec3d::Zero())
        : timestamped_(time), R_(R), p_(p), v_(v), bg_(bg), ba_(ba) {
    }

    NavState(double time, const SE3& pose, const Vec3d& p = Vec3d::Zero(), const Vec3d& v = Vec3d::Zero())
        : timestamped_(time), R_(pose.so3()), p_(pose.translation()), v_(v) {
    }

    SE3 GetSE3() const {
        return SE3(R_, p_);
    }

    // 重载打印
    friend std::ostream& operator<<(std::ostream& os, const NavState& s) {
        os << "p:" << s.p_.transpose() << ",vel:" << s.v_.transpose()
           << ",q:" << s.R_.unit_quaternion().coeffs().transpose() << ",bg:" << s.bg_.transpose() << ",ba"
           << s.ba_.transpose();
        return os;
    }

    double timestamped_;
    SO3 R_;
    Vec3d p_ = Vec3d::Zero();
    Vec3d v_ = Vec3d::Zero();
    Vec3d bg_ = Vec3d::Zero();
    Vec3d ba_ = Vec3d::Zero();
};
}  // namespace ctlio::slam