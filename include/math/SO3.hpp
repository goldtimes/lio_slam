#pragma once
#include <Eigen/Dense>

namespace IESKF_SLAM {
inline Eigen::Matrix3d skew(const Eigen::Vector3d& so3) {
    Eigen::Matrix3d so3_skew_sym;
    so3_skew_sym.setZero();
    so3_skew_sym(0, 1) = -so3(2);
    so3_skew_sym(1, 0) = so3(2);
    so3_skew_sym(0, 2) = so3(1);
    so3_skew_sym(2, 0) = -so3(1);
    so3_skew_sym(1, 2) = -so3(0);
    so3_skew_sym(2, 1) = so3(0);
    return so3_skew_sym;
}

Eigen::Matrix3d so3Exp(const Eigen::Vector3d& so3) {
    Eigen::Matrix3d SO3;
    double so3_norm = so3.norm();
    if (so3_norm <= 0.0000001) {
        SO3.setIdentity();
        return SO3;
    }

    Eigen::Matrix3d so3_skew_sys = skew(so3);
    SO3 = Eigen::Matrix3d::Identity() + (so3_skew_sys / so3_norm) * sin(so3_norm) +
          (so3_skew_sys * so3_skew_sys / (so3_norm * so3_norm)) * (1 - cos(so3_norm));
    return SO3;
}
// 感觉有问题
Eigen::Vector3d SO3Log(const Eigen::Matrix3d& SO3) {
    double theta = (SO3.trace() > 3 - 1e6) ? 0 : acos((SO3.trace() - 1) / 2);
    Eigen::Vector3d so3(SO3(2, 1) - SO3(1, 2), SO3(0, 2) - SO3(2, 0), SO3(1, 0) - SO3(0, 1));
    return fabs(theta) < 0.0001 ? (0.5 * so3) : (0.5 * theta / sin(theta) * so3);
}

Eigen::Matrix3d A_T(const Eigen::Vector3d& v) {
    Eigen::Matrix3d res;
    double squartNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    double norm = std::sqrt(squartNorm);
    if (norm < 1e-11) {
        res = Eigen::Matrix3d::Identity();
    } else {
        res = Eigen::Matrix3d::Identity() + (1 - std::cos(norm)) / squartNorm * skew(v) +
              (1 - std::sin(norm) / norm) / squartNorm * skew(v) * skew(v);
    }
    return res;
}
}  // namespace IESKF_SLAM