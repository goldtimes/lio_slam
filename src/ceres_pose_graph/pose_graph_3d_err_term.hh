#pragma once
#include <ceres/ceres.h>
#include "types.hh"

class PoseGraph3dErrorTerm {
   public:
    // 构造函数
    PoseGraph3dErrorTerm(const Pose3D& constraint, const Eigen::Matrix<double, 6, 6>& input_sqrt_information)
        : position_constraint(constraint.p), quat_constraint(constraint.q), sqrt_information(input_sqrt_information) {
    }

    /**
     * @brief 优化过程的残差计算
     * @param position_ptr1
     * @param quat_ptr1
     * @param position_ptr2
     * @param quat_ptr2
     * @param residuals 残差
     *            约束
     * 顶点1pose -------- 顶点2pose
     */
    template <typename T>
    bool operator()(const T* position_ptr1, const T* quat_ptr1, const T* position_ptr2, const T* quat_ptr2,
                    T* residuals_ptr) const {
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_a(position_ptr1);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_b(position_ptr2);
        Eigen::Map<const Eigen::Quaternion<T>> q_a(quat_ptr1);
        Eigen::Map<const Eigen::Quaternion<T>> q_b(quat_ptr2);
        Eigen::Quaternion<T> q_a_inverse = q_a.conjugate();
        Eigen::Quaternion<T> q_ab_relative = q_a_inverse * q_b;
        Eigen::Matrix<T, 3, 1> p_ab_inverse = q_a_inverse * (p_b - p_a);
        Eigen::Quaternion<T> delta_q = quat_constraint.cast<T>() * q_ab_relative.conjugate();
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residuals(residuals_ptr);
        residuals.template block<3, 1>(0, 0) = p_ab_inverse - position_constraint.cast<T>();
        residuals.template block<3, 1>(0, 0) = T(2.0) * delta_q.vec();
        // sqrt_information.cast<T>();
        residuals = sqrt_information.cast<T>() * residuals;
        return true;
    }
    // 写一个create函数来创建残差块
    static ceres::CostFunction* Creat(const Pose3D& constraint,
                                      const Eigen::Matrix<double, 6, 6>& input_sqrt_information) {
        // clang-format off
        return  new ceres::AutoDiffCostFunction
                <PoseGraph3dErrorTerm, 6, 3, 4, 3, 4>(
            new PoseGraph3dErrorTerm(constraint, input_sqrt_information));
    }

   private:
    Eigen::Vector3d position_constraint;
    Eigen::Quaterniond quat_constraint;
    Eigen::Matrix<double, 6, 6> sqrt_information;
};
