#pragma once
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <iostream>
#include "lio/lio_utils.hh"

namespace ctlio::slam {
struct FunctorPointToPlane {
    static constexpr int NumResiduals() {
        return 1;
    }
    FunctorPointToPlane(const Eigen::Vector3d& point_world, const Eigen::Vector3d& point_body,
                        const Eigen::Vector3d& norm, double weight = 1.0)
        : world_reference_(point_world), raw_point_(point_body), reference_normal_(norm), weight_(weight) {
    }
    /**
     * @brief 这里的rot 和 trans 在AddResidual残差块的时候传入
     * 残差的维度为1 因为点到面的距离是一维
     */
    template <typename T>
    bool operator()(const T* const rot_params, const T* const trans_params, T* residual) const {
        Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T*>(rot_params));
        // 转到世界坐标系下
        Eigen::Matrix<T, 3, 1> transformed = quat.nomalized() * raw_point_.template cast<T>();
        transformed(0, 0) += trans_params[0];
        transformed(1, 0) += trans_params[1];
        transformed(2, 0) += trans_params[2];

        T product =
            (world_reference_.template cast<T>() - transformed).transpose() * reference_normal_.template cast<T>();
        residual[0] = T(weight_) * product;
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& point_world, const Eigen::Vector3d& point_body,
                                       const Eigen::Vector3d& norm, double weight_ = 1.0) {
        return new ceres::AutoDiffCostFunction<FunctorPointToPlane, 1, 4, 3>(
            new FunctorPointToPlane(point_world, point_body, norm, weight_));
    }

    Eigen::Vector3d world_reference_;
    Eigen::Vector3d raw_point_;
    Eigen::Vector3d reference_normal_;
    double weight_ = 1.0;
    Eigen::Matrix<double, 1, 1> sqrt_info;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct FunctorPointToLine {
    static constexpr int NumResidual() {
        return 1;
    }

    FunctorPointToLine(const Eigen::Vector3d& point_world, const Eigen::Vector3d& point_body,
                       const Eigen::Vector3d& direction, double weight)
        : world_reference_(point_world), raw_point_(point_body), direction_(direction), weight_(weight) {
    }

    template <typename T>
    bool operator()(const T* const rot, const T* const trans, T* residuals) {
        Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T*>(rot));
        Eigen::Matrix<T, 3, 1> transformed = quat * raw_point_.template cast<T>();
        transformed(0, 0) += trans[0];
        transformed(1, 0) += trans[1];
        transformed(2, 0) += trans[2];
        Eigen::Matrix<T, 3, 1> cross = direction_.template cast<T>();
        residuals[0] =
            T(weight_) * cross.normalized().template cross((transformed - world_reference_.template cast<T>())).norm();
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& point_world, const Eigen::Vector3d& point_body,
                                       const Eigen::Vector3d& norm_vector, double weight = 1.0) {
        return new ceres::AutoDiffCostFunction<FunctorPointToLine, 1, 4, 3>(
            new FunctorPointToLine(point_world, point_body, norm_vector, weight));
    }

    Eigen::Vector3d world_reference_;
    Eigen::Vector3d raw_point_;
    Eigen::Vector3d direction_;
    double weight_ = 1.0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct FunctorPointToPoint {
    static constexpr int NumResiduals() {
        return 3;
    }
    FunctorPointToPoint(const Eigen::Vector3d& point_world, const Eigen::Vector3d& point_body,
                        const Eigen::Vector3d& norm_vector, double weight)
        : world_reference_(point_world), raw_point_(point_body), weight_(weight) {
    }

    template <typename T>
    bool operator()(const T* const rot, const T* const trans, T* residual) const {
        Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T*>(rot));
        Eigen::Matrix<T, 3, 1> transformed = quat * raw_point_.template cast<T>();
        transformed(0, 0) += trans[0];
        transformed(1, 0) += trans[1];
        transformed(2, 0) += trans[2];
        T t_weight = T(weight_);
        residual[0] = t_weight * (transformed(0) - T(world_reference_(0)));
        residual[1] = t_weight * (transformed(1) - T(world_reference_(1)));
        residual[2] = t_weight * (transformed(2) - T(world_reference_(2)));
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& point_world, const Eigen::Vector3d& point_body,
                                       const Eigen::Vector3d& norm_vector_, double weight_ = 1.0) {
        return new ceres::AutoDiffCostFunction<FunctorPointToPoint, 3, 4, 3>(
            new FunctorPointToPoint(point_world, point_body, norm_vector_, weight_));
    }

    Eigen::Vector3d world_reference_;
    Eigen::Vector3d raw_point_;
    double weight_ = 1.0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

}  // namespace ctlio::slam