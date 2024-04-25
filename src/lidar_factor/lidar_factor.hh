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

// 体素中点云分布的cost函数
struct FuntorPointToDistribution {
    static constexpr int NumResiduals() {
        return 1;
    }

    FuntorPointToDistribution(const Eigen::Vector3d& world_point, const Eigen::Vector3d& raw_point,
                              const Eigen::Matrix3d& cov, double weight = 1.0)
        : world_point_(world_point), raw_point_(raw_point), weight_(weight) {
        // Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov);
        Neighborhood_information_ = (cov + Eigen::Matrix3d::Identity() * epsilon).inverse();
    }

    template <typename T>
    bool operator()(const T* const rot_param, const T* const trans_params, T* residual) const {
        Eigen::Map<Eigen::Quaternion<T>> quat(const_cast<T*> rot_param);
        Eigen::Matrix<T, 3, 1> transformed_point = quat.normalized() * raw_point_.template cast<T>();
        transformed_point(0, 0) += trans_params[0];
        transformed_point(1, 0) += trans_params[1];
        transformed_point(2, 0) += trans_params[2];

        Eigen::Matrix<T, 3, 1> diff = transformed_point - world_point_.template cast<T>();
        residual[0] = T(weight_) * (diff.transpose() * Neighborhood_information_ * diff)(0, 0);
        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector3d& world_point, const Eigen::Vector3d& raw_point,
                                       const Eigen::Matrix3d& cov, double weight = 1.0) {
        return (new ceres::AutoDiffCostFunction<FuntorPointToDistribution, 1, 4, 3>(
            new FuntorPointToDistribution(world_point, raw_point, cov, weight)));
    }

    Eigen::Vector3d world_point_;
    Eigen::Vector3d raw_point_;
    Eigen::Matrix3d Neighborhood_information_;
    double weight_ = 1.0;
    double epsilon = 0.05;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// ct-functor
template <typename FunctorT>
struct CTFunctor {
    static constexpr int NumResiduals() {
        return FuncT::NumResiduals();
    }

    using cost_function_t = ceres::AutoDiffCostFunction<CTFunctor<Functor>, FunctorT::NumResiduals(), 4, 3, 4, 3>;
    CTFunctor(double timestamp, const Eigen::Vector3d& world_point, const Eigen::Vector3d& raw_point,
              const Eigen::Vector3d& desc, double weight)
        : FuncT(world_point, raw_point, desc, weight), alpha_time_(timestamp) {
    }
    template <typename T>
    inline bool operator()(const T* const begin_rot, const T* const begin_trans, const T* const end_rot,
                           const T* const end_trans, T* residual) const {
        T alpha_m = T(1.0 - alpha_time_);
        T alpha = T(alpha_time_);
        Eigen::Map<Eigen::Quaternion<T>> quat_begin(const_cast<T*>(begin_rot));
        Eigen::Map<Eigen::Quaternion<T>> quat_end(const_cast<T*>(end_rot));
        Eigen::Quaternion<T> quat_inter = quat_begin.normalized().slerp(T(alpha), quat_end.normalized());
        quat_inter.normalize();
        Eigen::Matrxi<T, 3, 1> tr;
        tr(0, 0) = alpha_m * begin_trans[0] + alpha_time_ * end_trans[0];
        tr(1, 0) = alpha_m * begin_trans[1] + alpha_time_ * end_trans[1];
        tr(2, 0) = alpha_m * begin_trans[2] + alpha_time_ * end_trans[2];
        return functor(quat_inter.coeffs().data(), tr.data(), residual);
    }

    FunctorT functor;
    double alpha_time_ = 1.0;
};
}  // namespace ctlio::slam