#pragma once
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <iostream>
#include "lio/lio_utils.hh"

namespace ctlio::slam {
/**
 * @brief 点到平面的残差解析求导
 */
class LidarPlaneNormFactor : public ceres::SizedCostFunction<1, 3, 4> {
   public:
    LidarPlaneNormFactor(const Eigen::Vector3d& point_body, const Eigen::Vector3d& norm_vec, double norm_offset_,
                         double weight = 1.0);

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    void check(double** parameters);

    Eigen::Vector3d point_body_;
    // 平面法向量的前三个参数
    Eigen::Vector3d norm_vector_;
    // 平面法向量的第四个参数
    double norm_offset_;
    double weight_;
    // i-l外参
    static Eigen::Vector3d t_il;
    static Eigen::Quaterniond q_il;
    static double sqrt_info;
};

class CTLidarPlaneNormFactor : public ceres::SizedCostFunction<1, 3, 4, 3, 4> {
   public:
    CTLidarPlaneNormFactor(const Eigen::Vector3d& point_body, const Eigen::Vector3d& norm_vector, double norm_offset,
                           double alpha_time, double weight = 1.0);

    virtual bool Evaluate(double const* const* parameters, double* residula, double** jacobians) const;

    void check(double** parameters);

    Eigen::Vector3d point_body_;
    Eigen::Vector3d norm_vector_;
    double norm_offset_;
    double alpha_time_;
    double weight_;

    static Eigen::Vector3d t_il;
    static Eigen::Quaterniond q_il;
    static double sqrt_info;
};

class LocationConsistencyFactor : public ceres::SizedCostFunction<3, 3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    LocationConsistencyFactor(const Eigen::Vector3d& prev_location_, double beta_);
    virtual ~LocationConsistencyFactor() {
    }
    virtual bool Evaluate(double const* const* parameters, double* residual, double** jabobians) const;
    Eigen::Vector3d previous_location;
    double beta = 1.0;
};

class RotationConsistencyFactor : public ceres::SizedCostFunction<3, 4> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    RotationConsistencyFactor(const Eigen::Quaterniond& previous_rotation, double beta);
    virtual ~RotationConsistencyFactor() = default;

    virtual bool Evaluate(double const* const* parameters, double* residual, double** jacobinas) const;
    Eigen::Quaterniond previous_rotation_;
    double beta_ = 1.0;
};

class SmallVelocityFactor : public ceres::SizedCostFunction<3, 3, 3> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SmallVelocityFactor(double beta_);

    virtual ~SmallVelocityFactor() {
    }

    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    double beta;
};

class VelocityConsistencyFactor : public ceres::SizedCostFunction<3, 6, 6> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VelocityConsistencyFactor(const Eigen::Vector3d& prev_velcotiy, double beta);
    virtual ~VelocityConsistencyFactor() = default;
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d previous_velocity_;

    double beta = 1.0;
};

class VelocityConsistencyFactor2 : public ceres::SizedCostFunction<9, 9> {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VelocityConsistencyFactor2(const State& prev_state, double beta);
    virtual ~VelocityConsistencyFactor2() = default;
    virtual bool Evaluate(double const* const* parameters, double* residuals, double** jacobians) const;

    Eigen::Vector3d previous_velocity_;
    Eigen::Vector3d previous_ba;
    Eigen::Vector3d previous_bg;

    double beta = 1.0;
};

class TruncatedLoss : public ceres::LossFunction {
   public:
    explicit TruncatedLoss(double sigma) : sigma2_(sigma * sigma) {
    }

    void Evaluate(double, double*) const override;

   private:
    const double sigma2_;
};

}  // namespace ctlio::slam