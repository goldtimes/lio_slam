#include "analytical_factor.hh"

namespace ctlio::slam {
double LidarPlaneNormFactor::sqrt_info;
Eigen::Vector3d LidarPlaneNormFactor::t_il;
Eigen::Quaterniond LidarPlaneNormFactor::q_il;

double CTLidarPlaneNormFactor::sqrt_info;
Eigen::Vector3d CTLidarPlaneNormFactor::t_il;
Eigen::Quaterniond CTLidarPlaneNormFactor::q_il;

LidarPlaneNormFactor::LidarPlaneNormFactor(const Eigen::Vector3d& point_body, const Eigen::Vector3d& norm_vec,
                                           double norm_offset, double weight = 1.0)
    : point_body_(point_body), norm_vector_(norm_vec), norm_offset_(norm_offset), weight_(weight) {
}
/**
 * @brief 求解残差和jacobian
 */
bool LidarPlaneNormFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    Eigen::Vector3d translation(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond rotation(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);

    Eigen::Vector3d point_world = rotation * point_body_ + translation;
    double error = norm_vector_.dot(point_world) + norm_offset_;
    residuals[0] = sqrt_info * weight_ * error;
    if (jacobians) {
        // de / dt
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_trans(jacobians[0]);
            jacobian_trans.setZero();
            jacobian_trans.block<1, 3>(0, 0) = sqrt_info * norm_vector_.transpose() * weight_;
        }
        // de / dr
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_rot(jacobians[1]);
            jacobian_rot.setZero();
            jacobian_rot.block<1, 3>(0, 0) = -sqrt_info * norm_vector_.transpose() * rotation.toRotationMatrix() *
                                             numType::skewSymmetric(point_body_) * weight_;
        }
    }
    return true;
}

CTLidarPlaneNormFactor::CTLidarPlaneNormFactor(const Eigen::Vector3d& point_body, const Eigen::Vector3d& norm_vector,
                                               double norm_offset, double alpha_time, double weight)
    : point_body_(point_body),
      norm_vector_(norm_vector),
      norm_offset_(norm_offset_),
      alpha_time_(alpha_time),
      weight_(weight) {
}

bool CTLidarPlaneNormFactor::Evaluate(double const* const* parameters, double* residula, double** jacobians) const {
    const Eigen::Vector3d trans_begin(parameters[0][0], parameters[0][1], parameters[0][2]);
    const Eigen::Vector3d trans_end(parameters[2][0], parameters[2][1], parameters[2][2]);
    const Eigen::Quaterniond rot_beign(parameters[1][3], parameters[1][0], parameters[1][1], parameters[1][2]);
    const Eigen::Quaterniond rot_end(parameters[3][3], parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Quaterniond rot_slerp = rot_beign.slerp(alpha_time_, rot_end);
    rot_slerp.normalize();
    Eigen::Vector3d trans_slerp = trans_begin * (1 - alpha_time_) + alpha_time_ * trans_end;
    Eigen::Vector3d point_world = rot_slerp * point_body_ + trans_slerp;
    double error = norm_vector_.dot(point_world) + norm_offset_;
    residula[0] = sqrt_info * weight_ * error;
    if (jacobians) {
        Eigen::Matrix<double, 1, 3> jacobian_rot_slerp =
            -norm_vector_.transpose() * rot_slerp.toRotationMatrix() * numType::skewSymmetric(point_body_) * weight_;
        Eigen::Quaterniond rot_delta = rot_beign.inverse() * rot_end;
        Eigen::Quaterniond rot_identity(Eigen::Matrix3d::Identity());
        Eigen::Quaterniond rot_delta_slerp = rot_identity.slerp(alpha_time_, rot_delta);
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran_begin(jacobians[0]);
            jacobian_tran_begin.setZero();
            jacobian_tran_begin.block<1, 3>(0, 0) = norm_vector_.transpose() * weight_ * (1 - alpha_time_);
            jacobian_tran_begin = sqrt_info * jacobian_tran_begin;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot_begin(jacobians[1]);
            jacobian_rot_begin.setZero();
            Eigen::Matrix<double, 3, 3> jacobian_slerp_begin =
                (rot_delta_slerp.toRotationMatrix()).transpose() *
                (Eigen::Matrix3d::Identity() - alpha_time_ * numType::Qleft(rot_delta_slerp).bottomRightCorner<3, 3>() *
                                                   (numType::Qleft(rot_delta).bottomRightCorner<3, 3>()).inverse());
            jacobian_rot_begin.block<1, 3>(0, 0) = jacobian_rot_slerp * jacobian_slerp_begin;
            jacobian_rot_begin = sqrt_info * jacobian_rot_begin;
        }
        if (jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 1, 3, Eigen::RowMajor>> jacobian_tran_end(jacobians[2]);
            jacobian_tran_end.setZero();
            jacobian_tran_end.block<1, 3>(0, 0) = norm_vector_.transpose() * weight_ * alpha_time_;
            jacobian_tran_end = sqrt_info * jacobian_tran_end;
        }
        if (jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 1, 4, Eigen::RowMajor>> jacobian_rot_end(jacobians[3]);
            jacobian_rot_end.setZero();
            Eigen::Matrix<double, 3, 3> jacobian_slerp_end =
                alpha_time_ * numType::Qleft(rot_delta_slerp).bottomRightCorner<3, 3>() *
                (numType::Qleft(rot_delta).bottomRightCorner<3, 3>()).inverse();
            jacobian_slerp_end.block<1, 3>(0, 0) = jacobian_rot_slerp * jacobian_slerp_end;
            jacobian_slerp_end = sqrt_info * jacobian_slerp_end;
        }
    }

    return true;
}

LocationConsistencyFactor::LocationConsistencyFactor(const Eigen::Vector3d& previous_location, double beta) {
    this->previous_location = previous_location;
    this->beta = beta;
}

bool LocationConsistencyFactor::Evaluate(double const* const* parameters, double* residual, double** jacobians) const {
    residual[0] = beta * (parameters[0][0] - previous_location(0, 0));
    residual[1] = beta * (parameters[0][1] - previous_location(0, 1));
    residual[2] = beta * (parameters[0][2] - previous_location(0, 2));
    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_trans_begin(jacobians[0]);
            jacobian_trans_begin.setZero();
            jacobian_trans_begin(0, 0) = beta;
            jacobian_trans_begin(1, 1) = beta;
            jacobian_trans_begin(2, 2) = beta;
        }
    }
    return true;
}

RotationConsistencyFactor::RotationConsistencyFactor(const Eigen::Quaterniond& previous_rotation, double beta) {
    previous_rotation_ = previous_rotation;
    beta_ = beta;
}

bool RotationConsistencyFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    Eigen::Quaterniond rot_cur(parameters[0][3], parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond q_delta = previous_rotation_.inverse() * rot_cur;
    Eigen::Vector3d error = 2 * q_delta.vec();

    residuals[0] = error[0] * beta_;
    residuals[1] = error[1] * beta_;
    residuals[2] = error[2] * beta_;

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_rot_begin(jacobians[0]);
            jacobian_rot_begin.setZero();
            jacobian_rot_begin.block<3, 3>(0, 0) =
                q_delta.w() * Eigen::Matrix3d::Identity() + numType::skewSymmetric(q_delta.vec());
            jacobian_rot_begin = jacobian_rot_begin * beta_;
        }
    }
    return true;
}

SmallVelocityFactor::SmallVelocityFactor(double beta_) {
    beta = beta_;
}

bool SmallVelocityFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    residuals[0] = beta * (parameters[0][0] - parameters[1][0]);
    residuals[1] = beta * (parameters[0][1] - parameters[1][1]);
    residuals[2] = beta * (parameters[0][2] - parameters[1][2]);

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_trans_begin(jacobians[0]);
            jacobian_trans_begin.setZero();
            jacobian_trans_begin(0, 0) = beta;
            jacobian_trans_begin(1, 1) = beta;
            jacobian_trans_begin(2, 2) = beta;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_trans_end(jacobians[1]);
            jacobian_trans_end.setZero();
            jacobian_trans_end(0, 0) = -beta;
            jacobian_trans_end(1, 1) = -beta;
            jacobian_trans_end(2, 2) = -beta;
        }
    }
    return true;
}
VelocityConsistencyFactor::VelocityConsistencyFactor(const Eigen::Vector3d& prev_velcotiy, double beta) {
    previous_velocity_ = prev_velcotiy;
    this->beta = beta;
}
bool VelocityConsistencyFactor::Evaluate(double const* const* parameters, double* residuals, double** jacobians) const {
    residuals[0] = beta * (parameters[1][0] - parameters[0][0] - previous_velocity_(0, 0));
    residuals[1] = beta * (parameters[1][1] - parameters[0][1] - previous_velocity_(1, 0));
    residuals[2] = beta * (parameters[1][2] - parameters[0][2] - previous_velocity_(2, 0));

    if (jacobians) {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> jacobian_veloctiy_bias_begin(jacobians[0]);
            jacobian_veloctiy_bias_begin.setZero();
            jacobian_veloctiy_bias_begin(0, 0) = -beta;
            jacobian_veloctiy_bias_begin(1, 1) = -beta;
            jacobian_veloctiy_bias_begin(2, 2) = -beta;
        }
        if (jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> jacobian_veloctiy_bias_end(jacobians[1]);
            jacobian_veloctiy_bias_end.setZero();
            jacobian_veloctiy_bias_end(0, 0) = beta;
            jacobian_veloctiy_bias_end(1, 1) = beta;
            jacobian_veloctiy_bias_end(2, 2) = beta;
        }
    }
}

void TruncatedLoss::Evaluate(double s, double* rho) const {
    if (s < sigma2_) {
        rho[0] = s;
        rho[1] = 1.0;
        rho[2] = 0.0;
        return;
    }
    rho[0] = sigma2_;
    rho[1] = 0.0;
    rho[2] = 0.0;
}
}  // namespace ctlio::slam