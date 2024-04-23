#include "lio_utils.hh"

namespace ctlio::slam {

void transformPoint(MotionCompensation compensation, point3D &point_temp, Eigen::Quaterniond &q_begin,
                    Eigen::Quaterniond &q_end, Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end,
                    Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar) {
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    double alpha_time = point_temp.alpha_time;
    switch (compensation) {
        case MotionCompensation::NONE:
        // 匀速
        case MotionCompensation::CONSTANT_VELOCITY:
            t = t_end;
            R = q_end.toRotationMatrix();
            break;
        case MotionCompensation::CONTINUOUS:
        // 插值
        case MotionCompensation::ITERATIVE:
            R = q_begin.slerp(alpha_time, q_end).normalized().toRotationMatrix();
            t = (1.0 - alpha_time) * t_begin + alpha_time * t_end;
            break;
    }
    point_temp.point = R * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t;
}

}  // namespace ctlio::slam