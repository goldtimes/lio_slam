#pragma once
#include "common/eigen_types.hh"
#include "common/math_utils.hh"
#include "sensors/imu.hh"
#include "sensors/odom.hh"
#include "state/navi_state.hh"
namespace ctlio::slam {

enum class IcpModel { POINT_TO_PLANE = 0, CT_POINT_TO_PLANE = 1 };

struct Neighborhood {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Identity();
    double a2D = 1.0;  // 平面性参数
};

struct MeasureGroup {
    double lidar_begin_time = 0.0;
    double lidar_end_time = 0.0;
    std::vector<point3D> lidar_;
    std::deque<IMUPtr> imus_;
};

class State {
   public:
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
    Eigen::Vector3d velocity;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;

    Eigen::Quaterniond rotation_begin;
    Eigen::Vector3d translation_begin;
    Eigen::Vector3d velocity_begin;
    Eigen::Vector3d ba_begin;
    Eigen::Vector3d bg_begin;

    State(const Eigen::Quaterniond& r, const Eigen::Vector3d& trans, const Eigen::Vector3d& vel,
          const Eigen::Vector3d& bg, const Eigen::Vector3d& ba);
    State(const State* state_tmp, bool copy = false);

    void release();
};

class CloudFrame {
   public:
    double time_frame_begin;
    double time_frame_end;
    int frame_id;

    State* p_state;                   // 关键帧的位置
    std::vector<point3D> point_surf;  // global frame
    std::vector<point3D> const_surf;  // lidar frame
    std::vector<point3D> surf_keypoints;

    double dt_offset;
    bool success;
    CloudFrame(std::vector<point3D>& point_surf, std::vector<point3D>& const_surf, State* p_state);
    CloudFrame(CloudFrame* p_cloud_frame);
    void release();
};

class numType {
   public:
    template <typename Derived>
    static Eigen::Quaternion<typename Derived::Scalar> deltaQ(const Eigen::MatrixBase<Derived>& theta) {
        using Scalar_t = typename Derived::Scalar;
        Eigen::Quaternion<Scalar_t> dq;
        Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
        half_theta /= static_cast<Scalar_t>(2.0);
        dq.w() = static_cast<Scalar_t>(1.0);
        dq.x() = half_theta.x();
        dq.y() = half_theta.y();
        dq.z() = half_theta.z();
        dq.normalize();
        return dq;
    }
    /**
     * @brief 反对称矩阵
     */
    template <typename Derived>
    static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived>& mat) {
        Eigen::Matrix<typename Derived::Scalar, 3, 3> mat_skew;
        // clang-format off
        mat_skew << typename Derived::Scalar(0), -mat(2), mat(1),
                mat(2), typename Derived::Scalar(0), -mat(0),
                -mat(1), mat(0), typename Derived::Scalar(0);
        // clang-format on
        return mat_skew
    }
};
}  // namespace ctlio::slam