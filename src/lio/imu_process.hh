#pragma once
#include "ieskf/ieskf.hpp"
#include "sensors/imu.hh"
#include "utils/lio_utils.hh"

namespace lio {

struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d pos;
    Eigen::Matrix3d rot;
    Eigen::Vector3d vel;
    Eigen::Vector3d bg;
    Eigen::Vector3d ba;

    State(double t, const Eigen::Vector3d& p, const Eigen::Matrix3d& r, const Eigen::Vector3d& v,
          const Eigen::Vector3d& bias_g, const Eigen::Vector3d& bias_a)
        : timestamp(t), pos(p), rot(r), vel(v), bg(bias_g), ba(bias_a) {
    }
    double timestamp;
};

/**
 * @brief imu的初始化以及对点云去畸变
 */
class ImuProcess {
   public:
    ImuProcess(const std::shared_ptr<kf::IESKF>& kf);

    bool init(const MeasureGroup& group);

    void undistortPointCloud(const MeasureGroup& group, sensors::PointNormalCloud::Ptr& out);

    void reset();

   private:
    int init_count_;
    int max_init_count_ = 20;
    // 外参旋转部分
    Eigen::Matrix3d rot_ext;
    // 外参平移部分
    Eigen::Vector3d pos_ext;

    std::shared_ptr<kf::IESKF> ieskf_;

    bool init_flag_ = false;
    bool align_gravity = true;

    std::vector<State> imu_state_;
    double last_lidar_time_end_;
    // 加速度的平均值
    Eigen::Vector3d mean_acc;
    // 角速度的平均值
    Eigen::Vector3d mean_gyro;
    // 协方差的对角线值
    Eigen::Vector3d gyro_sigma;
    Eigen::Vector3d acc_sigma;
    Eigen::Vector3d gyro_bias_sigma;
    Eigen::Vector3d acc_bias_sigma;
};
}  // namespace lio