#pragma once
#include "ieskf_fastlio/ieskf_fastlio.hh"
#include "sensors/imu.hh"
#include "utils/lio_utils.hh"

namespace lio {

struct State {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
    Eigen::Matrix3d rot;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;

    State(double t, const Eigen::Vector3d& a, const Eigen::Vector3d& g, const Eigen::Matrix3d& r,
          const Eigen::Vector3d& p, const Eigen::Vector3d& v)
        : offset(t), pos(p), rot(r), vel(v), acc(a), gyro(g) {
    }
    // 和lidar_begin_time的offset
    double offset;
};

/**
 * @brief imu的初始化以及对点云去畸变
 */
class ImuProcess {
   public:
    ImuProcess(const std::shared_ptr<kf::IESKF>& kf);

    bool init(const MeasureGroup& group);

    void undistortPointCloud(const MeasureGroup& group, sensors::PointNormalCloud::Ptr& out);

    void setExtParams(Eigen::Matrix3d& rot_ext, Eigen::Vector3d& pos_ext);
    void setCov(Eigen::Vector3d gyro_cov, Eigen::Vector3d acc_cov, Eigen::Vector3d gyro_bias_cov,
                Eigen::Vector3d acc_bias_cov);
    void setCov(double gyro_cov, double acc_cov, double gyro_bias_cov, double acc_bias_cov);
    void reset();
    bool operator()(const MeasureGroup& meas, sensors::PointNormalCloud::Ptr& out);
    void setAlignGravity(bool flag) {
        align_gravity = flag;
    }

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
    sensors::IMU last_imu_;
    Eigen::Vector3d last_acc_;
    Eigen::Vector3d last_gyro_;
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
    // 预测的系统噪声
    kf::Matrix12d Q_;
};
}  // namespace lio