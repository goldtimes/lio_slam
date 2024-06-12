#include "lio/imu_process.hh"

namespace lio {

ImuProcess::ImuProcess(const std::shared_ptr<kf::IESKF>& kf)
    : ieskf_(kf),
      init_count_(0),
      last_lidar_time_end_(0.0),
      mean_acc(Eigen::Vector3d::Zero()),
      mean_gyro(Eigen::Vector3d::Zero()),
      rot_ext(Eigen::Matrix3d::Identity()),
      pos_ext(Eigen::Vector3d::Zero()) {
}

bool ImuProcess::init(const MeasureGroup& meas) {
    if (meas.imudatas.empty()) {
        return init_flag_;
    }
    // 静态初始化，估计重力和角速度bias
    for (const auto& imu : meas.imudatas) {
        init_count_++;
        mean_acc += (imu.acc_ - mean_acc) / init_count_;
        mean_gyro += (imu.gyro_ - mean_gyro) / init_count_;
    }
    if (init_count_ < max_init_count_) {
        return init_flag_;
    }
    init_flag_ = true;
    // 设置ieskf的初始化状态
    kf::State state = ieskf_->GetState();
    state.rot_ext = rot_ext;
    state.pos_ext = pos_ext;
    state.bg_ = mean_gyro;
    // 重力对齐,初始为非水平放置的情况
    if (align_gravity) {
    } else {
    }
    // ieskf_->chang_x(state);

    // 初始化噪声的协方差矩阵

    return init_flag_;
}

void ImuProcess::undistortPointCloud(const MeasureGroup& group, sensors::PointNormalCloud::Ptr& out) {
}

void ImuProcess::reset() {
    init_count_ = 0;
    init_flag_ = false;
    mean_acc = Eigen::Vector3d::Zero();
    mean_gyro = Eigen::Vector3d::Zero();
}
}  // namespace lio