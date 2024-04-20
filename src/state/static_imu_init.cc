#include "static_imu_init.hh"
#include <glog/logging.h>
#include "common/math_utils.hh"
namespace ctlio::slam {
bool StaticInitImu::AddImu(const IMU& imu) {
    // 初始化成功了
    if (init_success_) {
        // LOG(INFO) << "init success";
        return true;
    }
    // 没静止
    if (config_.use_speed_for_static_checking_ && !is_static_) {
        LOG(INFO) << "等待车辆静止";
        init_imu_queue_.clear();
        return false;
    }

    if (init_imu_queue_.empty()) {
        // 记录静止的时间戳
        init_start_time_ = imu.timestamped_;
    }
    init_imu_queue_.push_back(imu);
    double init_time = imu.timestamped_ - init_start_time_;
    // LOG(INFO) << "init_time:" << init_time;
    // 静止1s后
    if (init_time > config_.init_time_seconds) {
        LOG(INFO) << "try_init";
        TryInit();
    }
    // 1s 200hz的频率,如果禁止大于1s,不能让imu的数据太多
    while (init_imu_queue_.size() > config_.init_imu_queue_size) {
        init_imu_queue_.pop_front();
    }
    current_time_ = imu.timestamped_;
    return false;
}

bool StaticInitImu::AddOdom(const Odom& odom) {
    if (init_success_) {
        return true;
    }
    if (odom.left_pluse_ < config_.static_odom_pulse_ && odom.right_pluse_ < config_.static_odom_pulse_) {
        is_static_ = true;
    } else {
        is_static_ = false;
    }
    current_time_ = odom.timestamped_;
    return true;
}

/**
 * @brief 初始化
 */
bool StaticInitImu::TryInit() {
    if (init_imu_queue_.size() < 10) {
        return false;
    }
    // 计算均值和方差
    Vec3d mean_gyro;
    Vec3d mean_acce;
    math::ComputeMeanAndCovDiag(init_imu_queue_, mean_gyro, cov_gryo_, [](const IMU& imu) { return imu.gyro_; });
    math::ComputeMeanAndCovDiag(init_imu_queue_, mean_acce, cov_acc_, [](const IMU& imu) { return imu.acce_; });
    LOG(INFO) << "mean acc:" << mean_acce.transpose();
    gravity_ = -mean_acce / mean_acce.norm() * config_.gravity_norm_;

    // 重新计算加速度
    math::ComputeMeanAndCovDiag(init_imu_queue_, mean_acce, cov_acc_,
                                [this](const IMU& imu) { return imu.acce_ + gravity_; });
    // 检查imu的噪声
    if (cov_gryo_.norm() > config_.max_static_gyro_var_) {
    }
    if (cov_acc_.norm() > config_.max_static_acc_var_) {
    }

    init_bg_ = mean_gyro;
    init_ba_ = mean_acce;

    LOG(INFO) << "IMU 初始化成功，初始化时间= " << current_time_ - init_start_time_ << ", bg = " << init_bg_.transpose()
              << ", ba = " << init_ba_.transpose();
    LOG(INFO) << "gyro sq = " << cov_gryo_.transpose() << ", acce sq = " << cov_acc_.transpose()
              << ", grav = " << gravity_.transpose() << ", norm: " << gravity_.norm();
    LOG(INFO) << "mean gyro: " << mean_gyro.transpose() << " acce: " << mean_acce.transpose();
    init_success_ = true;
    return true;
}
}  // namespace ctlio::slam