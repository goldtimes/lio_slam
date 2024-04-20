#pragma once
#include <deque>
#include "common/eigen_types.hh"
#include "sensors/imu.hh"
#include "sensors/odom.hh"

namespace ctlio::slam {
/**
 * @brief imu的静止初始化
 */
class StaticInitImu {
   public:
    struct StaticOptions {
        StaticOptions() {
        }
        double init_time_seconds = 1.0;              // 静止初始化的时间
        double init_imu_queue_size = 2000;           // 静止初始化的最大队列长度
        int static_odom_pulse_ = 5;                  //静止时轮速计的脉冲阈值
        double max_static_gyro_var_ = 0.5;           // 陀螺仪测量方差,大于这个值,禁止初始化是失败
        double max_static_acc_var_ = 0.6;            // acc测量方差
        double gravity_norm_ = 9.81;                 // 重力大小
        bool use_speed_for_static_checking_ = true;  // 使用轮速计来判断车辆是否静止
    };

    StaticInitImu(StaticOptions options = StaticOptions()) : config_(options) {
    }

    // 添加imu数据
    bool AddImu(const IMU& imu);
    // 添加轮速计信息
    bool AddOdom(const Odom& odom);
    // 静止初始化
    const bool InitSuccess() const {
        return init_success_;
    };

    const Vec3d GetCovGyro() const {
        return cov_gryo_;
    }
    const Vec3d GetGovAcc() const {
        return cov_acc_;
    }

    const Vec3d GetInitBg() const {
        return init_bg_;
    }

    const Vec3d GetInitBa() const {
        return init_ba_;
    }

    const Vec3d GetGravity() const {
        return gravity_;
    }

   private:
    bool TryInit();

   private:
    bool init_success_ = false;
    StaticOptions config_;

    Vec3d cov_gryo_ = Vec3d::Zero();  // gyro测量协方差
    Vec3d cov_acc_ = Vec3d::Zero();   // acc协方差
    Vec3d init_bg_ = Vec3d::Zero();   // gyro零偏
    Vec3d init_ba_ = Vec3d::Zero();   // acc零偏
    Vec3d gravity_ = Vec3d::Zero();   // 重力
    bool is_static_ = false;
    std::deque<IMU> init_imu_queue_;
    double init_start_time_ = 0.0;
    double current_time_ = 0.0;
};
}  // namespace ctlio::slam