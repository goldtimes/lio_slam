#pragma once
#include <memory>
#include "common/eigen_types.hh"

namespace ctlio::slam {
struct IMU {
    IMU() = default;
    IMU(double t, const Vec3d& gyro, const Vec3d acce) : timestamped_(t), gyro_(gyro), acce_(acce) {
    }

    double timestamped_ = 0.0;
    Vec3d gyro_ = Eigen::Vector3d::Zero();
    Vec3d acce_ = Vec3d::Zero();
};

using IMUPtr = std::shared_ptr<IMU>;
}  // namespace ctlio::slam