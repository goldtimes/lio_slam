#pragma once
#include <Eigen/Core>
#include <memory>

namespace sensors {
struct IMU {
    IMU() = default;
    IMU(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc) : timestamp_(t), gyro_(gyro), acc_(acc) {
    }

    double timestamp_ = 0.0;
    Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_ = Eigen::Vector3d::Zero();
};
}  // namespace sensors

using IMUPtr = std::shared_ptr<sensors::IMU>;