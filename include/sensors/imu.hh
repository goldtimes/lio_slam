#pragma once
#include <memory>
#include <Eigen/Core>

namespace sensors{
    struct IMU{
        IMU() = default;
        IMU(double t, const Eigen::Vector3d& gyro, const Eigen::Vector3d& acc) : gyro_(gyro), acc_(acc){}

        double timestamp_ = 0.0;
        Eigen::Vector3d gyro_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc_ = Eigen::Vector3d::Zero();
    };
}

using IMUPtr = std::shared_ptr<sensors::IMU>;