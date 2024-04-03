#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>

namespace IESKF_SLAM {
Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf.block<3, 3>(0, 0) = q.toRotationMatrix();
    tf.block<3, 1>(0, 3) = t;
    return tf;
}
}  // namespace IESKF_SLAM