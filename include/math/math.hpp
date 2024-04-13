/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-04 00:01:12
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-13 13:15:11
 * @FilePath: /lio_ws/src/ieskf_slam/include/math/math.hpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>

namespace IESKF_SLAM {
static inline Eigen::Matrix4d compositeTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    Eigen::Matrix4d tf = Eigen::Matrix4d::Identity();
    tf.block<3, 3>(0, 0) = q.toRotationMatrix();
    tf.block<3, 1>(0, 3) = t;
    return tf;
}
}  // namespace IESKF_SLAM