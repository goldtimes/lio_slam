/**
 * @brief ceres 图优化
 */

#pragma once
#include <Eigen/Dense>

/**
 * @brief 位姿的定义
 */
struct Pose3D {
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
};

/**
 * @brief 边的定义
 */
struct BinaryEdge {
    int id_a;                // 顶点A的id
    int id_b;                // 顶点B的id
    Pose3D constraint_pose;  // 顶点之间的约束
    Eigen::Matrix<double, 6, 6> information;
};
