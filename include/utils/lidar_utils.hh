/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-01 00:24:30
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-13 16:09:11
 * @FilePath: /lio_ws/src/ieskf_slam/include/utils/lidar_utils.hh
 * @Description: pcl的一些工具类
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */

#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "type/pointcloud.hh"

namespace IESKF_SLAM {
using VoxelFilter = pcl::VoxelGrid<Point>;
using KDTree = pcl::KdTreeFLANN<Point>;
using KDTreePtr = KDTree::Ptr;
using KDTreeConstPtr = KDTree::ConstPtr;
// 重力常量
const double GRAVITY = 9.81;

template <typename PointType, typename T>
static PointType transformPoint(const PointType& point_body, const Eigen::Quaternion<T>& q,
                                const Eigen::Matrix<T, 3, 1>& t) {
    PointType point_world;
    Eigen::Matrix<T, 3, 1> point_vec = {point_body.x, point_body.y, point_body.z};
    auto point_proj = q * point_vec + t;
    point_world.x = point_proj.x();
    point_world.y = point_proj.y();
    point_world.z = point_proj.z();
    return point_world;
}

template <typename PointType>
static bool FitPlane(const std::vector<PointType> points, Eigen::Vector4d& plane_norm, float plane_threshold) {
    Eigen::Vector3d norm;
    Eigen::MatrixXd A;
    Eigen::VectorXd B;
    int points_num = points.size();
    A.resize(points_num, 3);
    B.resize(points_num);
    B.setOnes();
    B = -1 * B;
    for (int i = 0; i < points_num; ++i) {
        A(i, 0) = points[i].x;
        A(i, 1) = points[i].y;
        A(i, 2) = points[i].z;
    }
    norm = A.colPivHouseholderQr().solve(B);
    // check点到拟合的平面距离
    for (int i = 0; i < points_num; ++i) {
        if (fabs(norm(0) * points[i].x + norm(1) * points[i].y + norm(2) * points[i].z + 1.0f) > plane_threshold) {
            return false;
        }
    }

    double normal = norm.norm();
    norm.normalize();
    plane_norm(0) = norm(0);
    plane_norm(1) = norm(1);
    plane_norm(2) = norm(2);
    plane_norm(3) = 1 / normal;
    return true;
}
}  // namespace IESKF_SLAM
