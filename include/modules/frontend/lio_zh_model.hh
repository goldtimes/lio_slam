/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-08 23:40:55
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-09 00:54:57
 * @FilePath: /lio_ws/src/ieskf_slam/include/modules/frontend/lio_zh_model.hh
 * @Description:
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
// #include "math/geometry.hh"
#include <omp.h>
#include <memory>
#include <vector>
#include "ieskf/ieskf.hh"
#include "math/SO3.hpp"
#include "utils/lidar_utils.hh"

namespace IESKF_SLAM {
/**
 * 紧耦合计算H矩阵和残差矩阵
 **/
class LIOZHModel {
   public:
    template <typename _first, typename _second, typename _third>
    struct triple {
        _first point_body;
        _second plane_norm;
        _third pd;
    };
    using Ptr = std::shared_ptr<LIOZHModel>;

    void prepare(KDTreeConstPtr map_kdtree, PCLPointCloudPtr current_cloud, PCLPointCloudPtr local_map) {
        global_map_kdtree_ptr = map_kdtree;
        current_scan_ptr = current_cloud;
        local_map_ptr = local_map;
    }

    /**
     * @brief scan_to_submap, 计算jacobian矩阵和观测残差
     */
    bool calculate(const IESKF::State18d& state, Eigen::MatrixXd& H, Eigen::MatrixXd& Z) {
        std::vector<match_info> matchs;
        matchs.resize(current_scan_ptr->size());
        std::vector<bool> effect_points(current_scan_ptr->size(), false);
        std::vector<match_info> match_real;

        int valid_points_num = 0;
// clang-format off
        #ifdef MP_EN;
            omp_set_num_threads(4);
            #pragma omp parallel for
        #endif
        // clang-format on
        // 遍历所有点
        for (size_t i = 0; i < current_scan_ptr->size(); ++i) {
            Point point_body = current_scan_ptr->points[i];
            Point point_world = transformPoint(point_body, state.rotation, state.position);
            // kdtree 搜索
            std::vector<int> indexs;
            std::vector<float> dist;
            global_map_kdtree_ptr->nearestKSearch(point_world, NEAR_POINTS_NUM, indexs, dist);
            // 没有找到5个点，或者5个点最远的距离大于1m,放弃该点
            if (indexs.size() < 5 || dist[indexs.size() - 1] > 1.0) {
                continue;
            }
            // 平面拟合
            std::vector<Point> target_points;
            for (size_t i = 0; i < indexs.size(); ++i) {
                target_points.push_back(local_map_ptr->points[indexs[i]]);
            }
            Eigen::Vector4d plane_norm;
            // 拟合平面成功
            if (FitPlane(target_points, plane_norm, 0.1)) {
                double point_to_plan_dist = point_world.x * plane_norm(0) + point_world.y * plane_norm(1) +
                                            point_world.z * plane_norm(2) + plane_norm(3);
                //
                match_info match_plane;
                match_plane.point_body = {point_body.x, point_body.y, point_body.z};
                match_plane.plane_norm = plane_norm.head<3>(0);
                match_plane.pd = point_to_plan_dist;
                double s = 1.0 - 0.9 * fabs(point_to_plan_dist) / sqrt(match_plane.point_body.norm());
                if (s > 0.9) {
                    valid_points_num++;
                    effect_points[i] = true;
                    matchs.push_back(match_plane);
                }
            }
        }
        for (size_t i = 0; i < current_scan_ptr->size(); i++) {
            if (effect_points[i]) match_real.push_back(matchs[i]);
        }
        valid_points_num = effect_points.size();
        H = Eigen::MatrixXd::Zero(valid_points_num, 18);
        Z.resize(valid_points_num, 1);
        for (int i = 0; i < valid_points_num; ++i) {
            // de / dr , de / dt
            Eigen::Vector3d de_dr = -1 * match_real[i].plane_norm.transpose() * state.rotation.toRotationMatrix() *
                                    skew(match_real[i].point_body);
            H.block<1, 3>(i, 0) = de_dr.transpose();
            H.block<1, 3>(i, 3) = match_real[i].plane_norm.transpose();

            Z(i, 0) = match_real[i].pd;
        }
        return true;
    }

   private:
    const int NEAR_POINTS_NUM = 5;
    using match_info = triple<Eigen::Vector3d, Eigen::Vector3d, double>;
    PCLPointCloudPtr current_scan_ptr;
    PCLPointCloudPtr local_map_ptr;
    KDTreeConstPtr global_map_kdtree_ptr;
};
}  // namespace IESKF_SLAM
