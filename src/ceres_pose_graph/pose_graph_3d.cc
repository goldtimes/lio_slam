/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-13 00:32:36
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-13 01:03:26
 * @FilePath: /lio_ws/src/ieskf_slam/src/ceres_pose_graph/pose_graph_3d.cc
 * @Description: }
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include <iostream>
#include "pose_graph_3d_err_term.hh"
#include "read_g2o_data.hpp"

std::vector<Pose3D> vertexs;
std::vector<BinaryEdge> edges;

void outputPose(const std::string& path) {
    std::fstream file(path, std::ios::out);
    if (!file) {
        std::cerr << "error open file" << std::endl;
    }
    for (int i = 0; i < vertexs.size(); ++i) {
        file << i << " " << vertexs[i].p.transpose() << " " << vertexs[i].q.x() << " " << vertexs[i].q.y() << " "
             << vertexs[i].q.z() << " " << vertexs[i].q.w() << std::endl;
    }
    file.close();
}

int main(int argc, char** argv) {
    std::cout << "WORK DIR: " << WORK_SPACE_DIR << std::endl;
    if (readG2O(WORK_SPACE_DIR + "ceres_pose_graph/" + "sphere_data.g2o.txt", vertexs, edges)) {
        std::cout << "READ G2O FILE COMPLETED!!" << std::endl;
        std::cout << "Vertexs Size: " << vertexs.size() << std::endl;
        std::cout << "Constraints Size: " << edges.size() << std::endl;
        std::cout << "WORK DIR: " << WORK_SPACE_DIR << std::endl;
    }
    // 输出原始的pose
    outputPose(WORK_SPACE_DIR + "ceres_pose_graph/" + "init_pose.txt");

    ceres::Problem* problem = new ceres::Problem();
    ceres::LossFunction* loss = nullptr;
    // 四元素的流形，指定四元素的求导方式
    // ceres::EigenQuaternionParameterization* quat_manifold = new ceres::EigenQuaternionParameterization();
    ceres::Manifold* quat_manifold = new ceres::EigenQuaternionManifold();
    for (const auto& edge : edges) {
        // 对信息矩阵开方
        Eigen::Matrix<double, 6, 6> sqrt_info = edge.information.llt().matrixL();
        // 构建误差
        ceres::CostFunction* cost_function = PoseGraph3dErrorTerm::Creat(edge.constraint_pose, sqrt_info);
        // 添加残差
        // clang-format off
        problem->AddResidualBlock(cost_function, 
                                   loss, 
                                   vertexs[edge.id_a].p.data(),
                                   vertexs[edge.id_a].q.coeffs().data(), 
                                   vertexs[edge.id_b].p.data(),
                                   vertexs[edge.id_b].q.coeffs().data());
        problem->SetManifold(vertexs[edge.id_a].q.coeffs().data(),quat_manifold);
        problem->SetManifold(vertexs[edge.id_b].q.coeffs().data(),quat_manifold);
    }
    // clang-format on
    // 对于第一个顶点，我们固定
    problem->SetParameterBlockConstant(vertexs.front().q.coeffs().data());
    problem->SetParameterBlockConstant(vertexs.front().p.data());
    // 配置求解参数
    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    // 线性求解方式：考虑一下这几种求解方式在应用场景和效率上各有什么不同？
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    ceres::Solver::Summary summary;
    // 求解
    ceres::Solve(options, problem, &summary);
    // 输出求解结果
    // std::cout << summary.FullReport() << std::endl;
    //* 保存结果
    outputPose(WORK_SPACE_DIR + "ceres_pose_graph/" + "out_pose.txt");
    return 0;
}