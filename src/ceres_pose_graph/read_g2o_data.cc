/**
 * @brief 读取g2o中的顶点和边信息
 */
#pragma once
#include <fstream>
#include "types.hh"
#ifndef PROJECT_DIR
#define PROJECT_DIR " "

const std::string WORK_SPACE_DIR = PROJECT_DIR;
static bool readG2o(const std::string& file_path, std::vector<Pose3D>& vertexs, std::vector<BinaryEdge>& edges) {
    vertexs.clear();
    edges.clear();

    std::fstream g2o_file(file_path, std::ios::in);
    if (!g2o_file) {
        return false;
    }
    std::string data_type;
    while (g2o_file.good()) {
        g2o_file >> data_type;
        // 读取顶点
        if (data_type == "VERTEX_SE3:QUAT") {
            Pose3D pose;
            size_t idx;
            g2o_file >> idx >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >> pose.q.y() >> pose.q.z() >>
                pose.q.w();
            assert(vertexs.size() == idx);
            vertexs.push_back(pose);
        }
        // 读取边
        else if (data_type == "EDGE_SE3::QUAT") {
            BinaryEdge edge;
            g2o_file >> edge.id_a >> edge.id_b >> edge.constraint_pose.p.x() >> edge.constraint_pose.p.y() >>
                edge.constraint_pose.p.z() >> edge.constraint_pose.q.x() >> edge.constraint_pose.q.y() >>
                edge.constraint_pose.q.z() >> edge.constraint_pose.q.w();
            for (int i = 0; i < 6; ++i) {
                for (int j = i; j < 6; j++) {
                    g2o_file >> edge.information(i, j);
                    if (i != j) {
                        edge.information(j, i) = edge.information(i, j);
                    }
                }
            }
            edges.push_back(edge);
        } else {
            return false;
        }
    }
    return true;
}
#endif