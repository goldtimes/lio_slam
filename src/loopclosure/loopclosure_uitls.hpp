#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include "sensors/point_types.hh"

namespace lio {
/**@
 * @brief 记录回环信息的
 */
struct LoopPair {
    LoopPair(int p_id, int c_id, double s, const Eigen::Matrix3d df_rot, const Eigen::Vector3d& df_vec)
        : pre_idx(p_id), cur_idx(c_id), score(s), diff_rot(df_rot), diff_vec(df_vec) {
    }
    int pre_idx;
    int cur_idx;
    Eigen::Matrix3d diff_rot;
    Eigen::Vector3d diff_vec;
    double score;
};

struct Pose6D {};

struct SharedData {
    bool keypose_add = false;
    std::vector<Pose6D> keyposes;
    std::vector<LoopPair> loop_pairs;
    std::vector<std::pair<int, int>> loop_history;
    std::vector<sensors::PointNormalCloud::Ptr> cloud_history;
};

class LoopClosureThread {
   public:
   private:
};
}  // namespace lio