#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <csignal>
#include <mutex>
#include <thread>
#include "sensors/point_types.hh"
// gtsam
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
// pcl
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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

struct LoopParms {
    // 回环的子图距离和时间设置
    double rad_thresh = 0.4;
    double dis_thresh = 2.5;
    double time_thresh = 30.0;
    double loop_pose_search_radius = 10.0;
    int loop_pose_min_size_thresh = 5;
    double submap_resolution = 0.2;
    int submap_search_num = 20.0;
    double loop_icp_thresh = 0.3;
    bool active = false;
    bool z_prior = false;
};

// keyframe的姿态
struct Pose6D {
    int index;
    double time;
    Eigen::Matrix3d local_rot;
    Eigen::Vector3d local_pos;
    Eigen::Matrix3d global_rot;
    Eigen::Vector3d global_pos;
    Pose6D(int id, double t, const Eigen::Matrix3d& local_r, const Eigen::Vector3d& local_p)
        : index(id), time(t), local_rot(local_r), local_pos(local_p) {
    }

    void setGlobalPose(const Eigen::Matrix3d& global_r, const Eigen::Vector3d& global_p) {
        global_rot = global_r;
        global_pos = global_p;
    }

    void addOffset(const Eigen::Matrix3d& offset_rot, const Eigen::Vector3d& offset_pos) {
        global_rot = offset_rot * local_rot;
        global_pos = offset_rot * local_pos + offset_pos;
    }
    void getOffset(Eigen::Matrix3d& offset_rot, Eigen::Vector3d& offset_pos) {
        offset_rot = global_rot * local_rot.transpose();
        offset_pos = global_pos - global_rot * local_rot.transpose() * local_pos;
    }
};

struct LoopSharedData {
    bool keypose_add = false;
    std::mutex mutex;
    Eigen::Matrix3d offset_rot = Eigen::Matrix3d::Identity();
    Eigen::Vector3d offset_trans = Eigen::Vector3d::Zero();
    std::vector<Pose6D> keyposes;
    std::vector<LoopPair> loop_pairs;
    std::vector<std::pair<int, int>> loop_history;
    std::vector<sensors::PointNormalCloud::Ptr> cloud_history;
};

class ZaxisPriorFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
   public:
    ZaxisPriorFactor(gtsam::Key key, const gtsam::SharedNoiseModel& noise, double z)
        : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise, key), z_prior_(z) {
    }
    virtual ~ZaxisPriorFactor() {
    }

    virtual gtsam::Vector evaluateError(const gtsam::Pose3& pose, boost::optional<gtsam::Matrix&>& H) const {
        double z_meas = pose.translation()(2);
        if (H) {
            gtsam::Matrix jac = gtsam::Matrix::Zero(1, 6);
            jac << 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
            (*H) = jac;
        }
        return gtsam::Vector1(z_meas - z_prior_);
    }

   protected:
    double z_prior_;
};

class LoopClosureThread {
   public:
    void init();
    void setSharedData(std::shared_ptr<LoopSharedData> share_data_ptr);
    void setLoopRate(std::shared_ptr<ros::Rate> loop_rate);
    LoopParms& getLoopParams() {
        return loop_params_;
    }
    sensors::PointNormalCloud::Ptr getSubmaps(std::vector<Pose6D>& poses_list,
                                              std::vector<sensors::PointNormalCloud::Ptr>& cloud_list, int index,
                                              int search_num);
    void operator()();

   private:
    void loopCheck();
    void addOdomFactor();
    void addLoopFactor();
    void smoothAndUpdate();

   private:
    LoopParms loop_params_;
    std::shared_ptr<LoopSharedData> shared_data_;
    std::shared_ptr<ros::Rate> loop_rate_;
    std::vector<Pose6D> tmp_keypose;

    int previous_index = 0;
    int lastest_index = -1;
    bool loop_found = false;

    gtsam::Values initialized_estimate;
    gtsam::Values optimized_estimate;

    std::shared_ptr<gtsam::ISAM2> isam2;
    gtsam::NonlinearFactorGraph gtsam_graph;

    // 将keypose的位姿看做点云存放起来
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_histories_pose;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_histories_pose;

    pcl::VoxelGrid<sensors::PointNormalType>::Ptr submap_filter;
    pcl::IterativeClosestPoint<sensors::PointNormalType, sensors::PointNormalType>::Ptr icp;
};
}  // namespace lio