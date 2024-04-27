#pragma once
#include <sys/timer.h>
#include <sys/vtimes.h>
#include <yaml-cpp/yaml.h>
#include <condition_variable>
#include <functional>
#include <memory>
#include <string>
#include "common/eigen_types.hh"
#include "common/timer.hh"
#include "lidar_factor/analytical_factor.hh"
#include "lidar_factor/lidar_factor.hh"
#include "lidar_factor/pose_parameterization.hh"
#include "sensors/point_types.hh"
#include "state/eskf.hpp"
#include "state/static_imu_init.hh"
#include "tools/tool_color_printf.hpp"
#include "voxel/voxel_map.hpp"
namespace ctlio::slam {
struct LioOptions {
    double surf_res;
    bool log_print;
    int max_num_iteration;
    //   ct_icp
    IcpModel icpmodel;
    double size_voxel_map;
    double min_distance_points;
    int max_num_points_in_voxel;
    double max_distance;
    double weight_alpha;
    double weight_neighborhood;
    double max_dist_to_plane_icp;
    int init_num_frames;
    int voxel_neighborhood;
    int max_number_neighbors;
    int threshold_voxel_occupancy;
    bool estimate_normal_from_neighborhood;
    int min_number_neighbors;
    double power_planarity;
    int num_closest_neighbors;

    double sampling_rate;

    double ratio_of_nonground;
    int max_num_residuals;
    int min_num_residuals;

    MotionCompensation motion_compensation;  // 运动模型
    bool point_to_plane_with_distortion = false;
    double beta_location_consistency;     // Constraints on location
    double beta_constant_velocity;        // Constraint on velocity
    double beta_small_velocity;           // Constraint on the relative motion
    double beta_orientation_consistency;  // Constraint on the orientation consistency

    double thres_orientation_norm;
    double thres_translation_norm;
};
class LidarOdom {
   public:
    LidarOdom();
    ~LidarOdom();

    bool init(const std::string& config_file);
    void pushData(const std::vector<point3D>& points, std::pair<double, double> data);
    void pushData(IMUPtr imu);

    void run();

    int getIndex();

    void setFunc(std::function<bool(std::string& topic_name, CloudPtr& cloud, double time)>& func);
    void setFunc(std::function<bool(std::string& topic_name, SE3& pose, double time)>& func);
    void setFunc(std::function<bool(std::string& topic_name, double time1, double time2)>& func);

   private:
    void loadOptions();
    void TryInitIMU();

    void Predict();
    void Undistort(std::vector<point3D>& points);
    void ProcessMeasurements(MeasureGroup& means);

    void stateInitialization();
    CloudFrame* buildFrame(std::vector<point3D>& const_surf, State* cur_state, double timestamped_begin,
                           double timestamped_end);

    void poseEstimation(CloudFrame* p_frame);
    void optimize(CloudFrame* p_frame);

    void lasermap_fov_segment();
    void map_incremental(CloudFrame* p_frame, int min_num_points = 0);

    void addPointToMap(VoxelHashMap& map, const Eigen::Vector3d& point, const double& intensity, double voxel_size,
                       int max_num_points_in_voxel, double min_distance_points, int min_num_points,
                       CloudFrame* p_frame);
    void addSurfCostFactor(std::vector<ceres::CostFunction*>& surf, std::vector<Eigen::Vector3d>& normals,
                           std::vector<point3D>& keypoints, const CloudFrame* p_frame);
    void addPointToPCL(CloudPtr pcl_points, const Eigen::Vector3d& point, const double& intensity, CloudFrame* p_frame);

    double checkLocalizability(std::vector<Eigen::Vector3d> planeNormals);

    // search neighbors
    Neighborhood computeNeighborhoodDistribution(
        const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points);

    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> searchNeighbors(
        const VoxelHashMap& map, const Eigen::Vector3d& point, int nb_voxels_visited, double size_voxel_map,
        int max_num_neighbors, int threshold_voxel_capacity = 1, std::vector<Voxel>* voxels = nullptr);

    inline Sophus::SO3 r2SO3(const Eigen::Vector3d r) {
        return Sophus::SO3::exp(r);
    }

    std::vector<MeasureGroup> getMeasureMents();

   private:
    std::string config_yaml_;
    std::shared_ptr<StaticInitImu> imu_init_;
    SE3 T_LtoI_;  // lidar->imu的外参
    MeasureGroup measure_;
    int index_frame = 1;
    LioOptions options_;
    VoxelHashMap voxel_map;
    Eigen::Matrix3d R_LtoI = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_LtoI = Eigen::Vector3d::Zero();
    bool imu_need_init_;

    double lidar_point_cov = 0.001;
    double PR_begin[6] = {0, 0, 0, 0, 0, 0};  //  p         r
    double PR_end[6] = {0, 0, 0, 0, 0, 0};    //  p         r
    // 滤波器
    ESKFD eskf_;
    std::vector<NavState> imu_states_;
    IMUPtr last_imu_ = nullptr;
    double time_curr;
    double delay_time;
    Vec3d g_{0, 0, -9.8};

    // 数据存储
    std::deque<std::vector<point3D>> lidar_buffer_;
    std::deque<IMUPtr> imu_buffer_;
    double last_timestamped_imu_ = -1.0;
    double last_timestamped_lidar_ = 0.0;
    std::deque<std::pair<double, double>> time_buffer_;

    // 锁
    std::mutex mtx_buffer;
    std::mutex mtx_state;
    std::condition_variable cond;

    State* current_state;
    std::vector<CloudFrame*> all_cloud_frame;
    std::vector<State*> all_state_frame;
    std::function<bool(std::string& topic_name, CloudPtr& cloud, double time)> pub_cloud_to_ros;
    std::function<bool(std::string& topic_name, SE3& pose, double time)> pub_pose_to_ros;
    std::function<bool(std::string& topic_name, double time1, double time2)> pub_data_to_ros;
    CloudPtr points_world;
};
}  // namespace ctlio::slam