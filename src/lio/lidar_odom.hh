#pragma once
#include <sys/timer.h>
#include <sys/vtimes.h>
#include <condition_variable>
#include <functional>
#include <string>
#include "common/eigen_types.hh"
#include "common/timer.hh"
#include "lidar_factor/lidar_factor.hh"
#include "lidar_factor/pose_parameterization.hh"
#include "sensors/point_types.hh"
#include "state/eskf.hpp"
#include "state/static_imu_init.hh"
#include "tools/tool_color_printf.hpp"
#include "voxel/voxel_map.hpp"
namespace ctlio::slam {
struct LioOptions {};
class LidarOdom {
   public:
    LidarOdom();
    ~LidarOdom();

    bool init(const std::string& config_file);
    void pushData(std::vector<point3D>& points, std::pair<double, double> data);
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

   private:
    std::vector<MeasureGroup> getMeasureMents();
};
}  // namespace ctlio::slam