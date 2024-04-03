#pragma once
#include "modules/module_base.hh"

#include <deque>
#include "type/imu.hh"
#include "type/pointcloud.hh"
#include "type/pose.hh"
#include "utils/lidar_utils.hh"

namespace IESKF_SLAM {
class Frontend : public ModuleBase {
   public:
    using Ptr = std::shared_ptr<Frontend>;

   public:
    Frontend(const std::string& config_file_path, const std::string& prefix);
    ~Frontend() = default;

    // 传入数据给前端
    void AddIMU(const IMU& imu_data);
    void AddCloud(const PointCloud& cloud);
    void AddPose(const Pose& pose);

    bool track();

    const PCLPointCloud& getCurrentCloud() const {
        return current_pointcloud_;
    }

   private:
    std::deque<IMU> imu_queue_;
    std::deque<PointCloud> clouds_queue_;
    std::deque<Pose> pose_queue_;
    PCLPointCloud current_pointcloud_;
};
}  // namespace IESKF_SLAM