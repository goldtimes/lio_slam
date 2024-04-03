#include "modules/frontend/frontend.hh"
#include <pcl/common/transforms.h>
#include <ros/ros.h>

namespace IESKF_SLAM {
Frontend::Frontend(const std::string& config_file_path, const std::string& prefix)
    : ModuleBase(config_file_path, prefix, "Frontend Modules") {
}

void Frontend::AddIMU(const IMU& imu) {
    imu_queue_.push_back(imu);
}

void Frontend::AddCloud(const PointCloud& cloud) {
    clouds_queue_.push_back(cloud);
}

void Frontend::AddPose(const Pose& pose) {
    pose_queue_.push_back(pose);
}

/**
 * @brief 根据pose找到对应时刻的点云,纯展示的过程
 */
bool Frontend::track() {
    // 都为空
    if (pose_queue_.empty() || clouds_queue_.empty()) {
        return false;
    }
    // 将早于cloud_queue_.front()时刻的pose删除
    while (!pose_queue_.empty() && pose_queue_.front().time_stamp.nsec() < clouds_queue_.front().time_stamp.nsec()) {
        ROS_INFO("delete pose time:{%d}", pose_queue_.front().time_stamp.nsec());
        pose_queue_.pop_front();
    }

    if (pose_queue_.empty()) {
        return false;
    }

    while (!clouds_queue_.empty() && clouds_queue_.front().time_stamp.nsec() < pose_queue_.front().time_stamp.nsec()) {
        clouds_queue_.pop_front();
    }
    if (clouds_queue_.empty()) {
        return false;
    }
    // 滤波
    VoxelFilter filter;
    filter.setLeafSize(0.5, 0.5, 0.5);
    auto cloud = clouds_queue_.front().cloud_ptr;
    filter.setInputCloud(cloud);
    filter.filter(*cloud);

    // 将odom变化转矩阵
    Eigen::Matrix4f tf = Eigen::Matrix4f::Identity();
    tf.block<3, 3>(0, 0) = pose_queue_.front().rotation.toRotationMatrix().cast<float>();
    tf.block<3, 1>(0, 3) = pose_queue_.front().position.cast<float>();
    pcl::transformPointCloud(*cloud, current_pointcloud_, tf);

    clouds_queue_.pop_front();
    pose_queue_.pop_front();
    return true;
}

}  // namespace IESKF_SLAM
