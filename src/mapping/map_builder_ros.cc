#include "mapping/map_builder_ros.hh"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"

namespace lio {
void MapBuilderRos::imu_callback(const sensor_msgs::Imu& imu_message) {
    // 加锁
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    Eigen::Vector3d acc(imu_message.linear_acceleration.x, imu_message.linear_acceleration.y,
                        imu_message.linear_acceleration.z);
    Eigen::Vector3d gyro(imu_message.angular_velocity.x, imu_message.angular_velocity.y,
                         imu_message.angular_velocity.z);
    double imu_timestamp = imu_message.header.stamp.toSec();
    sensors::IMU imu(imu_timestamp, acc, gyro);
    if (imu_timestamp < last_imu_timestamp) {
        ROS_WARN("imu loop back, clear buffer, last_timestamp: %f  current_timestamp: %f", last_imu_timestamp,
                 imu_timestamp);
        imu_datas.clear();
    }
    last_imu_timestamp = imu_timestamp;
    imu_datas.push_back(imu);
}

void MapBuilderRos::livox_callback(const livox_ros_driver::CustomMsgConstPtr& livox_cloud_msg) {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    double lidar_timestamp = livox_cloud_msg->header.stamp.toSec();
    if (lidar_timestamp < last_lidar_timestamp) {
        ROS_WARN("lidar loop back, clear buffer, last_timestamp: %f  current_timestamp: %f", last_lidar_timestamp,
                 lidar_timestamp);
        livox_datas.clouds_buff.clear();
        livox_datas.time_buffer.clear();
    }
    last_lidar_timestamp = lidar_timestamp;
    sensors::PointNormalCloud::Ptr cloud(new sensors::PointNormalCloud());
    livox2pcl(livox_cloud_msg, cloud);
    livox_datas.clouds_buff.push_back(cloud);
    livox_datas.time_buffer.push_back(lidar_timestamp);
}

void MapBuilderRos::run() {
}

}  // namespace lio