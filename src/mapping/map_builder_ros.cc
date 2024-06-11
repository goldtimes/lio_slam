#include "mapping/map_builder_ros.hh"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"

namespace lio {

MapBuilderRos::MapBuilderRos(ros::NodeHandle& nh, tf2_ros::TransformBroadcaster& tf) : nh_(nh), tf_(tf) {
    local_rate_ = std::make_shared<ros::Rate>(100);
}
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

void MapBuilderRos::livox2pcl(const livox_ros_driver::CustomMsg::ConstPtr& livox_cloud,
                              sensors::PointNormalCloud::Ptr& out_cloud) {
    // 获取点云中的个数
    int point_num = livox_cloud->point_num;
    out_cloud->clear();
    // 预分配空间
    out_cloud->reserve(point_num / (filter_num + 1));
    uint valid_num = 0;
    for (int i = 0; i < point_num; ++i) {
        // 有效的点
        if ((livox_cloud->points[i].line < 4) &&
            ((livox_cloud->points[i].tag & 0x03) == 0x10 || (livox_cloud->points[i].tag & 0x30) == 0x00)) {
            valid_num++;
            if (valid_num % filter_num != 0) {
                continue;
            }
            sensors::PointNormalType p;
            p.x = livox_cloud->points[i].x;
            p.y = livox_cloud->points[i].y;
            p.z = livox_cloud->points[i].z;
            p.intensity = livox_cloud->points[i].reflectivity;
            // 纳秒 / 毫秒的转换
            p.curvature = livox_cloud->points[i].offset_time / 1e6;
            if (p.x * p.x + p.y * p.y + p.z * p.z > (blind * blind)) {
                out_cloud->push_back(p);
            }
        }
    }
}

bool MapBuilderRos::syncMeasure(std::deque<sensors::IMU>& imu_queue, LivoxData& livox_datas) {
    // 1. 数据不为空
    if (imu_queue.empty() || livox_datas.clouds_buff.empty()) {
        return false;
    }
    // 2. push lidar
    if (!measure_group_.lidar_pushed) {
        measure_group_.lidar_cloud = livox_datas.clouds_buff.front();
        measure_group_.lidar_begin_time = livox_datas.time_buffer.front();
        measure_group_.lidar_end_time =
            measure_group_.lidar_begin_time + measure_group_.lidar_cloud->points.back().curvature / 1e3;
        measure_group_.lidar_pushed = true;
    }
    // 3. 时间同步
    if (last_imu_timestamp < measure_group_.lidar_end_time) {
        return false;
    }
    double imu_time = imu_queue.front().timestamp_;
    imu_datas.clear();
    while (!imu_queue.empty() && (imu_time < measure_group_.lidar_end_time)) {
        imu_time = imu_queue.front().timestamp_;
        if (imu_time > measure_group_.lidar_end_time) {
            break;
        }
        imu_datas.push_back(imu_queue.front());
        imu_queue.pop_front();
    }
    livox_datas.clouds_buff.pop_front();
    livox_datas.time_buffer.pop_front();
    measure_group_.lidar_pushed = false;
    measure_group_.imudatas.clear();
    measure_group_.imudatas.insert(measure_group_.imudatas.end(), imu_datas.begin(), imu_datas.end());
    return true;
}

void MapBuilderRos::run() {
    while (ros::ok) {
        local_rate_->sleep();
        ros::spinOnce();
        if (!syncMeasure(imu_datas, livox_datas)) {
            continue;
        }
    }
    LOG(INFO) << "map builder thread exits";
}

}  // namespace lio