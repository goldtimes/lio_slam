#pragma once
#include <glog/logging.h>
#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include "utils/lio_utils.hh"

namespace lio {
class MapBuilderRos {
   public:
    MapBuilderRos() = default;
    MapBuilderRos(ros::NodeHandle& nh, tf2_ros::TransformBroadcaster& tf);
    ~MapBuilderRos() = default;

    void run();

   private:
    void imu_callback(const sensor_msgs::Imu& imu_message);
    void livox_callback(const livox_ros_driver::CustomMsgConstPtr& cloud_msg);
    void livox2pcl(const livox_ros_driver::CustomMsg::ConstPtr& livox_cloud, sensors::PointNormalCloud::Ptr& out_cloud);
    bool syncMeasure(std::deque<sensors::IMU>& imu_queue, LivoxData& livox_datas);

    void init_params();
    void init_sub_pub();

   private:
    ros::NodeHandle& nh_;
    tf2_ros::TransformBroadcaster& tf_;
    std::string global_frame_;
    std::string local_frame_;
    std::string body_frame_;

    ros::Subscriber imu_sub_;
    ros::Subscriber cloud_sub_;
    std::string imu_topic_;
    std::string livox_topic_;

    ros::Publisher body_cloud_pub_;
    ros::Publisher local_cloud_pub_;

    // 时间同步对象
    MeasureGroup measure_group_;
    // 处理数据时的锁对象
    std::mutex data_mutex_;
    // 存放imu的数据和存放点云的数据
    std::deque<sensors::IMU> imu_queue_;
    double last_received_imu_timestamp;
    LivoxData livox_datas;
    double last_received_lidar_timestamp;

    // 过滤点
    int filter_num = 1;
    //
    double blind = 1.0;

    // rate
    std::shared_ptr<ros::Rate> local_rate_;
};
}  // namespace lio
