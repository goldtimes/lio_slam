#pragma once
#include <glog/logging.h>
#include <lio_slam/SaveMap.h>
#include <livox_ros_driver/CustomMsg.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include <mutex>
#include "lio/iglio_builder.hh"
#include "loopclosure/loopclosure.hh"
#include "timer/tic_toc.hh"
#include "utils/lio_utils.hh"

namespace lio {
class MapBuilderRos {
   public:
    MapBuilderRos() = default;
    MapBuilderRos(ros::NodeHandle& nh, tf2_ros::TransformBroadcaster& tf, std::shared_ptr<LoopSharedData> shared_data);
    ~MapBuilderRos() = default;

    void run();

   private:
    void imu_callback(const sensor_msgs::Imu& imu_message);
    void livox_callback(const livox_ros_driver::CustomMsgConstPtr& cloud_msg);
    void livox2pcl(const livox_ros_driver::CustomMsg::ConstPtr& livox_cloud, sensors::PointNormalCloud::Ptr& out_cloud);
    bool syncMeasure(std::deque<sensors::IMU>& imu_queue, LivoxData& livox_datas);

    void init_params();
    void init_sub_pub();
    void init_service();
    void publishOdom(const nav_msgs::Odometry& odom);
    void publishLocalPath();
    void publishGlobalPath();
    void publishCloud(const ros::Publisher& cloud_pub, const sensor_msgs::PointCloud2& cloud);
    void addKeypose();
    void publishLoopMark();
    bool save_map_callback(lio_slam::SaveMap::Request& req, lio_slam::SaveMap::Response& resp);

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

    ros::Publisher lidar_cloud_pub_;
    ros::Publisher body_cloud_pub_;
    ros::Publisher local_cloud_pub_;
    ros::Publisher odom_pub_;
    ros::Publisher local_path_pub_;
    ros::Publisher global_path_pub_;
    ros::Publisher loop_mark_pub_;
    nav_msgs::Path local_path_;
    nav_msgs::Path global_path_;
    pcl::PCDWriter writer_;

    ros::ServiceServer save_map_server_;

    // 时间同步对象
    MeasureGroup measure_group_;
    // 处理数据时的锁对象
    std::mutex data_mutex_;
    // 存放imu的数据和存放点云的数据
    std::deque<sensors::IMU> imu_queue_;
    double last_received_imu_timestamp;
    LivoxData livox_datas;
    double last_received_lidar_timestamp;
    std::shared_ptr<lio::IGLIOBuilder> lio_buidler_;
    IGLIOParams lio_params_;
    // 过滤点
    int filter_num = 1;
    //
    double blind = 1.0;

    kf::State current_navi_state_;
    double current_time_;

    // rate
    std::shared_ptr<ros::Rate> local_rate_;
    std::shared_ptr<ros::Rate> loop_rate_;

    Timer::TicToc tic_toc;

    // loopclosure
    std::shared_ptr<LoopClosureThread> loop_closure_;
    std::shared_ptr<std::thread> loop_thread_;
    std::shared_ptr<LoopSharedData> loop_shared_data_;
};
}  // namespace lio
