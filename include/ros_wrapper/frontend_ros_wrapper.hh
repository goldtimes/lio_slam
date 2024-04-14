/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-02 23:35:37
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-13 16:18:28
 * @FilePath: /lio_ws/src/ieskf_slam/include/ros_wrapper/frontend_ros_wrapper.hh
 * @Description: 前端的ros接口
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */

#pragma once
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <memory>
#include "lidar/avia_process.hh"
#include "modules/frontend/frontend.hh"
namespace IESKF_SLAM {

enum LIDAR_TYPE { AVIA, LEISHEN, ROBENSE };
class FrontendRosWrapper {
   public:
    FrontendRosWrapper(ros::NodeHandle& nh);
    ~FrontendRosWrapper() = default;

   private:
    void imu_callback(const sensor_msgs::ImuPtr& imu_msg);
    void cloud_callback(const sensor_msgs::PointCloud2Ptr& cloud_msgs);
    void odom_callback(const nav_msgs::OdometryPtr& odom_msgs);

    void run();

    void publishMsg();

   private:
    // ros sub pub
    ros::Subscriber imu_sub_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher curr_cloud_pub_;
    ros::Publisher path_pub_;
    ros::Publisher local_map_pub;
    // lidar_process
    std::shared_ptr<CommonLidarProcessInterface> lidar_process_ptr_;
    // 前端
    Frontend::Ptr frontend_ptr_;
    // state
    Eigen::Quaterniond curr_q;
    Eigen::Vector3d curr_t;
};

}  // namespace IESKF_SLAM
