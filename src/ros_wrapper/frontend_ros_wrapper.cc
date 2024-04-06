/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 00:03:01
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-06 22:58:56
 * @FilePath: /lio_ws/src/ieskf_slam/src/ros_wrapper/frontend_ros_wrapper.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "ros_wrapper/frontend_ros_wrapper.hh"
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <pcl/common/transforms.h>
#include "globaldefine.hh"

namespace IESKF_SLAM {
FrontendRosWrapper::FrontendRosWrapper(ros::NodeHandle& nh) {
    std::string config_file_name;
    std::string lidar_topic;
    std::string imu_topic;
    int lidar_type = 0;

    nh.param<std::string>("ros/config_file_name", config_file_name, "");
    nh.param<std::string>("ros/lidar_topic", lidar_topic, "/lidar");
    nh.param<std::string>("ros/imu_topic", imu_topic, "/imu");
    nh.param<int>("ros/lidar_type", lidar_type, LIDAR_TYPE::AVIA);
    ROS_INFO("lidar_topic:{%s}", lidar_topic.c_str());
    ROS_INFO("imu_topic:{%s}", imu_topic.c_str());
    frontend_ptr_ = std::make_shared<Frontend>(CONFIG_DIR + config_file_name, "front_end");

    // 发布订阅
    cloud_sub_ = nh.subscribe(lidar_topic, 100, &FrontendRosWrapper::cloud_callback, this);
    imu_sub_ = nh.subscribe(imu_topic, 1000, &FrontendRosWrapper::imu_callback, this);
    odom_sub_ = nh.subscribe("/odometry", 100, &FrontendRosWrapper::odom_callback, this);

    // 雷达类型
    if (lidar_type == LIDAR_TYPE::AVIA) {
        lidar_process_ptr_ = std::make_shared<AviaProcess>();
    } else {
        ROS_ERROR("unsupport lidar type");
        exit(100);
    }
    curr_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("curr_cloud", 100);
    path_pub_ = nh.advertise<nav_msgs::Path>("path", 100);
    run();
}

void FrontendRosWrapper::run() {
    ros::Rate rate(500);
    while (ros::ok()) {
        rate.sleep();
        ros::spinOnce();
        if (frontend_ptr_->track()) {
            publishMsg();
        }
    }
}

/**
 * @brief 发布世界坐标系下的当前点云
 */
void FrontendRosWrapper::publishMsg() {
    // auto pcl_currn_cloud = frontend_ptr_->getCurrentCloud();
    // sensor_msgs::PointCloud2 ros_cloud;
    // pcl::toROSMsg(pcl_currn_cloud, ros_cloud);
    // ros_cloud.header.frame_id = "map";
    // curr_cloud_pub_.publish(ros_cloud);
    static nav_msgs::Path path;
    IESKF::State18d state = frontend_ptr_->readState();
    path.header.frame_id = "map";
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = state.position.x();
    pose.pose.position.y = state.position.y();
    pose.pose.position.z = state.position.z();
    // ROS_INFO("x:%f,y:%f,z:%f", state.position.x(), state.position.y(), state.position.z());
    path.poses.push_back(pose);
    path_pub_.publish(path);
}

void FrontendRosWrapper::imu_callback(const sensor_msgs::ImuPtr& imu_msg) {
    IMU imu;
    imu.time_stamp.fromNsec(imu_msg->header.stamp.toNSec());
    imu.acc = {imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z};
    imu.gyro = {imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z};
    frontend_ptr_->AddIMU(imu);
}
void FrontendRosWrapper::cloud_callback(const sensor_msgs::PointCloud2Ptr& cloud_msgs) {
    PointCloud cloud;
    lidar_process_ptr_->process(*cloud_msgs, cloud);
    frontend_ptr_->AddCloud(cloud);
}
void FrontendRosWrapper::odom_callback(const nav_msgs::OdometryPtr& odom_msgs) {
    Pose pose;
    pose.time_stamp.fromNsec(odom_msgs->header.stamp.toNSec());
    pose.position = {odom_msgs->pose.pose.position.x, odom_msgs->pose.pose.position.y, odom_msgs->pose.pose.position.z};
    pose.rotation.w() = odom_msgs->pose.pose.orientation.w;
    pose.rotation.x() = odom_msgs->pose.pose.orientation.x;
    pose.rotation.y() = odom_msgs->pose.pose.orientation.y;
    pose.rotation.z() = odom_msgs->pose.pose.orientation.z;
    frontend_ptr_->AddPose(pose);
}

}  // namespace IESKF_SLAM