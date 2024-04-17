/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-17 00:24:39
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-18 00:09:17
 * @FilePath: /lio_ws/src/ieskf_slam/test/lidar_process_test.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include "common/timer.hh"
#include "globaldefine.hh"
#include "lidar_process/lidar_process.hh"
#include "livox_ros_driver/CustomMsg.h"
#include "tools/lidar_utils.hpp"
#include "tools/tool_color_printf.hpp"

using namespace ctlio::slam;

std::shared_ptr<LidarProcess> lidar_process;

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // copy msg
    sensor_msgs::PointCloud2::Ptr cloud(new sensor_msgs::PointCloud2(*msg));
    std::vector<point3D> cloud_out;
    Timer::Evaluate([&]() { lidar_process->Process(msg, cloud_out); }, "lidar_process");
    LOG(INFO) << "cloud size:" << cloud_out.size();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    ros::init(argc, argv, "lidar_process_test_node");
    ros::NodeHandle nh;
    std::string config_file = std::string(PROJECT_DIR) + "/config/mapping.yaml";
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    lidar_process = std::make_shared<LidarProcess>();
    lidar_process->LoadFromYaml(config_file);
    YAML::Node config_node = YAML::LoadFile(config_file);
    std::string lidar_topic = config_node["common"]["lidar_topic"].as<std::string>();
    // std::string imu_topic = config_node["common"]["imu_topic"].as<std::string>();
    // std::string encoder_topic = config_node["common"]["encoder_topic"].as<std::string>();

    ros::Subscriber lidar_sub = lidar_process->lidar_type_ == LidarProcess::LidarType::AVIA
                                    ? nh.subscribe(lidar_topic, 100, livox_pcl_cbk)
                                    : nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 100, standard_pcl_cbk);

    ros::spin();
    return 0;
}