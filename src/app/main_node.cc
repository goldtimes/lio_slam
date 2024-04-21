#include <chrono>
#include <cmath>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <random>

#include <memory>
#include "globaldefine.hh"
#include "lidar_process/lidar_process.hh"
#include "lio/lidar_odom.hh"
#include "tools/tool_color_printf.hpp"

nav_msgs::Path laserOdomPath;
std::shared_ptr<ctlio::slam::LidarOdom> lio_ptr;
std::shared_ptr<ctlio::slam::LidarProcess> lidar_convert_ptr;

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
}

void imu_callback(const sensor_msgs::Imu& msg) {
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    std::string config_file = std::string(PROJECT_DIR) + "/config/mapping.yaml";
    std::cout << ANSI_COLOR_GREEN << "config_file:" << config_file << ANSI_COLOR_RESET << std::endl;

    lio_ptr = std::make_shared<ctlio::slam::LidarOdom>();
    // if (!lio_ptr->init(config_file)) {
    //     return -1;
    // }

    ros::Publisher pub_lidar_cloud = nh.advertise<sensor_msgs::PointCloud2>("lidar", 10);
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("lidar_path", 5);
    ros::Publisher pubLaserOdOM = nh.advertise<nav_msgs::Odometry>("lidar_odom", 100);
    lidar_convert_ptr = std::make_shared<ctlio::slam::LidarProcess>();
    lidar_convert_ptr->LoadFromYaml(config_file);

    auto config_node = YAML::LoadFile(config_file);
    std::string lidar_topic = config_node["common"]["lidar_topic"].as<std::string>();
    std::string imu_topic = config_node["common"]["imu_topic"].as<std::string>();
    ros::Subscriber laser_sub = lidar_convert_ptr->lidar_type_ == ctlio::slam::LidarProcess::LidarType::AVIA
                                    ? nh.subscribe(lidar_topic, 100, livox_pcl_cbk)
                                    : nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 100, standard_pcl_cbk);

    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 500, imu_callback);

    std::thread measurement_process();

    ros::spin();

    return 0;
}