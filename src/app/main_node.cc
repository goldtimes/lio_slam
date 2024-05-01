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
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <std_msgs/Float32.h>
#include <yaml-cpp/yaml.h>
#include <random>

#include <memory>
#include "globaldefine.hh"
#include "lidar_process/lidar_process.hh"
#include "lio/lidar_odom.hh"
#include "sensors/imu.hh"
#include "tools/tool_color_printf.hpp"

nav_msgs::Path laserOdomPath;
std::shared_ptr<ctlio::slam::LidarOdom> lio_ptr;
std::shared_ptr<ctlio::slam::LidarProcess> lidar_convert_ptr;
tf::TransformBroadcaster tf_buff;

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // sensor_msgs::PointCloud2::ConstPtr cloud(new sensor_msgs::PointCloud2(*msg));
    static int c = 0;
    std::vector<ctlio::slam::point3D> cloud_out;
    ctlio::slam::Timer::Evaluate([&]() { lidar_convert_ptr->Process(msg, cloud_out); }, "laser_conver");

    ctlio::slam::Timer::Evaluate(
        [&]() {
            // 如果frame > 20帧,降采样size为0.05
            double sample_size = lio_ptr->getIndex() < 20 ? 0.01 : 0.05;
            std::mt19937_64 g;
            // 打乱cloud
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
            // cloud to voxel frame
            subSampleFrame(cloud_out, sample_size);
            // 再打乱cloud
            std::shuffle(cloud_out.begin(), cloud_out.end(), g);
        },
        "laser downsample");
    lio_ptr->pushData(cloud_out, std::make_pair(msg->header.stamp.toSec(), lidar_convert_ptr->GetTimeSpan()));
    c++;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    sensor_msgs::Imu::Ptr imu_ptr(new sensor_msgs::Imu(*msg));
    ctlio::slam::IMUPtr imu = std::make_shared<ctlio::slam::IMU>(
        imu_ptr->header.stamp.toSec(),
        Vec3d(imu_ptr->angular_velocity.x, imu_ptr->angular_velocity.y, imu_ptr->angular_velocity.z),
        Vec3d(imu_ptr->linear_acceleration.x, imu_ptr->linear_acceleration.y, imu_ptr->linear_acceleration.z));
    lio_ptr->pushData(imu);
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
    if (!lio_ptr->init(config_file)) {
        return -1;
    }

    ros::Publisher pub_lidar_cloud = nh.advertise<sensor_msgs::PointCloud2>("scan", 10);
    // function传入bool(std::string & topic_name, ctlio::slam::CloudPtr & cloud, double time)的函数
    auto cloud_pub_func = std::function<bool(std::string & topic_name, ctlio::slam::CloudPtr & cloud, double time)>(
        [&](std::string& topic_name, ctlio::slam::CloudPtr& cloud, double time) -> bool {
            sensor_msgs::PointCloud2Ptr cloud_out_ptr(new sensor_msgs::PointCloud2());
            pcl::toROSMsg(*cloud, *cloud_out_ptr);
            cloud_out_ptr->header.stamp = ros::Time().fromSec(time);
            cloud_out_ptr->header.frame_id = "map";
            pub_lidar_cloud.publish(*cloud_out_ptr);
            return true;
        });
    ros::Publisher pubLaserPath = nh.advertise<nav_msgs::Path>("odom_path", 5);
    ros::Publisher pubLaserOdOM = nh.advertise<nav_msgs::Odometry>("odom", 100);

    auto pose_pub_func = std::function<bool(std::string & topic_name, SE3 & pose, double timestamp)>(
        [&](std::string& topic_name, SE3& pose, double timestamp) -> bool {
            //
            tf::Transform transform;
            Eigen::Quaterniond q_curr(pose.so3().matrix());
            transform.setOrigin(tf::Vector3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
            tf::Quaternion q(q_curr.x(), q_curr.y(), q_curr.z(), q_curr.w());
            transform.setRotation(q);
            if (topic_name == "laser") {
                // 发布tf信息
                tf_buff.sendTransform(
                    tf::StampedTransform(transform, ros::Time().fromSec(timestamp), "map", "base_link"));
                // 发布里程计信息
                nav_msgs::Odometry laserodom;
                laserodom.header.frame_id = "map";
                laserodom.child_frame_id = "base_link";
                laserodom.header.stamp = ros::Time().fromSec(timestamp);
                laserodom.pose.pose.position.x = pose.translation().x();
                laserodom.pose.pose.position.y = pose.translation().y();
                laserodom.pose.pose.position.z = pose.translation().z();
                laserodom.pose.pose.orientation.x = q_curr.x();
                laserodom.pose.pose.orientation.y = q_curr.y();
                laserodom.pose.pose.orientation.z = q_curr.z();
                laserodom.pose.pose.orientation.w = q_curr.w();
                pubLaserOdOM.publish(laserodom);
                // pub laserpath
                geometry_msgs::PoseStamped laserpose;
                laserpose.header = laserodom.header;
                laserpose.pose = laserodom.pose.pose;
                laserOdomPath.header.stamp = laserodom.header.stamp;
                laserOdomPath.poses.push_back(laserpose);
                laserOdomPath.header.frame_id = "map";
                pubLaserPath.publish(laserOdomPath);
            }
            return true;
        });
    ros::Publisher vel_pub = nh.advertise<std_msgs::Float32>("/velocity", 1);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float32>("/move_dist", 1);
    auto data_pub_func = std::function<bool(std::string & topic_name, double time1, double time2)>(
        [&](std::string& topic_name, double time1, double time2) {
            std_msgs::Float32 time_rviz;

            time_rviz.data = time1;
            if (topic_name == "velocity")
                vel_pub.publish(time_rviz);
            else
                dist_pub.publish(time_rviz);

            return true;
        });

    lidar_convert_ptr = std::make_shared<ctlio::slam::LidarProcess>();
    lidar_convert_ptr->LoadFromYaml(config_file);

    lio_ptr->setFunc(cloud_pub_func);
    lio_ptr->setFunc(pose_pub_func);
    lio_ptr->setFunc(data_pub_func);

    auto config_node = YAML::LoadFile(config_file);
    std::string lidar_topic = config_node["common"]["lidar_topic"].as<std::string>();
    std::string imu_topic = config_node["common"]["imu_topic"].as<std::string>();
    ros::Subscriber laser_sub = lidar_convert_ptr->lidar_type_ == ctlio::slam::LidarProcess::LidarType::AVIA
                                    ? nh.subscribe(lidar_topic, 100, livox_pcl_cbk)
                                    : nh.subscribe<sensor_msgs::PointCloud2>(lidar_topic, 100, standard_pcl_cbk);

    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 500, imu_callback);

    std::thread measurement_process();

    ros::spin();
    sleep(3);
    return 0;
}