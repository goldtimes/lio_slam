#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2_ros/transform_broadcaster.h>
#include "eskf/eskf.hpp"

void imu_callback(const sensor_msgs::Imu& imu_msg) {
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_eskf");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe("imu", 200, imu_callback);
    ros::spin();
    return 0;
}
