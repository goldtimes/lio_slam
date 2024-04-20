#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <memory>
#include "state/static_imu_init.hh"
std::shared_ptr<ctlio::slam::StaticInitImu> static_imu_init_;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    // std::cout << "callback" << std::endl;
    ctlio::slam::IMU imu;
    imu.gyro_ = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    imu.acce_ = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    imu.timestamped_ = msg->header.stamp.toSec();

    static_imu_init_->AddImu(imu);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "static_imu_init_node");
    ros::NodeHandle nh;

    ros::Subscriber imu_sub = nh.subscribe("/imu", 1000, imu_callback);
    ctlio::slam::StaticInitImu::StaticOptions options;

    options.use_speed_for_static_checking_ = false;
    static_imu_init_ = std::make_shared<ctlio::slam::StaticInitImu>(options);

    ros::spin();
    return 0;
}