#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <csignal>
#include "mapping/map_builder_ros.hh"
// bool terminate_flag = false;

// 注册程序退出的信号回调函数
// void signalHandler(int signum) {
//     std::cout << "SHUTTING DOWN MAPPING NODE!" << std::endl;
//     terminate_flag = true;
// }

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_builder_node");
    ros::NodeHandle nh;
    tf2_ros::TransformBroadcaster tf_;
    // signal(SIGINT, signalHandler);
    lio::MapBuilderRos map_builder_ros(nh, tf_);
    map_builder_ros.run();
    return 0;
}