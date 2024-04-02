/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 00:04:31
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-03 00:06:55
 * @FilePath: /lio_ws/src/ieskf_slam/src/app/ieskf_node.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "ros_wrapper/frontend_ros_wrapper.hh"

int main(int argc, char** argv) {
    ros::init(argc, argv, "frontend_node");
    ros::NodeHandle nh;
    std::shared_ptr<IESKF_SLAM::FrontendRosWrapper> frontend_ros_ptr =
        std::make_shared<IESKF_SLAM::FrontendRosWrapper>(nh);
    return 0;
}