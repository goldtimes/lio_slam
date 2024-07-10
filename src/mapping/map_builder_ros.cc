#include "mapping/map_builder_ros.hh"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include "sensors/imu.hh"
#include "sensors/point_types.hh"

namespace lio {

MapBuilderRos::MapBuilderRos(ros::NodeHandle& nh, tf2_ros::TransformBroadcaster& tf,
                             std::shared_ptr<LoopSharedData> shared_data)
    : nh_(nh), tf_(tf), loop_shared_data_(shared_data) {
    loop_closure_ = std::make_shared<LoopClosureThread>();
    init_params();
    init_sub_pub();
    init_service();
    lio_buidler_ = std::make_shared<IGLIOBuilder>(lio_params_);
    // loop closure
    loop_closure_->setLoopRate(loop_rate_);
    loop_closure_->setSharedData(loop_shared_data_);
    loop_closure_->init();
    // 使用类对象创建线程，需要重载()运算符
    loop_thread_ = std::make_shared<std::thread>(std::ref(*loop_closure_));
}

void MapBuilderRos::init_params() {
    nh_.param<std::string>("map_frame", global_frame_, "map");
    nh_.param<std::string>("local_frame", local_frame_, "odom");
    nh_.param<std::string>("body_frame", body_frame_, "imu");
    nh_.param<std::string>("imu_topic", imu_topic_, "/livox/imu");
    nh_.param<std::string>("livox_topic", livox_topic_, "/livox/lidar");
    double local_rate, loop_rate;
    nh_.param<double>("local_rate", local_rate, 20.0);
    nh_.param<double>("loop_rate", loop_rate, 1.0);
    local_rate_ = std::make_shared<ros::Rate>(local_rate);
    loop_rate_ = std::make_shared<ros::Rate>(loop_rate);
    nh_.param<double>("lio_builder/scan_resolution", lio_params_.scan_resolution, 0.5);
    nh_.param<double>("lio_builder/map_resolution", lio_params_.map_resolution, 0.5);
    nh_.param<double>("lio_builder/point2plane_gain", lio_params_.point2plane_gain, 1000.0);
    nh_.param<double>("lio_builder/plane2plane_gain", lio_params_.plane2plane_gain, 100.0);
    int map_capacity, grid_capacity;
    nh_.param<int>("lio_builder/map_capacity", map_capacity, 5000000);
    nh_.param<int>("lio_builder/grid_capacity", grid_capacity, 20);

    lio_params_.map_capacity = static_cast<size_t>(map_capacity);
    lio_params_.grid_capacity = static_cast<size_t>(grid_capacity);
    nh_.param<bool>("lio_builder/align_gravity", lio_params_.align_gravity, true);
    nh_.param<bool>("lio_builder/extrinsic_est_en", lio_params_.extrisic_est_en, false);
    nh_.param<std::vector<double>>("lio_builder/imu_ext_rot", lio_params_.imu_rot_ext, std::vector<double>());
    nh_.param<std::vector<double>>("lio_builder/imu_ext_pos", lio_params_.imu_ext_pos, std::vector<double>());
    int mode;
    nh_.param<int>("lio_builder/near_mode", mode, 1);
    switch (mode) {
        case 1:
            lio_params_.nearby_ = lio::VoxelMap::NEARBY::NEARBY1;
            break;
        case 2:
            lio_params_.nearby_ = lio::VoxelMap::NEARBY::NEARBY7;
            break;
        case 3:
            lio_params_.nearby_ = lio::VoxelMap::NEARBY::NEARBY19;
            break;
        case 4:
            lio_params_.nearby_ = lio::VoxelMap::NEARBY::NEARBY26;
            break;
        default:
            lio_params_.nearby_ = lio::VoxelMap::NEARBY::NEARBY1;
            break;
    }

    nh_.param<bool>("loop_closure/active", loop_closure_->getLoopParams().active, true);
    nh_.param<double>("loop_closure/rad_thresh", loop_closure_->getLoopParams().rad_thresh, 0.4);
    nh_.param<double>("loop_closure/dis_thresh", loop_closure_->getLoopParams().dis_thresh, 2.5);
    nh_.param<double>("loop_closure/time_thresh", loop_closure_->getLoopParams().time_thresh, 30.0);
    nh_.param<double>("loop_closure/loop_pose_search_radius", loop_closure_->getLoopParams().loop_pose_search_radius,
                      10.0);
    nh_.param<int>("loop_closure/loop_pose_min_size_thresh", loop_closure_->getLoopParams().loop_pose_min_size_thresh,
                   5);
    nh_.param<double>("loop_closure/submap_resolution", loop_closure_->getLoopParams().submap_resolution, 0.2);
    nh_.param<int>("loop_closure/submap_search_num", loop_closure_->getLoopParams().submap_search_num, 20);
    nh_.param<double>("loop_closure/loop_icp_thresh", loop_closure_->getLoopParams().loop_icp_thresh, 0.3);
    nh_.param<bool>("loop_closure/z_prior", loop_closure_->getLoopParams().z_prior, false);
}

void MapBuilderRos::init_sub_pub() {
    imu_sub_ = nh_.subscribe(imu_topic_, 1000, &MapBuilderRos::imu_callback, this);
    cloud_sub_ = nh_.subscribe(livox_topic_, 1000, &MapBuilderRos::livox_callback, this);

    lidar_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("lidar_cloud", 10);
    body_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("body_cloud", 10);
    local_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("local_cloud", 10);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("slam_odom", 1000);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("local_path", 1000);
    global_path_pub_ = nh_.advertise<nav_msgs::Path>("global_path", 1000);
    loop_mark_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("loop_mark", 1000);
}

void MapBuilderRos::init_service() {
    save_map_server_ = nh_.advertiseService("save_map", &MapBuilderRos::save_map_callback, this);
}

bool MapBuilderRos::save_map_callback(lio_slam::SaveMap::Request& req, lio_slam::SaveMap::Response& resp) {
    std::string file_path = req.save_path;
    sensors::PointNormalCloud::Ptr out_cloud(new sensors::PointNormalCloud());
    for (Pose6D& p : loop_shared_data_->keyposes) {
        sensors::PointNormalCloud::Ptr tmp_cloud(new sensors::PointNormalCloud);
        pcl::transformPointCloud(*(loop_shared_data_->cloud_history[p.index]), *tmp_cloud, p.global_pos,
                                 Eigen::Quaterniond(p.global_rot));
        *out_cloud += *tmp_cloud;
    }
    if (out_cloud->empty()) {
        resp.status = false;
        resp.message = "Empty cloud!";
        return false;
    }
    resp.status = true;
    resp.message = "Save Map Success!, path is " + file_path;
    writer_.writeBinaryCompressed(file_path, *out_cloud);
    return true;
}

void MapBuilderRos::imu_callback(const sensor_msgs::Imu& imu_message) {
    // 加锁
    // std::cout << "imu_callback" << std::endl;
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    Eigen::Vector3d acc(imu_message.linear_acceleration.x, imu_message.linear_acceleration.y,
                        imu_message.linear_acceleration.z);
    Eigen::Vector3d gyro(imu_message.angular_velocity.x, imu_message.angular_velocity.y,
                         imu_message.angular_velocity.z);
    double imu_timestamp = imu_message.header.stamp.toSec();
    sensors::IMU imu(imu_timestamp, gyro, acc);
    if (imu_timestamp < last_received_imu_timestamp) {
        ROS_WARN("imu loop back, clear buffer, last_timestamp: %f  current_timestamp: %f", last_received_imu_timestamp,
                 imu_timestamp);
        imu_queue_.clear();
    }
    last_received_imu_timestamp = imu_timestamp;
    imu_queue_.push_back(imu);
}

void MapBuilderRos::livox_callback(const livox_ros_driver::CustomMsgConstPtr& livox_cloud_msg) {
    // LOG(INFO) << "livox_callback" << std::endl;
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    double lidar_timestamp = livox_cloud_msg->header.stamp.toSec();
    if (lidar_timestamp < last_received_lidar_timestamp) {
        ROS_WARN("lidar loop back, clear buffer, last_timestamp: %f  current_timestamp: %f",
                 last_received_lidar_timestamp, lidar_timestamp);
        livox_datas.clouds_buff.clear();
        livox_datas.time_buffer.clear();
    }
    last_received_lidar_timestamp = lidar_timestamp;
    sensors::PointNormalCloud::Ptr cloud(new sensors::PointNormalCloud());
    livox2pcl(livox_cloud_msg, cloud);
    // pub cloud
    sensor_msgs::PointCloud2 cloud_lidar;
    pcl::toROSMsg(*cloud, cloud_lidar);
    cloud_lidar.header.frame_id = "lidar";
    cloud_lidar.header.stamp = ros::Time().fromSec(lidar_timestamp);
    lidar_cloud_pub_.publish(cloud_lidar);
    livox_datas.clouds_buff.push_back(cloud);
    livox_datas.time_buffer.push_back(lidar_timestamp);
}

void MapBuilderRos::livox2pcl(const livox_ros_driver::CustomMsg::ConstPtr& livox_cloud,
                              sensors::PointNormalCloud::Ptr& out_cloud) {
    // 获取点云中的个数
    int point_num = livox_cloud->point_num;
    out_cloud->clear();
    // 预分配空间
    out_cloud->reserve(point_num / (filter_num + 1));
    uint valid_num = 0;
    for (int i = 0; i < point_num; ++i) {
        // 有效的点
        if ((livox_cloud->points[i].line < 4) &&
            ((livox_cloud->points[i].tag & 0x03) == 0x10 || (livox_cloud->points[i].tag & 0x30) == 0x00)) {
            valid_num++;
            if (valid_num % filter_num != 0) {
                continue;
            }
            sensors::PointNormalType p;
            p.x = livox_cloud->points[i].x;
            p.y = livox_cloud->points[i].y;
            p.z = livox_cloud->points[i].z;
            p.intensity = livox_cloud->points[i].reflectivity;
            // 纳秒 / 毫秒的转换
            p.curvature = livox_cloud->points[i].offset_time / 1e6;
            if (p.x * p.x + p.y * p.y + p.z * p.z > (blind * blind)) {
                out_cloud->push_back(p);
            }
        }
    }
}

bool MapBuilderRos::syncMeasure(std::deque<sensors::IMU>& imu_queue, LivoxData& livox_datas) {
    // 1. 数据不为空
    if (imu_queue.empty() || livox_datas.clouds_buff.empty()) {
        // std::cerr << "imu or lidar empty" << std::endl;
        return false;
    }
    // 2. push lidar
    if (!measure_group_.lidar_pushed) {
        measure_group_.lidar_cloud = livox_datas.clouds_buff.front();
        measure_group_.lidar_begin_time = livox_datas.time_buffer.front();
        measure_group_.lidar_end_time =
            measure_group_.lidar_begin_time + measure_group_.lidar_cloud->points.back().curvature / 1e3;
        measure_group_.lidar_pushed = true;
    }
    // 3. 时间同步
    if (last_received_lidar_timestamp < measure_group_.lidar_end_time) {
        return false;
    }
    double imu_time = imu_queue.front().timestamp_;
    std::deque<sensors::IMU> imu_datas;
    imu_datas.clear();
    // ROS_INFO("lidar_begin_time: %f", measure_group_.lidar_begin_time);

    while (!imu_queue.empty() && (imu_time < measure_group_.lidar_end_time)) {
        imu_time = imu_queue.front().timestamp_;
        if (imu_time > measure_group_.lidar_end_time) {
            break;
        }
        if (imu_time < measure_group_.lidar_begin_time) {
            imu_queue.pop_front();
            continue;
        }
        // ROS_INFO("imu_time: %f", imu_time);

        imu_datas.push_back(imu_queue.front());
        imu_queue.pop_front();
    }
    livox_datas.clouds_buff.pop_front();
    livox_datas.time_buffer.pop_front();
    measure_group_.lidar_pushed = false;
    measure_group_.imudatas.clear();
    measure_group_.imudatas.insert(measure_group_.imudatas.end(), imu_datas.begin(), imu_datas.end());
    // ROS_INFO("lidar_end_time: %f", measure_group_.lidar_end_time);
    // ROS_DEBUG("imu datas: %d", measure_group_.imudatas.size());

    return true;
}

void MapBuilderRos::run() {
    while (ros::ok()) {
        local_rate_->sleep();
        ros::spinOnce();
        // 时间同步ok,同步的数据放在了measure_group中
        tic_toc.tic();
        if (!syncMeasure(imu_queue_, livox_datas)) {
            continue;
        }
        double sync_time = tic_toc.toc();
        // ROS_INFO("syncMeasuer: %6f", sync_time);
        // 那么接下来就是imu的初始化和点云的去畸变,发布去畸变后的点云
        // 将同步的数据放到lio中
        lio_buidler_->mapping(measure_group_);
        // ieskf系统初始化
        if (lio_buidler_->GetCurrentStatus() == LIO_STATUS::INITIALIZE) {
            continue;
        }
        current_time_ = measure_group_.lidar_end_time;
        current_navi_state_ = lio_buidler_->GetState();
        // 可视化发布
        // 发布tf
        tf_.sendTransform(eigen2Tf(loop_shared_data_->offset_rot, loop_shared_data_->offset_trans, global_frame_,
                                   local_frame_, current_time_));
        tf_.sendTransform(
            eigen2Tf(current_navi_state_.rot, current_navi_state_.pos, local_frame_, body_frame_, current_time_));
        // 发布里程计
        publishOdom(
            eigen2odom(current_navi_state_.rot, current_navi_state_.pos, local_frame_, body_frame_, current_time_));
        // 更新keypose
        addKeypose();
        // 发布点云
        publishCloud(body_cloud_pub_, pcl2msg(lio_buidler_->cloudUndistortedBody(), body_frame_, current_time_));
        publishCloud(local_cloud_pub_, pcl2msg(lio_buidler_->cloudWorld(), local_frame_, current_time_));
        publishLocalPath();
        publishGlobalPath();
        publishLoopMark();
    }
    std::cout << "map builder thread exits" << std::endl;
}

void MapBuilderRos::addKeypose() {
    int idx = loop_shared_data_->keyposes.size();
    // 第一帧
    if (loop_shared_data_->keyposes.empty()) {
        std::lock_guard<std::mutex> lck(loop_shared_data_->mutex);
        loop_shared_data_->keyposes.emplace_back(idx, current_time_, current_navi_state_.rot, current_navi_state_.pos);
        loop_shared_data_->keyposes.back().addOffset(loop_shared_data_->offset_rot, loop_shared_data_->offset_trans);
        loop_shared_data_->keypose_add = true;
        loop_shared_data_->cloud_history.push_back(lio_buidler_->cloudUndistortedBody());
        return;
    }
    Pose6D& last_key_pose = loop_shared_data_->keyposes.back();
    Eigen::Matrix3d diff_rot = last_key_pose.local_rot.transpose() * current_navi_state_.rot;
    Eigen::Vector3d diff_pos =
        last_key_pose.local_rot.transpose() * (current_navi_state_.pos - last_key_pose.local_pos);
    Eigen::Vector3d rpy = rotate2rpy(diff_rot);
    // keyframe
    if (diff_pos.norm() > loop_closure_->getLoopParams().dis_thresh ||
        std::abs(rpy(0)) > loop_closure_->getLoopParams().rad_thresh ||
        std::abs(rpy(1)) > loop_closure_->getLoopParams().rad_thresh ||
        std::abs(rpy(2)) > loop_closure_->getLoopParams().rad_thresh) {
        std::lock_guard<std::mutex> lck(loop_shared_data_->mutex);
        loop_shared_data_->keyposes.emplace_back(idx, current_time_, current_navi_state_.rot, current_navi_state_.pos);
        loop_shared_data_->keyposes.back().addOffset(loop_shared_data_->offset_rot, loop_shared_data_->offset_trans);
        loop_shared_data_->keypose_add = true;
        loop_shared_data_->cloud_history.push_back(lio_buidler_->cloudUndistortedBody());
    }
}

void MapBuilderRos::publishOdom(const nav_msgs::Odometry& odom) {
    odom_pub_.publish(odom);
}

void MapBuilderRos::publishLocalPath() {
    if (local_path_pub_.getNumSubscribers() == 0) {
        return;
    }

    if (loop_shared_data_->keyposes.empty()) {
        return;
    }
    nav_msgs::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = ros::Time().fromSec(current_time_);
    for (const Pose6D& p : loop_shared_data_->keyposes) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = ros::Time().fromSec(current_time_);
        pose.pose.position.x = p.local_pos(0);
        pose.pose.position.y = p.local_pos(1);
        pose.pose.position.z = p.local_pos(2);
        Eigen::Quaterniond q(p.local_rot);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        path.poses.push_back(pose);
    }
    local_cloud_pub_.publish(path);
}

void MapBuilderRos::publishGlobalPath() {
    if (global_path_pub_.getNumSubscribers() == 0) {
        return;
    }

    if (loop_shared_data_->keyposes.empty()) {
        return;
    }
    nav_msgs::Path path;
    path.header.frame_id = global_frame_;
    path.header.stamp = ros::Time().fromSec(current_time_);
    for (Pose6D& p : loop_shared_data_->keyposes) {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = global_frame_;
        pose.header.stamp = ros::Time().fromSec(current_time_);
        pose.pose.position.x = p.global_pos(0);
        pose.pose.position.y = p.global_pos(1);
        pose.pose.position.z = p.global_pos(2);
        Eigen::Quaterniond q(p.global_rot);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        path.poses.push_back(pose);
    }
    global_path_pub_.publish(path);
}

void MapBuilderRos::publishCloud(const ros::Publisher& cloud_pub, const sensor_msgs::PointCloud2& cloud) {
    cloud_pub.publish(cloud);
}

void MapBuilderRos::publishLoopMark() {
    if (loop_mark_pub_.getNumSubscribers() == 0) {
        return;
    }
    if (loop_shared_data_->loop_history.empty()) {
        return;
    }
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker nodes_marker;

    nodes_marker.header.frame_id = global_frame_;
    nodes_marker.header.stamp = ros::Time().fromSec(current_time_);
    nodes_marker.ns = "loop_nodes";
    nodes_marker.id = 0;
    nodes_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    nodes_marker.action = visualization_msgs::Marker::ADD;
    nodes_marker.pose.orientation.w = 1.0;
    nodes_marker.scale.x = 0.3;
    nodes_marker.scale.y = 0.3;
    nodes_marker.scale.z = 0.3;
    nodes_marker.color.r = 1.0;
    nodes_marker.color.g = 0.8;
    nodes_marker.color.b = 0.0;
    nodes_marker.color.a = 1.0;

    visualization_msgs::Marker edges_marker;
    edges_marker.header.frame_id = global_frame_;
    edges_marker.header.stamp = ros::Time().fromSec(current_time_);
    edges_marker.ns = "loop_edges";
    edges_marker.id = 1;
    edges_marker.type = visualization_msgs::Marker::LINE_LIST;
    edges_marker.action = visualization_msgs::Marker::ADD;
    edges_marker.pose.orientation.w = 1.0;
    edges_marker.scale.x = 0.1;

    edges_marker.color.r = 0.0;
    edges_marker.color.g = 0.8;
    edges_marker.color.b = 0.0;
    edges_marker.color.a = 1.0;
    for (auto& p : loop_shared_data_->loop_history) {
        Pose6D& p1 = loop_shared_data_->keyposes[p.first];
        Pose6D& p2 = loop_shared_data_->keyposes[p.second];
        geometry_msgs::Point point1;
        point1.x = p1.global_pos(0);
        point1.y = p1.global_pos(1);
        point1.z = p1.global_pos(2);
        geometry_msgs::Point point2;
        point2.x = p2.global_pos(0);
        point2.y = p2.global_pos(1);
        point2.z = p2.global_pos(2);
        nodes_marker.points.push_back(point1);
        nodes_marker.points.push_back(point2);
        edges_marker.points.push_back(point1);
        edges_marker.points.push_back(point2);
    }
    marker_array.markers.push_back(nodes_marker);
    marker_array.markers.push_back(edges_marker);
    loop_mark_pub_.publish(marker_array);
}

}  // namespace lio