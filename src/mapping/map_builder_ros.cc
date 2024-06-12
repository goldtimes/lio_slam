#include "mapping/map_builder_ros.hh"
#include <iostream>
#include "sensors/imu.hh"
#include "sensors/point_types.hh"

namespace lio {

MapBuilderRos::MapBuilderRos(ros::NodeHandle& nh, tf2_ros::TransformBroadcaster& tf) : nh_(nh), tf_(tf) {
    LOG(INFO) << "MapBuilderRos";
    // local_rate_ = std::make_shared<ros::Rate>(100);
    init_params();
    init_sub_pub();
}

void MapBuilderRos::init_params() {
    nh_.param<std::string>("map_frame", global_frame_, "map");
    nh_.param<std::string>("local_frame", local_frame_, "local");
    nh_.param<std::string>("body_frame", body_frame_, "body");
    nh_.param<std::string>("imu_topic", imu_topic_, "/livox/imu");
    nh_.param<std::string>("livox_topic", livox_topic_, "/livox/lidar");
    double local_rate, loop_rate;
    nh_.param<double>("local_rate", local_rate, 20.0);
    nh_.param<double>("loop_rate", loop_rate, 1.0);
    local_rate_ = std::make_shared<ros::Rate>(local_rate);
    // loop_rate_ = std::make_shared<ros::Rate>(loop_rate);
    // nh_.param<double>("lio_builder/scan_resolution", lio_params_.scan_resolution, 0.5);
    // nh_.param<double>("lio_builder/map_resolution", lio_params_.map_resolution, 0.5);
    // nh_.param<double>("lio_builder/point2plane_gain", lio_params_.point2plane_gain, 1000.0);
    // nh_.param<double>("lio_builder/plane2plane_gain", lio_params_.plane2plane_gain, 100.0);
    // int map_capacity, grid_capacity;
    // nh_.param<int>("lio_builder/map_capacity", map_capacity, 5000000);
    // nh_.param<int>("lio_builder/grid_capacity", grid_capacity, 20);

    // lio_params_.map_capacity = static_cast<size_t>(map_capacity);
    // lio_params_.grid_capacity = static_cast<size_t>(grid_capacity);
    // nh_.param<bool>("lio_builder/align_gravity", lio_params_.align_gravity, true);
    // nh_.param<bool>("lio_builder/extrinsic_est_en", lio_params_.extrinsic_est_en, false);
    // nh_.param<std::vector<double>>("lio_builder/imu_ext_rot", lio_params_.imu_ext_rot, std::vector<double>());
    // nh_.param<std::vector<double>>("lio_builder/imu_ext_pos", lio_params_.imu_ext_pos, std::vector<double>());
    // int mode;
    // nh_.param<int>("lio_builder/near_mode", mode, 1);
    // switch (mode) {
    //     case 1:
    //         lio_params_.mode = lio::VoxelMap::MODE::NEARBY_1;
    //         break;
    //     case 2:
    //         lio_params_.mode = lio::VoxelMap::MODE::NEARBY_7;
    //         break;
    //     case 3:
    //         lio_params_.mode = lio::VoxelMap::MODE::NEARBY_19;
    //         break;
    //     case 4:
    //         lio_params_.mode = lio::VoxelMap::MODE::NEARBY_26;
    //         break;

    //     default:
    //         lio_params_.mode = lio::VoxelMap::MODE::NEARBY_1;
    //         break;
    // }

    // nh_.param<bool>("loop_closure/activate", loop_closure_.mutableParams().activate, true);
    // nh_.param<double>("loop_closure/rad_thresh", loop_closure_.mutableParams().rad_thresh, 0.4);
    // nh_.param<double>("loop_closure/dist_thresh", loop_closure_.mutableParams().dist_thresh, 2.5);
    // nh_.param<double>("loop_closure/time_thresh", loop_closure_.mutableParams().time_thresh, 30.0);
    // nh_.param<double>("loop_closure/loop_pose_search_radius", loop_closure_.mutableParams().loop_pose_search_radius,
    //                   10.0);
    // nh_.param<int>("loop_closure/loop_pose_index_thresh", loop_closure_.mutableParams().loop_pose_index_thresh, 5);
    // nh_.param<double>("loop_closure/submap_resolution", loop_closure_.mutableParams().submap_resolution, 0.2);
    // nh_.param<int>("loop_closure/submap_search_num", loop_closure_.mutableParams().submap_search_num, 20);
    // nh_.param<double>("loop_closure/loop_icp_thresh", loop_closure_.mutableParams().loop_icp_thresh, 0.3);
    // nh_.param<bool>("loop_closure/z_prior", loop_closure_.mutableParams().z_prior, false);
}

void MapBuilderRos::init_sub_pub() {
    imu_sub_ = nh_.subscribe(imu_topic_, 1000, &MapBuilderRos::imu_callback, this);
    cloud_sub_ = nh_.subscribe(livox_topic_, 1000, &MapBuilderRos::livox_callback, this);
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
    sensors::IMU imu(imu_timestamp, acc, gyro);
    if (imu_timestamp < last_imu_timestamp) {
        ROS_WARN("imu loop back, clear buffer, last_timestamp: %f  current_timestamp: %f", last_imu_timestamp,
                 imu_timestamp);
        imu_queue_.clear();
    }
    last_imu_timestamp = imu_timestamp;
    imu_queue_.push_back(imu);
}

void MapBuilderRos::livox_callback(const livox_ros_driver::CustomMsgConstPtr& livox_cloud_msg) {
    // LOG(INFO) << "livox_callback" << std::endl;
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    double lidar_timestamp = livox_cloud_msg->header.stamp.toSec();
    if (lidar_timestamp < last_lidar_timestamp) {
        ROS_WARN("lidar loop back, clear buffer, last_timestamp: %f  current_timestamp: %f", last_lidar_timestamp,
                 lidar_timestamp);
        livox_datas.clouds_buff.clear();
        livox_datas.time_buffer.clear();
    }
    last_lidar_timestamp = lidar_timestamp;
    sensors::PointNormalCloud::Ptr cloud(new sensors::PointNormalCloud());
    livox2pcl(livox_cloud_msg, cloud);
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
    if (last_imu_timestamp < measure_group_.lidar_end_time) {
        return false;
    }
    double imu_time = imu_queue.front().timestamp_;
    std::deque<sensors::IMU> imu_datas;
    imu_datas.clear();
    ROS_DEBUG("imu_time: %f", imu_time);

    while (!imu_queue.empty() && (imu_time < measure_group_.lidar_end_time)) {
        imu_time = imu_queue.front().timestamp_;
        if (imu_time > measure_group_.lidar_end_time) {
            break;
        }
        imu_datas.push_back(imu_queue.front());
        imu_queue.pop_front();
    }
    livox_datas.clouds_buff.pop_front();
    livox_datas.time_buffer.pop_front();
    measure_group_.lidar_pushed = false;
    measure_group_.imudatas.clear();
    measure_group_.imudatas.insert(measure_group_.imudatas.end(), imu_datas.begin(), imu_datas.end());
    ROS_DEBUG("imu datas: %d", measure_group_.imudatas.size());
    ROS_DEBUG("lidar_begin_time: %f", measure_group_.lidar_begin_time);
    ROS_DEBUG("lidar_end_time: %f", measure_group_.lidar_end_time);
    return true;
}

void MapBuilderRos::run() {
    while (ros::ok()) {
        local_rate_->sleep();
        ros::spinOnce();
        if (!syncMeasure(imu_queue_, livox_datas)) {
            continue;
        }
        // 时间同步ok
        // 将同步的数据放到lio中
        // 可视化发布
    }
    std::cout << "map builder thread exits" << std::endl;
}

}  // namespace lio