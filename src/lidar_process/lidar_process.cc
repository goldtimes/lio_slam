/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-16 23:10:10
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-17 00:09:55
 * @FilePath: /lio_ws/src/ieskf_slam/src/lidar_process/lidar_process.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "lidar_process.hh"
#include <glog/logging.h>

namespace ctlio::slam {

void LidarProcess::LoadFromYaml(const std::string& yaml_file) {
    YAML::Node config_node = YAML::LoadFile(yaml_file);
    int lidar_type = config_node["lidar_process"]["lidar_type"].as<int>();
    point_filter_num = config_node["lidar_process"]["point_filter_num"].as<int>();
    blind = config_node["lidar_process"]["blind"].as<double>();
    if (lidar_type == 1) {
        lidar_type_ = LidarType::AVIA;
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        lidar_type_ = LidarType::VELO32;
        LOG(INFO) << "Using AVIA VELO32";
    } else if (lidar_type == 3) {
        lidar_type_ == LidarType::OUST64;
        LOG(INFO) << "Using AVIA OUST64";
    } else if (lidar_type == 4) {
        lidar_type_ == LidarType::ROBOSENSE16;
        LOG(INFO) << "Using AVIA ROBOSENSE16";
    } else if (lidar_type == 5) {
        lidar_type_ == LidarType::PANDAR;
        LOG(INFO) << "Using AVIA PANDAR";
    } else if (lidar_type == 6) {
        lidar_type_ == LidarType::LEISHEN;
        LOG(INFO) << "Using AVIA LEISHEN";
    } else {
        LOG(ERROR) << "unknown lidar_type";
    }
}

void LidarProcess::Process(const livox_ros_driver::CustomMsg::ConstPtr& msg, std::vector<point3D>& pcl_out) {
    AviaHandler(msg);
    pcl_out = cloud_out_;
}

void LidarProcess::Process(const sensor_msgs::PointCloud2::ConstPtr& msg, std::vector<point3D>& pcl_out) {
    switch (lidar_type_) {
        case LidarType::VELO32:
            VelodyneHandler(msg);
            break;
        case LidarType::OUST64:
            Oust64Handler(msg);
            /* code */
            break;
        case LidarType::ROBOSENSE16:
            RobosenseHandler(msg);
            /* code */
            break;
        case LidarType::PANDAR:
            /* code */
            PandarHandler(msg);
            break;
        case LidarType::LEISHEN:
            /* code */
            LeishenHandler(msg);
            break;
        default:
            LOG(ERROR) << "Error LiDAR Type: " << int(lidar_type_);
            break;
    }
    pcl_out = cloud_out_;
}

void LidarProcess::AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr& msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    int points_size = msg->point_num;
    cloud_out_.reserve(points_size);

    static int tm_scale = 1e9;  // 1s = 1e9 ns
    // 时间戳是帧首的时间戳
    double headertime = msg->header.stamp.toSec();
    timespan_ = msg->points.back().offset_time / tm_scale;
    // 遍历所有的点
    for (size_t i = 0; i < points_size; ++i) {
        // 过滤nan点
        if (!(std::isfinite(msg->points[i].x) && std::isfinite(msg->points[i].y) && std::isfinite(msg->points[i].z))) {
            continue;
        }
        // 跳点，如果point_filter_num = 1, 那么就不跳点
        if (i % point_filter_num != 0) {
            continue;
        }
        double range = msg->points[i].x * msg->points[i].x + msg->points[i].y * msg->points[i].y +
                       msg->points[i].z * msg->points[i].z;
        // 选择0.1 ~ 150m范围的点
        if (range > 150 * 150 || range < blind * blind) {
            continue;
        }
        if ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00) {
            point3D point_tmp;
            point_tmp.raw_point = Vec3d(msg->points[i].x, msg->points[i].y, msg->points[i].z);
            point_tmp.point = point_tmp.raw_point;
            point_tmp.relative_time = msg->points[i].offset_time / tm_scale;  // ms
            point_tmp.intensity = msg->points[i].reflectivity;
            point_tmp.timestamp = headertime + point_tmp.relative_time;
            point_tmp.alpha_time = point_tmp.relative_time / timespan_;
            point_tmp.timespan = timespan_;
            point_tmp.ring = msg->points[i].line;
            cloud_out_.push_back(point_tmp);
        }
    }
}

void LidarProcess::VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    double headertime = msg->header.stamp.toSec();

    static double tm_scale = 1;  //   1e6 - nclt kaist or 1

    //  FIXME:  nclt 及kaist时间戳大于0.1
    auto time_list_velodyne = [&](velodyne_ros::Point& point_1, velodyne_ros::Point& point_2) {
        return (point_1.time < point_2.time);
    };
    sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_velodyne);
    while (pl_orig.points[plsize - 1].time / tm_scale >= 0.1) {
        plsize--;
        pl_orig.points.pop_back();
    }
    timespan_ = pl_orig.points.back().time / tm_scale;
    // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].time / tm_scale << " , 100: " <<
    // pl_orig.points[100].time / tm_scale << std::endl;

    for (int i = 0; i < plsize; i++) {
        if (!(std::isfinite(pl_orig.points[i].x) && std::isfinite(pl_orig.points[i].y) &&
              std::isfinite(pl_orig.points[i].z)))
            continue;

        if (i % point_filter_num != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        if (range > 150 * 150 || range < blind * blind) continue;

        point3D point_temp;
        point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
        point_temp.point = point_temp.raw_point;
        point_temp.relative_time = pl_orig.points[i].time / tm_scale;  // curvature unit: s
        point_temp.intensity = pl_orig.points[i].intensity;

        point_temp.timestamp = headertime + point_temp.relative_time;
        point_temp.alpha_time = point_temp.relative_time / timespan_;
        point_temp.timespan = timespan_;
        point_temp.ring = pl_orig.points[i].ring;

        cloud_out_.push_back(point_temp);
    }
}

void LidarProcess::Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_out_.clear();
    cloud_full_.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    cloud_out_.reserve(plsize);

    static double tm_scale = 1e9;

    double headertime = msg->header.stamp.toSec();
    timespan_ = pl_orig.points.back().t / tm_scale;
    // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[0].t / tm_scale
    //           << " , 100: " << pl_orig.points[100].t / tm_scale
    //           << std::endl;

    for (int i = 0; i < pl_orig.points.size(); i++) {
        if (!(std::isfinite(pl_orig.points[i].x) && std::isfinite(pl_orig.points[i].y) &&
              std::isfinite(pl_orig.points[i].z)))
            continue;

        if (i % point_filter_num != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        if (range > 150 * 150 || range < blind * blind) continue;

        point3D point_temp;
        point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
        point_temp.point = point_temp.raw_point;
        point_temp.relative_time = pl_orig.points[i].t / tm_scale;  // curvature unit: ms
        point_temp.intensity = pl_orig.points[i].intensity;

        point_temp.timestamp = headertime + point_temp.relative_time;
        point_temp.alpha_time = point_temp.relative_time / timespan_;
        point_temp.timespan = timespan_;
        point_temp.ring = pl_orig.points[i].ring;

        cloud_out_.push_back(point_temp);
    }
}
void LidarProcess::RobosenseHandler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_full_.clear();
    cloud_out_.clear();
    pcl::PointCloud<robosense_ros::Point> pl_orin;
    pcl::fromROSMsg(*msg, pl_orin);
    int points_size = pl_orin.size();
    cloud_out_.reserve(points_size);

    double headertime = msg->header.stamp.toSec();

    auto sort_by_time = [&](const robosense_ros::Point& point_1, const robosense_ros::Point& point_2) {
        return point_1.timestamp < point_2.timestamp;
    };

    std::sort(pl_orin.begin(), pl_orin.end(), sort_by_time);

    // 修复帧>0.1的间隔问题
    while (pl_orin[points_size - 1].timestamp - pl_orin[0].timestamp >= 0.1) {
        points_size--;
        pl_orin.points.pop_back();
    }
    timespan_ = pl_orin.points.back().timestamp - pl_orin.points[0].timestamp;
    for (size_t i = 0; i < points_size; ++i) {
        auto point = pl_orin.points[i];
        if (!(std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))) {
            continue;
        }
        // if (i % point_filter_num != 0) {
        //     continue;
        // }
        double range = point.x * point.x + point.y * point.y + point.z * point.z;
        if (range > 150 * 150 || range < blind * blind) {
            continue;
        }

        point3D point_tmp;
        point_tmp.raw_point = Vec3d(point.x, point.y, point.z);
        point_tmp.point = point_tmp.raw_point;

        point_tmp.relative_time = point.timestamp - pl_orin.points[0].timestamp;
        point_tmp.intensity = point.intensity;
        point_tmp.timestamp = point.timestamp;
        point_tmp.alpha_time = point_tmp.relative_time / timespan_;
        point_tmp.timespan = timespan_;
        point_tmp.ring = point.ring;
        if (point_tmp.alpha_time > 1 || point_tmp.alpha_time < 0) {
            std::cout << point_tmp.alpha_time << ", this may error." << std::endl;
        }
        cloud_full_.push_back(point_tmp);
    }
}
void LidarProcess::PandarHandler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    cloud_out_.clear();
    cloud_full_.clear();

    pcl::PointCloud<pandar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    cloud_out_.reserve(plsize);

    double headertime = msg->header.stamp.toSec();

    static double tm_scale = 1;  //   1e6

    auto time_list_pandar = [&](pandar_ros::Point& point_1, pandar_ros::Point& point_2) {
        return (point_1.timestamp < point_2.timestamp);
    };
    sort(pl_orig.points.begin(), pl_orig.points.end(), time_list_pandar);
    while (pl_orig.points[plsize - 1].timestamp - pl_orig.points[0].timestamp >= 0.1) {
        plsize--;
        pl_orig.points.pop_back();
    }
    timespan_ = pl_orig.points.back().timestamp - pl_orig.points[0].timestamp;

    // std::cout << "span:" << timespan_ << ",0: " << pl_orig.points[1].timestamp - pl_orig.points[0].timestamp
    //           << " , 100: " << pl_orig.points[100].timestamp - pl_orig.points[0].timestamp
    //           << msg->header.stamp.toSec() - pl_orig.points[0].timestamp << ", "
    //           << msg->header.stamp.toSec() - pl_orig.points.back().timestamp << std::endl;

    for (int i = 0; i < plsize; i++) {
        if (!(std::isfinite(pl_orig.points[i].x) && std::isfinite(pl_orig.points[i].y) &&
              std::isfinite(pl_orig.points[i].z)))
            continue;

        if (i % point_filter_num != 0) continue;

        double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                       pl_orig.points[i].z * pl_orig.points[i].z;
        if (range > 150 * 150 || range < blind * blind) continue;

        point3D point_temp;
        point_temp.raw_point = Eigen::Vector3d(pl_orig.points[i].x, pl_orig.points[i].y, pl_orig.points[i].z);
        point_temp.point = point_temp.raw_point;
        point_temp.relative_time = pl_orig.points[i].timestamp - pl_orig.points[0].timestamp;
        point_temp.intensity = pl_orig.points[i].intensity;

        point_temp.timestamp = headertime + point_temp.relative_time;
        point_temp.alpha_time = point_temp.relative_time / timespan_;
        point_temp.timespan = timespan_;
        point_temp.ring = pl_orig.points[i].ring;

        cloud_out_.push_back(point_temp);
    }
}
void LidarProcess::LeishenHandler(const sensor_msgs::PointCloud2::ConstPtr& msg) {
}

}  // namespace ctlio::slam