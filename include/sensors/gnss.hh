#pragma once

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

namespace sensors {
/// GNSS状态位信息
/// 通常由GNSS厂商提供，这里使用千寻提供的状态位
enum class GpsStatusType {
    GNSS_FLOAT_SOLUTION = 5,         // 浮点解（cm到dm之间）
    GNSS_FIXED_SOLUTION = 4,         // 固定解（cm级）
    GNSS_PSEUDO_SOLUTION = 2,        // 伪距差分解（分米级）
    GNSS_SINGLE_POINT_SOLUTION = 1,  // 单点解（10m级）
    GNSS_NOT_EXIST = 0,              // GPS无信号
    GNSS_OTHER = -1,                 // 其他
};
// UTM 坐标
struct UTMCoordinate {
    UTMCoordinate() = default;
    explicit UTMCoordinate(int zone, const Eigen::Vector2d& xy = Eigen::Vector2d::Zero(), bool north = true)
        : zone_(zone), xy_(xy), north_(north) {
    }
    int zone_ = 0;  // utm区域
    Eigen::Vector2d xy_ = Eigen::Vector2d::Zero();
    double z_ = 0;
    bool north_ = true;
};

// GNSS
struct GNSS {
    GNSS() = default;
    GNSS(double unix_time, int status, const Eigen::Vector3d& lat_lon_alt, double heading, bool heading_valid)
        : unix_time_(unix_time), lat_lon_alt_(lat_lon_alt), heading_(heading), heading_valid_(heading_valid) {
        status_ = GpsStatusType(status);
    }

    /// 从ros的NavSatFix进行转换
    /// NOTE 这个只有位置信息而没有朝向信息，UTM坐标请从ch3的代码进行转换
    GNSS(sensor_msgs::NavSatFix::Ptr msg) {
        unix_time_ = msg->header.stamp.toSec();
        // 状态位
        if (int(msg->status.status) >= int(sensor_msgs::NavSatStatus::STATUS_FIX)) {
            status_ = GpsStatusType::GNSS_FIXED_SOLUTION;
        } else {
            status_ = GpsStatusType::GNSS_OTHER;
        }
        // 经纬度
        lat_lon_alt_ << msg->latitude, msg->longitude, msg->altitude;
    }
    double unix_time_ = 0;
    GpsStatusType status_ = GpsStatusType::GNSS_NOT_EXIST;
    // 经纬高
    Eigen::Vector3d lat_lon_alt_ = Eigen::Vector3d::Zero();
    double heading_ = 0.0;
    bool heading_valid_ = false;

    UTMCoordinate utm_;
    bool utm_valid_;
    Sophus::SE3d utm_pose_;
};

}  // namespace sensors