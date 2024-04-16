
#ifndef SLAM_IN_AUTO_DRIVING_POINT_TYPES_H
#define SLAM_IN_AUTO_DRIVING_POINT_TYPES_H
// 多种点云类型的定义
#include <livox_ros_driver/CustomMsg.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/pcl_base.hpp>
#include "common/eigen_types.hh"

namespace ctlio {
// 定义点和点云类型
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
}  // namespace ctlio
namespace liovx_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float time;
    uint16_t ring;
    uint16_t tag;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace liovx_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(liovx_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      float, time, time)(uint16_t, ring, ring)(uint16_t, tag, tag))

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))
// clang-format on

namespace robosense_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    uint8_t intensity;
    uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace robosense_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(robosense_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
                                      uint16_t, ring, ring)(double, timestamp, timestamp))

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                      (std::uint32_t, t, t)
                                      (std::uint16_t, reflectivity, reflectivity)
                                      (std::uint8_t, ring, ring)
                                      (std::uint16_t, ambient, ambient)
                                      (std::uint32_t, range, range)
)
// clang-format on

namespace pandar_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pandar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(pandar_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(double, timestamp,
                                                                                     timestamp)(uint16_t, ring, ring))
// 添加镭神点云定义
namespace leishen_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint16_t ring;  // 行号
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(leishen_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      uint16_t, ring, ring)(double, timestamp, timestamp))

}  // namespace leishen_ros
#endif