#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/impl/pcl_base.hpp>
#include <vector>

namespace sensors {
using PointType = pcl::PointXYZI;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
using IndexVec = std::vector<int>;

// 点云到Eigen vector的转换

inline Eigen::Vector3f ToVec3f(const PointType& point) {
    return point.getVector3fMap();
}

inline Eigen::Vector3d ToVec3d(const PointType& point) {
    return point.getVector3fMap().cast<double>();
}
// Eigen to 点云类型，可以接受浮点或者double
template <typename S>
inline PointType ToPointType(const Eigen::Matrix<S, 3, 1>& pt) {
    PointType point;
    point.x = pt.x();
    point.y = pt.y();
    point.z = pt.z();
    return point;
}

// 定义全量信息点云结构体
struct FullPointType {
    PCL_ADD_POINT4D;
    float range = 0;
    float radius = 0;
    uint8_t intensity = 0;
    uint8_t ring = 0;
    uint8_t angle = 0;
    double time = 0;
    float height = 0;

    inline FullPointType() {
    }
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// 全量点云的定义
using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;

inline Eigen::Vector3f ToVec3f(const FullPointType& pt) {
    return pt.getVector3fMap();
}
inline Eigen::Vector3d ToVec3d(const FullPointType& pt) {
    return pt.getVector3fMap().cast<double>();
}
}  // namespace sensors

// 注册点云
POINT_CLOUD_REGISTER_POINT_STRUCT(sensors::FullPointType,
                                  (float, x, x)(float, y, y)(float, z, z)(float, range, range)(float, radius, radius)(
                                      std::uint8_t, intensity, intensity)(std::uint16_t, angle, angle)(
                                      std::uint8_t, ring, ring)(double, time, time)(float, height, height))
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
namespace leishen_ros {
struct PointXYZIRT_LS {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint16_t ring;  // 行号
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
};
}  // namespace leishen_ros
   // clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(leishen_ros::PointXYZIRT_LS,
                                  (float, x, x)(float, y, y)
                                  (float, z, z)
                                  (float, intensity,intensity)
                                  (std::uint16_t, ring,ring)
                                  (double, time, time)
                                 )
// clang-format on

namespace rslidar_ros {
struct PointXYZIRT_RS {
    PCL_ADD_POINT4D;
    PCL_ADD_INTENSITY;
    std::uint16_t ring;  // 行号
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
};
}  // namespace rslidar_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::PointXYZIRT_RS,
                                  (float, x, x)(float, y, y)
                                  (float, z, z)
                                  (float, intensity,intensity)
                                  (std::uint16_t, ring,ring)
                                  (double, timestamp, timestamp)
                                 )
