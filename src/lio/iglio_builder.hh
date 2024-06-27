#pragma once
#include <pcl/common/transforms.h>
#include "ieskf_fastlio/ieskf_fastlio.hh"
#include "lio/imu_process.hh"
#include "sensors/imu.hh"
#include "sensors/point_types.hh"
#include "utils/lio_utils.hh"
#include "voxel_map/voxel_map.hh"

namespace lio {
// 体素的约束
struct GICPCorrespond {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d meanA;
    Eigen::Vector3d meanB;
    Eigen::Matrix3d covA;
    Eigen::Matrix3d covB;
    GICPCorrespond(const Eigen::Vector3d& ma, const Eigen::Vector3d& mb, const Eigen::Matrix3d& ca,
                   const Eigen::Matrix3d& cb)
        : meanA(ma), meanB(mb), covA(ca), covB(cb) {
    }
};
// 点面的约束
struct FastlioCorrespond {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // lidar坐标系下的点
    Eigen::Vector3d point;
    // 平面的法向量
    Eigen::Vector4d plane;
};
// 配置参数
struct IGLIOParams {
    double point2plane_gain = 1000.0;
    double plane2plane_gain = 100.0;
    double scan_resolution = 0.5;
    double map_resolution = 0.5;

    VoxelMap::NEARBY nearby_ = VoxelMap::NEARBY::NEARBY7;

    int max_points_in_scan = 10000;
    // LRU 体素的个数
    size_t map_capacity = 5000000;
    // 每个体素中点的个数
    size_t grid_capacity = 20;
    // mode
    size_t ieskf_min_iterations = 2;
    size_t ieskf_max_iterations = 30;
    double imu_acc_cov = 0.01;
    double imu_gyro_cov = 0.01;
    double imu_acc_bias_cov = 0.0001;
    double imu_gyro_bias_cov = 0.0001;
    // clang-format off
    std::vector<double> imu_rot_ext = {1, 0, 0,
                                       0, 1, 0,
                                       0, 0, 1};
    // clang-format on
    std::vector<double> imu_ext_pos = {-0.011, -0.02329, 0.04412};

    bool extrisic_est_en = false;
    bool align_gravity = true;
};

class IGLIOBuilder {
   public:
    IGLIOBuilder(IGLIOParams& params);
    LIO_STATUS GetCurrentStatus() const {
        return lio_status_;
    };
    kf::State GetState() const {
        return ieskf_->x();
    }

    void mapping(const MeasureGroup& meas);
    /**
     * @brief 计算H矩阵和b矩阵
     *  @param state 状态量
     *  @param shared_state H矩阵和b矩阵
     */
    void sharedUpdateFunc(kf::State& state, kf::SharedState& shared_state);

    void gicpConstraint(kf::State& state, kf::SharedState& shared_state);
    void fastlioConstraint(kf::State& state, kf::SharedState& shared_state);

    sensors::PointNormalCloud::Ptr transformToWorld(const sensors::PointNormalCloud::Ptr cloud);
    sensors::PointNormalCloud::Ptr cloudUndistortedBody();
    // sensors::PointNormalCloud::Ptr cloudUndistortedlidar();
    sensors::PointNormalCloud::Ptr cloudWorld();

    void reset();

   private:
    IGLIOParams params_;

    LIO_STATUS lio_status_ = LIO_STATUS::INITIALIZE;
    std::shared_ptr<VoxelMap> voxel_map_;
    std::shared_ptr<FastVoxelMap> fast_voxel_map_;
    sensors::PointNormalCloud::Ptr cloud_lidar_;
    sensors::PointNormalCloud::Ptr cloud_body_;
    sensors::PointNormalCloud::Ptr cloud_world_;
    sensors::PointNormalCloud::Ptr submap_;

    Eigen::Matrix3d key_rot;
    Eigen::Vector3d key_trans;

    std::shared_ptr<ImuProcess> imu_process_ptr_;
    std::shared_ptr<kf::IESKF> ieskf_;
    // size_t 需要初始化，不然默认值有问题
    size_t frame_count_ = 0;
    size_t key_frame_cout_ = 0;
    std::vector<PointWithCov> point_array_lidar_;

    std::vector<bool> cached_flag_;
    std::vector<FastlioCorrespond> fastlio_corr_cached_;
    std::vector<GICPCorrespond> gicp_corr_cached_;
};

}  // namespace lio