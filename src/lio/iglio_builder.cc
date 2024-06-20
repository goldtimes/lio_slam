#include "lio/iglio_builder.hh"

namespace lio {

inline void CauchyLossFunction(const double error, const double delta, Eigen::Vector3d& rho) {
    double delta2 = delta * delta;
    if (error <= delta2) {
        rho[0] = error;
        rho[1] = 1.0;
        rho[2] = 0.0;
    } else {
        double sqrt = std::sqrt(error);
        rho[0] = 2 * sqrt * delta - delta2;
        rho[1] = delta / sqrt;
        rho[2] = -0.5 / rho[1] / error;
    }
}
IGLIOBuilder::IGLIOBuilder(IGLIOParams& params) : params_(params) {
    // 初始化ieskf
    ieskf_ = std::make_shared<kf::IESKF>(params_.ieskf_max_iterations);
    // Todo
    // 初始化 imu_process
    imu_process_ptr_ = std::make_shared<ImuProcess>(ieskf_);
    // 设置imu_process的系统噪声
    Eigen::Matrix3d rot_ext;  // T_I_L
    Eigen::Vector3d trans_ext;
    // clang-format off
    rot_ext << params.imu_rot_ext[0], params.imu_rot_ext[1], params.imu_rot_ext[2], params.imu_rot_ext[3],
        params.imu_rot_ext[4], params.imu_rot_ext[5], params.imu_rot_ext[6], params.imu_rot_ext[7],
        params.imu_rot_ext[8];
    trans_ext << params.imu_ext_pos[0], params.imu_ext_pos[1], params.imu_ext_pos[2];
    // clang-format on

    // 设置imu_process的内参和外参

    // voxel map
    fast_voxel_map_ = std::make_shared<FastVoxelMap>(params_.scan_resolution);
    voxel_map_ = std::make_shared<VoxelMap>(params_.map_resolution, params_.map_capacity, params_.grid_capacity,
                                            params_.nearby_);

    point_array_lidar_.reserve(params_.max_points_in_scan);
    // cached_flag_
    cached_flag_.reserve(params_.max_points_in_scan);
    fastlio_corr_cached_.reserve(params_.max_points_in_scan);
    gicp_corr_cached_.reserve(params_.max_points_in_scan * voxel_map_->searchRange().size());
    // fastlio_corr_cached_
    // gicp_corr_cached_
    cloud_body_.reset(new sensors::PointNormalCloud());
    cloud_lidar_.reset(new sensors::PointNormalCloud());
    cloud_world_.reset(new sensors::PointNormalCloud());
    submap_.reset(new sensors::PointNormalCloud());
}

void IGLIOBuilder::mapping(const MeasureGroup& meas) {
    // 先imu初始化
    // 去畸变
    if (lio_status_ == LIO_STATUS::INITIALIZE) {
        // 第一帧初始化
        cloud_world_ = transformToWorld(cloud_lidar_);
        voxel_map_->addCloud(cloud_world_);
        frame_count_++;
        key_frame_cout_++;
        key_rot = ieskf_->x().rot;
        key_trans = ieskf_->x().pos;
        lio_status_ = LIO_STATUS::MAPPING;
        return;
    }
    // 下一帧进来
    fast_voxel_map_->filter(cloud_lidar_, point_array_lidar_);
    // 做一次lio
    ieskf_->update();
    frame_count_++;
    // 在前几帧的情况下，我们不根据key_rot,key_trans来构建子图
    if (frame_count_ < 10) {
        cloud_world_ = transformToWorld(cloud_lidar_);
        voxel_map_->addCloud(cloud_world_);
        key_rot = ieskf_->x().rot;
        key_trans = ieskf_->x().pos;
        return;
    }
    // 选取关键帧
    if (cloud_lidar_->size() < 1000 || Sophus::SO3d(ieskf_->x().rot.transpose() * key_rot).log().norm() > 0.18 ||
        (ieskf_->x().pos - key_trans).norm() > 0.5) {
        key_frame_cout_++;
        cloud_world_ = transformToWorld(cloud_lidar_);
        voxel_map_->addCloud(cloud_world_);
        key_rot = ieskf_->x().rot;
        key_trans = ieskf_->x().pos;
    }
}

sensors::PointNormalCloud::Ptr IGLIOBuilder::transformToWorld(const sensors::PointNormalCloud::Ptr cloud) {
    sensors::PointNormalCloud::Ptr cloud_world(new sensors::PointNormalCloud());
    Eigen::Matrix3d Rwb = ieskf_->x().rot;
    Eigen::Vector3d twb = ieskf_->x().pos;
    Eigen::Matrix3d R_IL = ieskf_->x().rot_ext;
    Eigen::Vector3d t_IL = ieskf_->x().pos_ext;
    cloud_world->reserve(cloud->size());
    for (const auto& point : cloud->points) {
        Eigen::Vector3d point_lidar(point.x, point.y, point.z);
        Eigen::Vector3d point_body;
        point_body = R_IL * point_lidar + t_IL;
        Eigen::Vector3d point_world;
        point_world = Rwb * point_body + twb;
        sensors::PointNormalType pt_world;
        pt_world.x = point_world(0);
        pt_world.y = point_world(1);
        pt_world.z = point_world(2);
        pt_world.intensity = point.intensity;
        cloud_world->points.push_back(pt_world);
    }
    return cloud_world;
}

void IGLIOBuilder::sharedUpdateFunc(kf::State& state, kf::SharedState& shared_state) {
    if (key_frame_cout_ >= 20) {
        gicpConstraint(state, shared_state);
    } else {
        fastlioConstraint(state, shared_state);
    }
}

void IGLIOBuilder::gicpConstraint(kf::State& state, kf::SharedState& shared_state) {
    int size = point_array_lidar_.size();
    if (shared_state.iter_num < 3) {
        gicp_corr_cached_.clear();
        Eigen::Vector3d mean_B = Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov_B = Eigen::Matrix3d::Zero();
        for (int i = 0; i < size; ++i) {
            Eigen::Vector3d point_lidar = point_array_lidar_[i].point;
            Eigen::Vector3d point_body = state.rot_ext * point_body + state.pos_ext;
            Eigen::Vector3d point_world = state.rot * point_body + state.pos;
            Eigen::Matrix3d cov_A = point_array_lidar_[i].point_cov;
            // 最近邻体素
            for (Eigen::Vector3d& near : voxel_map_->searchRange()) {
                Eigen::Vector3d pw_near = point_world + near;
                if (voxel_map_->getCentroidAndConvariance(pw_near, mean_B, cov_A) &&
                    voxel_map_->isSameGrid(pw_near, mean_B)) {
                    gicp_corr_cached_.emplace_back(point_lidar, mean_B, cov_A, cov_B);
                }
            }
        }
    }
    shared_state.H.setZero();
    shared_state.b.setZero();
    Eigen::Matrix<double, 3, 12> J;
    for (int i = 0; i < gicp_corr_cached_.size(); ++i) {
        GICPCorrespond& gicp_error = gicp_corr_cached_[i];
        Eigen::Vector3d p_lidar = gicp_error.meanA;
        Eigen::Vector3d p_body = state.rot_ext * p_lidar + state.pos_ext;
        Eigen::Vector3d p_world = state.rot * p_body + state.pos;
        Eigen::Matrix3d omega = (gicp_error.covB + state.rot * state.rot_ext * gicp_error.covA *
                                                       state.rot_ext.transpose() * state.rot.transpose())
                                    .inverse();
        Eigen::Vector3d error = gicp_error.meanB - p_world;
        double chi2_error = error.transpose() * omega * error;
        if (shared_state.iter_num > 2 && chi2_error > 7.815) {
            continue;
        }
        Eigen::Vector3d rho;
        CauchyLossFunction(chi2_error, 10.0, rho);
        J.setZero();
        J.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        J.block<3, 3>(0, 3) = state.rot * Sophus::SO3d::hat(p_body);
        if (params_.extrisic_est_en) {
            J.block<3, 3>(0, 6) = state.rot * state.rot_ext * Sophus::SO3d::hat(p_lidar);
            J.block<3, 3>(0, 9) = -state.rot;
        }
        Eigen::Matrix3d robust_information_matrix =
            params_.point2plane_gain * (rho[1] * omega + 2.0 * rho[2] * omega * error * error.transpose() * omega);
        shared_state.H += J.transpose() * robust_information_matrix * J;
        shared_state.b += params_.plane2plane_gain * rho[1] * J.transpose() * omega * error;
    }
    if (gicp_corr_cached_.size() < 1) {
        std::cout << "NO EFFECTIVE POINTS!" << std::endl;
    }
}
// 和fastlio一样 点面的残差
void IGLIOBuilder::fastlioConstraint(kf::State& state, kf::SharedState& shared_state) {
    int size = point_array_lidar_.size();
    if (shared_state.iter_num < 3) {
        for (int i = 0; i < size; ++i) {
            Eigen::Vector3d p_lidar = point_array_lidar_[i].point;
            Eigen::Vector3d p_body = state.rot_ext * p_lidar + state.pos_ext;
            Eigen::Vector3d p_world = state.rot * p_body + state.pos;
            // 找最近的点
            std::vector<Eigen::Vector3d> nearest_points;
            nearest_points.reserve(5);
            voxel_map_->searchKnn(p_world, 5, 5.0, nearest_points);
            // 平面拟合
            Eigen::Vector4d plane_coeffs;
            cached_flag_[i] = false;
            //  平面拟合成功
            if (nearest_points.size() >= 3 && esti_plane(plane_coeffs, nearest_points, 0.1, false)) {
                // 点到平面的距离平方
                double pd2 = plane_coeffs(0) * p_world(0) + plane_coeffs(1) * p_world(1) +
                             plane_coeffs(2) * p_world(2) + plane_coeffs(3);
                // 和点面距离正相关，和点的远近距离负相关
                double s = 1 - 0.9 * std::fabs(pd2) / std::sqrt(point_array_lidar_[i].point.norm());
                if (s > 0.9) {
                    cached_flag_[i] = true;
                    fastlio_corr_cached_[i].point = p_lidar;
                    fastlio_corr_cached_[i].plane(0) = plane_coeffs(0);
                    fastlio_corr_cached_[i].plane(1) = plane_coeffs(1);
                    fastlio_corr_cached_[i].plane(2) = plane_coeffs(2);
                    fastlio_corr_cached_[i].plane(3) = pd2;
                }
            }
        }
    }
    int effect_feat_num = 0;
    shared_state.H.setZero();
    shared_state.b.setZero();
    // p,q,rot_ext, pos_ext
    Eigen::Matrix<double, 1, 12> J;
    for (int i = 0; i < size; ++i) {
        // 找到周围的点并估计平面
        if (!cached_flag_[i]) {
            continue;
        }
        effect_feat_num++;
        J.setZero();
        Eigen::Vector3d norm_vec = fastlio_corr_cached_[i].plane.segment<3>(0);
        double error = fastlio_corr_cached_[i].plane(3);
        Eigen::Vector3d p_lidar = fastlio_corr_cached_[i].point;
        Eigen::Matrix<double, 1, 3> de_dr =
            -norm_vec.transpose() * state.rot * Sophus::SO3d::hat(state.rot_ext * p_lidar + state.pos_ext);
        J.block<1, 3>(0, 0) = norm_vec.transpose();
        J.block<1, 3>(0, 3) = de_dr;
        if (params_.extrisic_est_en) {
            Eigen::Matrix<double, 1, 3> de_drot_ext =
                -norm_vec.transpose() * state.rot * state.rot_ext * Sophus::SO3d::hat(p_lidar);
            Eigen::Matrix<double, 1, 3> de_dpos_ext = norm_vec.transpose() * state.rot_ext;
            J.block<1, 3>(0, 6) = de_drot_ext;
            J.block<1, 3>(0, 9) = de_dpos_ext;
        }
        shared_state.H += J.transpose() * params_.point2plane_gain * J;
        shared_state.b += J.transpose() * params_.point2plane_gain * error;
    }
    if (effect_feat_num < 1) {
        std::cout << "NO_EFFECTIVE POINTS" << std::endl;
    }
}

sensors::PointNormalCloud::Ptr IGLIOBuilder::cloudUndistortedBody() {
    sensors::PointNormalCloud::Ptr cloud_body(new sensors::PointNormalCloud());
    pcl::transformPointCloud(*cloud_lidar_, *cloud_body, ieskf_->x().pos_ext, Eigen::Quaterniond(ieskf_->x().rot_ext));
    return cloud_body;
}
sensors::PointNormalCloud::Ptr IGLIOBuilder::cloudWorld() {
    return transformToWorld(cloud_lidar_);
}

void IGLIOBuilder::reset() {
    lio_status_ = LIO_STATUS::INITIALIZE;
    imu_process_ptr_->reset();
    kf::State state = ieskf_->x();
    state.rot.setIdentity();
    state.pos.setZero();
    state.vel.setZero();
    state.rot_ext.setIdentity();
    state.pos_ext.setZero();
    state.ba.setZero();
    state.bg.setZero();
    ieskf_->change_x(state);
    voxel_map_->reset();
}
}  // namespace lio