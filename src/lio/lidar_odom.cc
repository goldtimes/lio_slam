#include "lidar_odom.hh"
#define USE_ANALYTICAL_DERIVATE 1

namespace ctlio::slam {

using pair_distance_t = std::tuple<double, Eigen::Vector3d, Voxel>;
struct comparator {
    bool operator()(const pair_distance_t& left, const pair_distance_t& right) const {
        return std::get<0>(left) < std::get<0>(right);
    }
};

using priority_queue_t = std::priority_queue<pair_distance_t, std::vector<pair_distance_t>, comparator>;

LidarOdom::LidarOdom() {
    lidar_point_cov = 0.001;
    index_frame = 1;
    // 设置信息矩阵
    points_world.reset(new PointCloudType);
}
LidarOdom::~LidarOdom() {
}

void LidarOdom::loadOptions() {
    auto yaml = YAML::LoadFile(config_yaml_);
    options_.surf_res = yaml["odometry"]["surf_res"].as<double>();
    options_.log_print = yaml["odometry"]["log_print"].as<bool>();
    options_.max_num_iteration = yaml["odometry"]["max_num_iteration"].as<int>();

    options_.size_voxel_map = yaml["odometry"]["size_voxel_map"].as<double>();
    options_.min_distance_points = yaml["odometry"]["min_distance_points"].as<double>();
    options_.max_num_points_in_voxel = yaml["odometry"]["max_num_points_in_voxel"].as<int>();
    options_.max_distance = yaml["odometry"]["max_distance"].as<double>();
    options_.weight_alpha = yaml["odometry"]["weight_alpha"].as<double>();
    options_.weight_neighborhood = yaml["odometry"]["weight_neighborhood"].as<double>();
    options_.max_dist_to_plane_icp = yaml["odometry"]["max_dist_to_plane_icp"].as<double>();
    options_.init_num_frames = yaml["odometry"]["init_num_frames"].as<int>();
    options_.voxel_neighborhood = yaml["odometry"]["voxel_neighborhood"].as<int>();
    options_.max_number_neighbors = yaml["odometry"]["max_number_neighbors"].as<int>();
    options_.threshold_voxel_occupancy = yaml["odometry"]["threshold_voxel_occupancy"].as<int>();
    options_.estimate_normal_from_neighborhood = yaml["odometry"]["estimate_normal_from_neighborhood"].as<bool>();
    options_.min_number_neighbors = yaml["odometry"]["min_number_neighbors"].as<int>();
    options_.power_planarity = yaml["odometry"]["power_planarity"].as<double>();
    options_.num_closest_neighbors = yaml["odometry"]["num_closest_neighbors"].as<int>();

    options_.sampling_rate = yaml["odometry"]["sampling_rate"].as<double>();
    options_.ratio_of_nonground = yaml["odometry"]["ratio_of_nonground"].as<double>();
    options_.max_num_residuals = yaml["odometry"]["max_num_residuals"].as<int>();
    std::string str_motion_compensation = yaml["odometry"]["motion_compensation"].as<std::string>();
    if (str_motion_compensation == "NONE")
        options_.motion_compensation = MotionCompensation::NONE;
    else if (str_motion_compensation == "CONSTANT_VELOCITY")
        options_.motion_compensation = MotionCompensation::CONSTANT_VELOCITY;
    else if (str_motion_compensation == "ITERATIVE")
        options_.motion_compensation = MotionCompensation::ITERATIVE;
    else if (str_motion_compensation == "CONTINUOUS")
        options_.motion_compensation = MotionCompensation::CONTINUOUS;
    else
        std::cout << "The `motion_compensation` " << str_motion_compensation << " is not supported." << std::endl;

    std::string str_icpmodel = yaml["odometry"]["icpmodel"].as<std::string>();
    if (str_icpmodel == "POINT_TO_PLANE")
        options_.icpmodel = IcpModel::POINT_TO_PLANE;
    else if (str_icpmodel == "CT_POINT_TO_PLANE")
        options_.icpmodel = IcpModel::CT_POINT_TO_PLANE;
    else
        std::cout << "The `icp_residual` " << str_icpmodel << " is not supported." << std::endl;

    options_.beta_location_consistency = yaml["odometry"]["beta_location_consistency"].as<double>();
    options_.beta_orientation_consistency = yaml["odometry"]["beta_orientation_consistency"].as<double>();
    options_.beta_constant_velocity = yaml["odometry"]["beta_constant_velocity"].as<double>();
    options_.beta_small_velocity = yaml["odometry"]["beta_small_velocity"].as<double>();
    options_.thres_orientation_norm = yaml["odometry"]["thres_orientation_norm"].as<double>();
    options_.thres_translation_norm = yaml["odometry"]["thres_translation_norm"].as<double>();
}

bool LidarOdom::init(const std::string& config_file) {
    config_yaml_ = config_file;
    StaticInitImu::StaticOptions static_init_options;
    static_init_options.use_speed_for_static_checking_ = false;
    imu_init_ = std::make_shared<StaticInitImu>(static_init_options);
    auto yaml = YAML::LoadFile(config_file);
    // lidar和imu的时间延迟
    delay_time = yaml["delay_time"].as<double>();
    std::vector<double> extrinsic_t = yaml["mapping"]["extrinsic_T"].as<std::vector<double>>();
    std::vector<double> extrinsic_r = yaml["mapping"]["extrinsic_R"].as<std::vector<double>>();
    // lidar->imu的外参
    Vec3d lidar_T_wrt_imu = math::VecFromArray(extrinsic_t);
    Mat3d lidar_R_wrt_imu = math::Mat3FromArray(extrinsic_r);
    // 下标从右到左I为父坐标系,L为child frame
    Eigen::Quaterniond q_IL(lidar_R_wrt_imu);
    q_IL.normalize();

    T_LtoI_ = SE3(q_IL, lidar_T_wrt_imu);
    R_LtoI = lidar_R_wrt_imu;
    t_LtoI = lidar_T_wrt_imu;

    // 设置factor的外参
    //
    loadOptions();
    switch (options_.motion_compensation) {
        case MotionCompensation::NONE:
        case MotionCompensation::CONSTANT_VELOCITY:
            options_.point_to_plane_with_distortion = false;
            options_.icpmodel = IcpModel::POINT_TO_PLANE;
            break;
        case MotionCompensation::ITERATIVE:
            options_.point_to_plane_with_distortion = true;
            options_.icpmodel = IcpModel::POINT_TO_PLANE;
            break;
        case MotionCompensation::CONTINUOUS:
            options_.point_to_plane_with_distortion = true;
            options_.icpmodel = IcpModel::CT_POINT_TO_PLANE;
            break;
    }
    // LOG(WARNING) << "motion_compensation:" << options_.motion_compensation << ", model: " << options_.icpmodel;
    return true;
}

void LidarOdom::pushData(const std::vector<point3D>& points, std::pair<double, double> data) {
    if (data.first < last_timestamped_lidar_) {
        LOG(ERROR) << "lidar loop back, clean buffer";
        lidar_buffer_.clear();
        time_buffer_.clear();
    }
    std::lock_guard<std::mutex> lck(mtx_buffer);
    lidar_buffer_.push_back(points);
    time_buffer_.push_back(data);
    last_timestamped_lidar_ = data.first;
    cond.notify_one();
}

void LidarOdom::pushData(IMUPtr imu) {
    double timestamp = imu->timestamped_;
    if (timestamp < last_timestamped_imu_) {
        LOG(ERROR) << "imu loop back, clean buffer";
        imu_buffer_.clear();
    }
    std::lock_guard<std::mutex> lck(mtx_buffer);
    imu_buffer_.emplace_back(imu);
    cond.notify_one();
}
void LidarOdom::run() {
    while (true) {
        // 获取时间同步的数据
        std::vector<MeasureGroup> groups;
        std::unique_lock<std::mutex> lck(mtx_buffer);
        // 一直等
        cond.wait(lck, [&] { return (groups = getMeasureMents()).size() != 0; });
        lck.unlock();
        // 处理数据
        for (auto& group : groups) {
            Timer::Evaluate([&]() { ProcessMeasurements(group); }, "process Measurement");
            auto real_time = std::chrono::high_resolution_clock::now();
            static std::chrono::system_clock::time_point prev_real_time = real_time;
            if (real_time - prev_real_time > std::chrono::seconds(5)) {
                auto data_time = group.lidar_end_time;
                static double prev_data_time = data_time;
                auto delta_real =
                    std::chrono::duration_cast<std::chrono::milliseconds>(real_time - prev_real_time).count() * 0.001;
                auto delta_sim = data_time - prev_data_time;
                printf("Processing the rosbag at %.1fX speed.", delta_sim / delta_real);

                prev_data_time = data_time;
                prev_real_time = real_time;
            }
        }
    }
}

std::vector<MeasureGroup> LidarOdom::getMeasureMents() {
    std::vector<MeasureGroup> measurements;
    while (true) {
        // 数据为空
        if (imu_buffer_.empty() || lidar_buffer_.empty()) {
            return measurements;
        }

        if (imu_buffer_.back()->timestamped_ - time_curr < delay_time) {
            return measurements;
        }

        MeasureGroup meas;
        meas.lidar_ = lidar_buffer_.front();
        meas.lidar_begin_time = time_buffer_.front().first;
        meas.lidar_end_time = meas.lidar_begin_time + time_buffer_.front().second;
        lidar_buffer_.pop_front();
        time_buffer_.pop_front();

        time_curr = meas.lidar_end_time;
        double imu_start_time = imu_buffer_.front()->timestamped_;
        meas.imus_.clear();
        while (!imu_buffer_.empty() && (imu_start_time < meas.lidar_end_time)) {
            imu_start_time = imu_buffer_.front()->timestamped_;
            if (imu_start_time > meas.lidar_end_time) {
                break;
            } else {
                meas.imus_.push_back(imu_buffer_.front());
                imu_buffer_.pop_front();
            }
        }
        if (!imu_buffer_.empty()) {
            meas.imus_.push_back(imu_buffer_.front());
        }
        measurements.push_back(meas);
    }
}

void LidarOdom::ProcessMeasurements(MeasureGroup& meas) {
    measure_ = meas;
    // 需要imu初始化
    if (imu_need_init_) {
        TryInitIMU();
        return;
    }
    std::cout << ANSI_DELETE_LAST_LINE;
    std::cout << ANSI_COLOR_GREEN << "============== process frame: " << index_frame << ANSI_COLOR_RESET << std::endl;
    imu_states_.clear();  //   need clear here

    // imu状态预测
    Timer::Evaluate([&]() { Predict(); }, "eskf predict");
    // 更新当前状态
    stateInitialization();
    std::vector<point3D> const_surf;
    const_surf.insert(const_surf.begin(), meas.lidar_.begin(), meas.lidar_.end());

    CloudFrame* p_frame;
    Timer::Evaluate(
        [&]() { p_frame = buildFrame(const_surf, current_state, meas.lidar_begin_time, meas.lidar_end_time); },
        "build frame");

    // scan_to_submap
    Timer::Evaluate([&] { poseEstimation(p_frame); }, "pose  estimation");

    // 松耦合的观测更新
    SE3 pose_of_lo = SE3(current_state->rotation, current_state->translation);
    Timer::Evaluate([&] { eskf_.ObserveSE3(pose_of_lo, 1e-2, 1e-2); }, "observeSE3");

    // 可视化发布
    Timer::Evaluate(
        [&] {
            std::string laser_topic = "laser";
            pub_pose_to_ros(laser_topic, pose_of_lo, meas.lidar_end_time);
            std::string topic = "velocity";
            SE3 pred_pose = eskf_.GetSE3();
            Eigen::Vector3d vel_world = eskf_.GetV();
            Eigen::Vector3d vel_body = pred_pose.rotation_matrix().inverse() * vel_world;
            pub_data_to_ros(topic, vel_body.x(), 0);
            if (index_frame % 8 == 0) {
                static Eigen::Vector3d last_t = Eigen::Vector3d::Zero();
                static double dist = 0;
                topic = "dist";
                Eigen::Vector3d t = pred_pose.translation();
                dist += (t - last_t).norm();
                last_t = t;
                pub_data_to_ros(topic, dist, 0);
            }
        },
        "visiualization");
    p_frame->p_state = new State(current_state, true);
    State* tmp_state = new State(current_state, true);
    all_state_frame.push_back(tmp_state);
    current_state = new State(current_state, false);

    index_frame++;
    p_frame->release();
    // 清空
    std::vector<point3D>().swap(meas.lidar_);
    std::vector<point3D>().swap(const_surf);
}

void LidarOdom::TryInitIMU() {
    for (auto imu : measure_.imus_) {
        imu_init_->AddImu(*imu);
    }
    // 初始化成功
    if (imu_init_->InitSuccess()) {
        ESKFD::Options eskf_options;
        eskf_.SetInitailConditions(eskf_options, imu_init_->GetInitBg(), imu_init_->GetInitBa(),
                                   imu_init_->GetGravity());
        imu_need_init_ = false;
    }
}

void LidarOdom::Predict() {
    imu_states_.emplace_back(eskf_.GetState());
    double curr_time = measure_.lidar_end_time;
    Vec3d last_gyro, last_acc;
    for (auto& imu : measure_.imus_) {
        double time_imu = imu->timestamped_;
        if (imu->timestamped_ <= curr_time) {
            if (last_imu_ == nullptr) {
                last_imu_ = imu;
            }
            eskf_.Predict(*imu);
            imu_states_.push_back(eskf_.GetState());
            last_imu_ = imu;
        } else {
            double dt_1 = time_imu - curr_time;
            double dt_2 = curr_time - last_imu_->timestamped_;
            double w1 = dt_1 / (dt_2 + dt_1);
            double w2 = dt_2 / (dt_2 + dt_1);
            Eigen::Vector3d acc_aver = w1 * last_imu_->acce_ + w2 * imu->acce_;
            Eigen::Vector3d gyro_aver = w1 * last_imu_->gyro_ + w2 * imu->gyro_;
            IMUPtr imu_tmp = std::make_shared<IMU>(curr_time, gyro_aver, acc_aver);
            eskf_.Predict(*imu_tmp);
            imu_states_.emplace_back(eskf_.GetState());
            last_imu_ = imu_tmp;
        }
    }
}

void LidarOdom::stateInitialization() {
    // first frame
    if (index_frame < 2) {
        current_state->rotation_begin = Eigen::Quaterniond(imu_states_.front().R_.matrix());
        current_state->translation_begin = imu_states_.front().p_;
        current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
        current_state->translation = imu_states_.back().p_;
    } else {
        // 上一帧的pose
        current_state->rotation_begin = all_state_frame[all_state_frame.size() - 1]->rotation;
        current_state->translation_begin = all_state_frame[all_state_frame.size() - 1]->translation;
        // imu的预测
        current_state->rotation = Eigen::Quaterniond(imu_states_.back().R_.matrix());
        current_state->translation = imu_states_.back().p_;
    }
}

CloudFrame* LidarOdom::buildFrame(std::vector<point3D>& const_surf, State* cur_state, double time_begin,
                                  double time_end) {
    std::vector<point3D> frame_surf(const_surf);
    if (index_frame < 2) {
        // 第一帧
        for (auto& point_tmp : frame_surf) {
            point_tmp.alpha_time = 1.0;
        }
    }
    // 匀速运动去畸变
    if (options_.motion_compensation == MotionCompensation::CONSTANT_VELOCITY) {
        // Undistort(frame_surf);
    }
    // 用imu去畸变
    for (auto& point_tmp : frame_surf) {
        transformPoint(options_.motion_compensation, point_tmp, current_state->rotation_begin, current_state->rotation,
                       current_state->translation_begin, current_state->translation, R_LtoI, t_LtoI);
    }
    CloudFrame* frame = new CloudFrame(frame_surf, const_surf, cur_state);
    frame->time_frame_begin = time_begin;
    frame->time_frame_end = time_end;
    frame->dt_offset = 0;
    frame->frame_id = index_frame;
    return frame;
}

void LidarOdom::poseEstimation(CloudFrame* frame) {
    // 不是第一帧
    if (index_frame > 1) {
        Timer::Evaluate([&]() { optimize(frame); }, "optimize");
    }
    bool add_points = true;
    if (add_points) {
        // build local map
        Timer::Evaluate([&]() { map_incremental(frame); }, "map update");
    }
    //
    Timer::Evaluate([&]() { lasermap_fov_segment(); }, "lasermap_fov_segment");
}

void LidarOdom::optimize(CloudFrame* frame) {
    State* previous_state = nullptr;
    Eigen::Quaterniond previous_orien = Eigen::Quaterniond::Identity();
    Eigen::Vector3d previous_vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d previous_tras = Eigen::Vector3d::Zero();

    State* current_state = frame->p_state;
    Eigen::Quaterniond begin_quad = current_state->rotation_begin;
    Eigen::Quaterniond end_quad = current_state->rotation;
    Eigen::Vector3d begin_trans = current_state->translation_begin;
    Eigen::Vector3d end_trans = current_state->translation;

    if (frame->frame_id > 1) {
        previous_state = all_state_frame[frame->frame_id - 2];
        previous_tras = previous_state->translation;
        previous_orien = previous_state->rotation;
        previous_vel = previous_state->translation - previous_state->translation_begin;
    }
    if (options_.log_print) {
    }
    std::vector<point3D> surf_keypoints;
    gridSampling(frame->point_surf, surf_keypoints, options_.sampling_rate * options_.surf_res);
    size_t num_points = frame->point_surf.size();
    // 已经去畸变了
    for (int iter = 0; iter < options_.max_num_iteration; ++iter) {
        ceres::LossFunction* loss_function = new ceres::HuberLoss(0.5);
        ceres::Problem::Options problem_options;
        ceres::Problem problem(problem_options);
#ifdef USE_ANALYTICAL_DERIVATE
        ceres::LocalParameterization* parameterization = new RotationParameterization();
#else
        auto* parameterization = ceres::EigenQuaternionParameterization();
#endif
        switch (options_.icpmodel) {
            case IcpModel::CT_POINT_TO_PLANE:
                problem.AddParameterBlock(&begin_quad.x(), 4, parameterization);
                problem.AddParameterBlock(&end_quad.x(), 4, parameterization);
                problem.AddParameterBlock(&begin_trans.x(), 3);
                problem.AddParameterBlock(&end_trans.x(), 3);
                break;
            case IcpModel::POINT_TO_PLANE:
                problem.AddParameterBlock(&end_quad.x(), 4, parameterization);
                problem.AddParameterBlock(&end_trans.x(), 3);
                break;
        }
        std::vector<ceres::CostFunction*> surf_cost_function;
        std::vector<Eigen::Vector3d> normal_vec;
        addSurfCostFactor(surf_cost_function, normal_vec, surf_keypoints, frame);
        //   TODO: 退化后，该如何处理
        checkLocalizability(normal_vec);
        int surf_num = 0;
        std::cout << "get factor:" << surf_cost_function.size() << std::endl;
        for (auto& error : surf_cost_function) {
            surf_num++;
            switch (options_.icpmodel) {
                case IcpModel::CT_POINT_TO_PLANE:
                    problem.AddResidualBlock(error, loss_function, &begin_trans.x(), &begin_quad.x(), &end_trans.x(),
                                             &end_quad.x());
                    break;
                case IcpModel::POINT_TO_PLANE:
                    problem.AddResidualBlock(error, loss_function, &end_trans.x(), &end_quad.x());
                    break;
            }
        }
        //   release
        std::vector<Eigen::Vector3d>().swap(normal_vec);
        std::vector<ceres::CostFunction*>().swap(surf_cost_function);
        if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE) {
            if (options_.beta_location_consistency > 0.0) {
#ifdef USE_ANALYTICAL_DERIVATE
                LocationConsistencyFactor* cost_location_consistency = new LocationConsistencyFactor(
                    previous_tras, std::sqrt(surf_num * options_.beta_location_consistency * lidar_point_cov));
#else
#endif
                problem.AddResidualBlock(cost_location_consistency, nullptr, &begin_trans.x());
            }
            if (options_.beta_orientation_consistency > 0.0) {
                RotationConsistencyFactor* cost_rotation_consistency = new RotationConsistencyFactor(
                    previous_orien, std::sqrt(surf_num * options_.beta_constant_velocity * lidar_point_cov));
                problem.AddResidualBlock(cost_rotation_consistency, nullptr, &begin_quad.x());
            }
            if (options_.beta_small_velocity > 0.0) {
                SmallVelocityFactor* cost_small_velocity =
                    new SmallVelocityFactor(std::sqrt(surf_num * options_.beta_constant_velocity * lidar_point_cov));
                problem.AddResidualBlock(cost_small_velocity, nullptr, &begin_trans.x(), &end_trans.x());
            }
        }
        if (surf_num < options_.min_num_residuals) {
            std::stringstream ss_out;
            ss_out << "[Optimization] Error : not enough keypoints selected in ct-icp!" << std::endl;
            ss_out << "[Optimization] number_of_residuals:" << surf_num << std::endl;
            std::cout << "Error: " << ss_out.str();
        }

        ceres::Solver::Options options;
        options.max_num_iterations = 5;
        options.num_threads = 3;
        options.minimizer_progress_to_stdout = false;
        options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        if (!summary.IsSolutionUsable()) {
            std::cout << summary.FullReport() << std::endl;
            throw std::runtime_error("Error During Optimization");
        }
        begin_quad.normalize();
        end_quad.normalize();

        double diff_trans = 0, diff_rot = 0;
        diff_trans += (current_state->translation_begin - begin_trans).norm();
        diff_rot += AngularDistance(current_state->rotation_begin, begin_quad);

        diff_trans += (current_state->translation - end_trans).norm();
        diff_rot += AngularDistance(current_state->rotation, end_quad);

        if (options_.icpmodel == IcpModel::CT_POINT_TO_PLANE) {
            frame->p_state->translation_begin = begin_trans;
            frame->p_state->rotation_begin = begin_quad;
            frame->p_state->translation = end_trans;
            frame->p_state->rotation = end_quad;

            current_state->translation_begin = begin_trans;
            current_state->rotation_begin = begin_quad;
            current_state->translation = end_trans;
            current_state->rotation = end_quad;
        }
        if (diff_rot < options_.thres_orientation_norm && diff_trans < options_.thres_translation_norm) {
            if (options_.log_print) {
                std::cout << "Optimization: Finished with N=" << iter << "ICP iterations" << std::endl;
                break;
            }
        }
    }
    std::vector<point3D>().swap(surf_keypoints);
    // transformKeyPoints(frame->point_surf);
}

void LidarOdom::addSurfCostFactor(std::vector<ceres::CostFunction*>& surf, std::vector<Eigen::Vector3d>& normals,
                                  std::vector<point3D>& keypoints, const CloudFrame* p_frame) {
    auto estimatePointNeighborhood =
        [&](std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& vector_neightbors,
            Eigen::Vector3d& location, double& planarity_weight) {
            auto neightborhood = computeNeighborhoodDistribution(vector_neightbors);
            planarity_weight = std::pow(neightborhood.a2D, options_.power_planarity);
            if (neightborhood.normal.dot(p_frame->p_state->translation_begin - location) < 0.0) {
                neightborhood.normal = -1.0 * neightborhood.normal;
            }
            return neightborhood;
        };

    double lambda_weight = std::abs(options_.weight_alpha);
    double lambda_neighborhood = std::abs(options_.weight_neighborhood);
    const double kMaxPointToPlan = options_.max_dist_to_plane_icp;
    const double sum = lambda_weight + lambda_neighborhood;

    lambda_weight /= sum;
    lambda_neighborhood /= sum;

    const short nb_voxels_visited = p_frame->frame_id < options_.init_num_frames ? 2 : options_.voxel_neighborhood;
    const int kThresholdCapacity =
        p_frame->frame_id < options_.init_num_frames ? 1 : options_.threshold_voxel_occupancy;

    size_t num = keypoints.size();
    int num_residuals = 0;
    for (int k = 0; k < num; ++k) {
        auto& keypoint = keypoints[k];
        auto& raw_point = keypoint.raw_point;
        std::vector<Voxel> voxels;
        auto vector_neighbors = searchNeighbors(voxel_map, keypoint.point, nb_voxels_visited, options_.size_voxel_map,
                                                options_.max_number_neighbors, kThresholdCapacity,
                                                options_.estimate_normal_from_neighborhood ? nullptr : &voxels);
        if (vector_neighbors.size() < options_.min_number_neighbors) {
            continue;
        }
        double weight;
        Eigen::Vector3d point_world = T_LtoI_ * raw_point;
        auto neightborhood = estimatePointNeighborhood(vector_neighbors, point_world, weight);

        weight =
            lambda_weight * weight + lambda_neighborhood * std::exp(-(vector_neighbors[0] - keypoint.point).norm() /
                                                                    (kMaxPointToPlan * options_.min_number_neighbors));
        double point_to_plane_dist;
        std::set<Voxel> neightbor_voxels;
        for (int i = 0; i < options_.num_closest_neighbors; ++i) {
            point_to_plane_dist = std::abs((keypoint.point - vector_neighbors[i]).transpose() * neightborhood.normal);
            if (point_to_plane_dist < options_.max_dist_to_plane_icp) {
                num_residuals++;
                Eigen::Vector3d norm_vector = neightborhood.normal;
                norm_vector.normalize();
                normals.push_back(norm_vector);
                double norm_offset = -norm_vector.dot(vector_neighbors[i]);
                switch (options_.icpmodel) {
                    case IcpModel::CT_POINT_TO_PLANE:
                        // CTLidarPlaneNormFactor* cost_function = new CTLidarPlaneNormFactor(
                        //     keypoints[k].raw_point, norm_vector, norm_offset, keypoints[k].alpha_time, weight);
                        // surf.push_back(cost_function);
                        break;

                    case IcpModel::POINT_TO_PLANE:
                        // Eigen::Vector3d point_end =
                        //     p_frame->p_state->rotation.inverse() * keypoints[k].point -
                        //     p_frame->p_state->rotation.inverse() * p_frame->p_state->translation;
                        // LidarPlaneNormFactor* cost_function =
                        //     new LidarPlaneNormFactor(point_end, norm_vector, norm_offset, weight);
                        // surf.push_back(cost_function);
                        break;
                }
            }
        }
        if (num_residuals >= options_.max_num_iteration) {
            break;
        }
    }
}

double LidarOdom::checkLocalizability(std::vector<Eigen::Vector3d> planeNormals) {
    return 0.0;
}

Neighborhood LidarOdom::computeNeighborhoodDistribution(
    const std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>& points) {
    Neighborhood neightborhood;
    Eigen::Vector3d barycenter(Eigen::Vector3d(0, 0, 0));
    for (auto& point : points) {
        barycenter += point;
    }
    barycenter /= (double)points.size();
    neightborhood.center = barycenter;
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (auto& point : points) {
        for (int i = 0; i < 3; ++i) {
            for (int j = i; j < 3; ++j) {
                cov(i, j) += (point(i) - barycenter(i)) * (point(j) - barycenter(j));
            }
        }
    }
    cov(1, 0) = cov(0, 1);
    cov(2, 0) = cov(0, 2);
    cov(2, 1) = cov(1, 2);
    neightborhood.covariance = cov;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
    Eigen::Vector3d normal(es.eigenvectors().col(0).normalized());
    neightborhood.normal = normal;

    double sigma_1 = sqrt(std::abs(es.eigenvalues()[2]));
    double sigma_2 = sqrt(std::abs(es.eigenvalues()[1]));
    double sigma_3 = sqrt(std::abs(es.eigenvalues()[0]));
    neightborhood.a2D = (sigma_2 - sigma_3) / sigma_1;
    if (neightborhood.a2D != neightborhood.a2D) {
        throw std::runtime_error("error");
    }
    return neightborhood;
}

std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> LidarOdom::searchNeighbors(
    const VoxelHashMap& map, const Eigen::Vector3d& point, int nb_voxels_visited, double size_voxel_map,
    int max_num_neighbors, int threshold_voxel_capacity, std::vector<Voxel>* voxels) {
    if (voxels != nullptr) {
        voxels->reserve(max_num_neighbors);
    }
    short kx = static_cast<short>(point[0] / size_voxel_map);
    short ky = static_cast<short>(point[1] / size_voxel_map);
    short kz = static_cast<short>(point[2] / size_voxel_map);

    priority_queue_t priority_queue;
    Voxel voxel_temp(kx, ky, kz);
    for (short kxx = kx - nb_voxels_visited; kxx < kx + nb_voxels_visited + 1; ++kxx) {
        for (short kyy = ky - nb_voxels_visited; kyy < ky + nb_voxels_visited + 1; ++kyy) {
            for (short kzz = kz - nb_voxels_visited; kzz < kz + nb_voxels_visited + 1; ++kzz) {
                voxel_temp.x = kxx;
                voxel_temp.y = kyy;
                voxel_temp.z = kzz;
                auto search = map.find(voxel_temp);
                if (search != map.end()) {
                    const auto& voxel_block = search.value();
                    if (voxel_block.NumPoints() < threshold_voxel_capacity) {
                        continue;
                    }
                    for (int i = 0; i < voxel_block.NumPoints(); ++i) {
                        auto& neighbor = voxel_block.points[i];
                        double distance = (neighbor - point).norm();
                        if (priority_queue.size() == max_num_neighbors) {
                            if (distance < std::get<0>(priority_queue.top())) {
                                priority_queue.pop();
                                priority_queue.emplace(distance, neighbor, voxel_temp);
                            }
                        } else {
                            priority_queue.emplace(distance, neighbor, voxel_temp);
                        }
                    }
                }
            }
        }
    }
    auto size = priority_queue.size();
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> closeset_neightbors(size);
    if (voxels != nullptr) {
        voxels->resize(size);
    }
    for (int i = 0; i < size; ++i) {
        closeset_neightbors[size - 1 - i] = std::get<1>(priority_queue.top());
        if (voxels != nullptr) {
            (*voxels)[size - 1 - i] = std::get<2>(priority_queue.top());
            priority_queue.pop();
        }
    }
    return closeset_neightbors;
}

void LidarOdom::map_incremental(CloudFrame* frame, int min_num_points) {
    for (const auto& point : frame->point_surf) {
        addPointToMap(voxel_map, point.point, point.intensity, options_.size_voxel_map,
                      options_.max_num_points_in_voxel, options_.min_distance_points, min_num_points, frame);
    }
    std::string topic = "laser";
    pub_cloud_to_ros(topic, points_world, frame->time_frame_end);
    points_world->clear();
}

void LidarOdom::lasermap_fov_segment() {
    Eigen::Vector3d location = current_state->translation;
    std::vector<Voxel> voxels_to_erase;
    for (auto& pair : voxel_map) {
        // 体素中点云中的第一个点
        Eigen::Vector3d pt = pair.second.points[0];
        if ((pt - location).squaredNorm() > options_.max_distance * options_.max_distance) {
            // 将需要删除的点云存放起来
            voxels_to_erase.push_back(pair.first);
        }
    }
    for (auto& voxel : voxels_to_erase) {
        voxel_map.erase(voxel);
    }
    // voxels_to_erase
    std::vector<Voxel>().swap(voxels_to_erase);
}

void LidarOdom::addPointToMap(VoxelHashMap& map, const Eigen::Vector3d& point, const double& intensity,
                              double voxel_size, int max_num_points_in_voxel, double min_distance_points,
                              int min_num_points, CloudFrame* p_frame) {
    short kx = static_cast<short>(point[0] / voxel_size);
    short ky = static_cast<short>(point[1] / voxel_size);
    short kz = static_cast<short>(point[2] / voxel_size);
    VoxelHashMap::iterator iter = map.find(Voxel(kx, ky, kz));
    if (iter != map.end()) {
        auto& voxel_block = iter.value();
        if (!voxel_block.IsFull()) {
            double sq_dist_min_to_points = 10 * voxel_size * voxel_size;
            for (int i = 0; i < voxel_block.NumPoints(); ++i) {
                auto& _point = voxel_block.points[i];
                double sq_dist = (_point - point).squaredNorm();
                if (sq_dist < sq_dist_min_to_points) {
                    sq_dist_min_to_points = sq_dist;
                }
            }
            if (sq_dist_min_to_points > min_distance_points * min_distance_points) {
                if (min_num_points <= 0 || voxel_block.NumPoints() >= min_num_points) {
                    voxel_block.AddPoint(point);
                }
            }
        }
    } else {
        if (min_num_points <= 0) {
            VoxelBlock block(max_num_points_in_voxel);
            block.AddPoint(point);
            map[Voxel(kx, ky, kz)] = std::move(block);
        }
    }
    addPointToPCL(points_world, point, intensity, p_frame);
}

void LidarOdom::addPointToPCL(CloudPtr pcl_points, const Eigen::Vector3d& point, const double& intensity,
                              CloudFrame* p_frame) {
    pcl::PointXYZI cloudTemp;

    cloudTemp.x = point.x();
    cloudTemp.y = point.y();
    cloudTemp.z = point.z();
    cloudTemp.intensity = intensity;
    // cloudTemp.intensity = 50 * (point.z() - p_frame->p_state->translation.z());
    pcl_points->points.push_back(cloudTemp);
}

void LidarOdom::setFunc(std::function<bool(std::string& topic_name, CloudPtr& cloud, double time)>& func) {
    pub_cloud_to_ros = func;
}
void LidarOdom::setFunc(std::function<bool(std::string& topic_name, SE3& pose, double time)>& func) {
    pub_pose_to_ros = func;
}
void LidarOdom::setFunc(std::function<bool(std::string& topic_name, double time1, double time2)>& func) {
    pub_data_to_ros = func;
}

}  // namespace ctlio::slam
