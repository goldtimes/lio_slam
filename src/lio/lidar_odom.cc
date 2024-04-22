#include "lidar_odom.hh"
#define USE_ANALYTICAL_DERIVATE 1

namespace ctlio::slam {
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
        "build local map");

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

}  // namespace ctlio::slam
