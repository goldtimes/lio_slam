#include "lio/imu_process.hh"

namespace lio {

ImuProcess::ImuProcess(const std::shared_ptr<kf::IESKF>& kf)
    : ieskf_(kf),
      init_count_(0),
      last_lidar_time_end_(0.0),
      mean_acc(Eigen::Vector3d::Zero()),
      mean_gyro(Eigen::Vector3d::Zero()),
      last_acc_(Eigen::Vector3d::Zero()),
      last_gyro_(Eigen::Vector3d::Zero()),
      rot_ext(Eigen::Matrix3d::Identity()),
      pos_ext(Eigen::Vector3d::Zero()) {
}

bool ImuProcess::init(const MeasureGroup& meas) {
    if (meas.imudatas.empty()) {
        return init_flag_;
    }
    // 静态初始化，估计重力和角速度bias
    for (const auto& imu : meas.imudatas) {
        init_count_++;
        mean_acc += (imu.acc_ - mean_acc) / init_count_;
        mean_gyro += (imu.gyro_ - mean_gyro) / init_count_;
        // Todo 估计零偏的协方差
    }
    if (init_count_ < max_init_count_) {
        return init_flag_;
    }
    init_flag_ = true;
    // 设置ieskf的初始化状态
    kf::State state = ieskf_->x();
    state.rot_ext = rot_ext;
    state.pos_ext = pos_ext;
    state.bg = mean_gyro;
    // 重力对齐,初始为非水平放置的情况
    if (align_gravity) {
    } else {
    }
    // ieskf_->chang_x(state);

    // 初始化噪声的协方差矩阵
    last_imu_ = meas.imudatas.back();
    return init_flag_;
}

// fastlio中 imu的前向传播和后向传播去畸变
void ImuProcess::undistortPointCloud(const MeasureGroup& group, sensors::PointNormalCloud::Ptr& out) {
    // 静态初始化完成之后才开始去畸变
    // 需要计算一系列的imu姿态
    std::deque<sensors::IMU> imu_tmps(group.imudatas.begin(), group.imudatas.end());
    // 这里为什么需要添加上一帧的imu？
    imu_tmps.push_front(last_imu_);
    const double imu_begin_time = imu_tmps.front().timestamp_;
    const double imu_end_time = imu_tmps.back().timestamp_;
    const double lidar_begin_time = group.lidar_begin_time;
    const double lidar_end_time = group.lidar_end_time;
    // 将点云按时间排序
    sensors::PointNormalCloud::Ptr tmp_cloud(new sensors::PointNormalCloud());
    tmp_cloud = group.lidar_cloud;
    std::sort(tmp_cloud->points.begin(), tmp_cloud->points.end(),
              [](const sensors::PointNormalType& pt1, const sensors::PointNormalType& pt2) {
                  return pt1.curvature < pt2.curvature;
              });
    // 获取当前state状态
    kf::State curr_state = ieskf_->x();
    // 将上一帧的状态记录
    imu_state_.emplace_back(0.0, last_acc_, last_gyro_, curr_state.rot, curr_state.pos, curr_state.vel);

    Eigen::Vector3d acc_val, gyro_val;
    double dt = 0.0;
    Q_.setIdentity();
    kf::Input input;
    for (auto iter_imu = imu_tmps.begin(); iter_imu < (imu_tmps.end() - 1); ++iter_imu) {
        sensors::IMU head = *iter_imu;
        sensors::IMU next = *(iter_imu + 1);
        gyro_val = 0.5 * (head.gyro_ + next.gyro_);
        acc_val = 0.5 * (head.acc_ + next.acc_);
        acc_val = acc_val * 9.81 / mean_acc.norm();
        if (head.timestamp_ < last_lidar_time_end_) {
            // 针对第一帧的情况
            dt = next.timestamp_ - last_lidar_time_end_;
        } else {
            dt = next.timestamp_ - head.timestamp_;
        }
        Q_.block<3, 3>(0, 0).diagonal() = gyro_sigma;
        Q_.block<3, 3>(3, 3).diagonal() = acc_sigma;
        Q_.block<3, 3>(6, 6).diagonal() = gyro_bias_sigma;
        Q_.block<3, 3>(9, 9).diagonal() = acc_bias_sigma;
        input.acc = acc_val;
        input.gyro = gyro_val;
        // 预测
        ieskf_->predict(input, dt, Q_);
        curr_state = ieskf_->x();
        last_gyro_ = gyro_val - curr_state.bg;
        // 转到世界坐标系下
        last_acc_ = curr_state.rot * (acc_val - curr_state.ba);
        last_acc_ += curr_state.g;
        double time_offset = next.timestamp_ - lidar_begin_time;
        imu_state_.emplace_back(time_offset, last_acc_, last_gyro_, curr_state.rot, curr_state.pos, curr_state.vel);
    }
    // 单独处理最后一帧imu的数据
    // fastlio认为最后一帧的imu数据可能小于可能大于lidar_end_time，但是打印时间戳出来
    // 并没有发现大于lidar_end_time
    dt = lidar_end_time - imu_end_time;
    ieskf_->predict(input, dt, Q_);
    last_imu_ = imu_tmps.back();
    last_lidar_time_end_ = lidar_end_time;

    // lidar_end_time时刻的状态量
    curr_state = ieskf_->x();
    Eigen::Matrix3d curr_rot = curr_state.rot;
    Eigen::Vector3d curr_pos = curr_state.pos;
    // 从右往左看所以是雷达在imu下的坐标
    Eigen::Vector3d p_I_L = curr_state.pos_ext;
    Eigen::Matrix3d R_I_L = curr_state.rot_ext;
    // 去畸变
    // end() - 1就是最后一个数据的位置
    auto iter_pcl = tmp_cloud->points.end() - 1;
    for (auto iter = imu_state_.end() - 1; iter != imu_state_.begin(); --iter) {
        auto head = iter - 1;
        auto tail = iter;
        Eigen::Matrix3d imu_rot = head->rot;
        Eigen::Vector3d imu_pos = head->pos;
        Eigen::Vector3d vel = head->pos;
        Eigen::Vector3d acc = tail->acc;
        Eigen::Vector3d gyro = tail->gyro;
        // 我们知道lidar的扫描周期为0.1s,在0.1内产生了十几万个点
        // 我们时间同步后，0.1s内的扫描时间找到了20帧的imu数据
        // 从后往前
        for (; iter_pcl->curvature / double(1000) > head->offset; --iter_pcl) {
            // 点到该imu时刻的dt
            double dt = iter_pcl->curvature / double(1000) - head->offset;
            // 点在lidar坐标系下的坐标
            Eigen::Vector3d point(iter_pcl->x, iter_pcl->y, iter_pcl->z);
            // 我们知道前一刻的imu状态和角速度，加速度，获得雷达点时刻的状态量
            Eigen::Matrix3d point_time_rot = imu_rot * Sophus::SO3d::exp(gyro * dt).matrix();
            Eigen::Vector3d point_time_pos = imu_pos + vel * dt + 0.5 * acc * dt * dt;
            // // T_l_b * T_b_w_j * T_w_b_i * T_b_l * p
            // Eigen::Vector3d p_compensate = cur_rot_ext.transpose() * (cur_rot.transpose() * (point_rot * (cur_rot_ext
            // * point + cur_pos_ext) + point_pos - cur_pos) - cur_pos_ext);

            // 接下来就是补偿到雷达尾部的时刻
            //  R_I_L * L^p_k + p_I_L 将k时刻雷达坐标系中的点转换到imu坐标系下的k时刻的点
            Eigen::Vector3d point_in_I = R_I_L * point + p_I_L;
            // 在将imu坐标系下的点，转换到世界坐标系下
            Eigen::Vector3d point_in_W = point_time_rot * point_in_I + point_time_pos;
            // T_ei i时刻的世界姿态 - 雷达末尾时刻的世界姿态(世界坐标下的补偿量)
            Eigen::Vector3d t_ei = point_in_W - curr_pos;
            // 将该补偿量变换到lidar坐标系下
            Eigen::Vector3d p_compensate = R_I_L.transpose() * (curr_rot.transpose() * t_ei) - p_I_L;
            iter_pcl->x = p_compensate(0);
            iter_pcl->y = p_compensate(1);
            iter_pcl->z = p_compensate(2);
            if (iter_pcl == tmp_cloud->begin()) {
                break;
            }
        }
    }
    // 去畸变完成
    out = tmp_cloud;
}

void ImuProcess::reset() {
    init_count_ = 0;
    init_flag_ = false;
    mean_acc = Eigen::Vector3d::Zero();
    mean_gyro = Eigen::Vector3d::Zero();
}
}  // namespace lio