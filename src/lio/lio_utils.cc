#include "lio_utils.hh"

namespace ctlio::slam {

State::State(const Eigen::Quaterniond &r, const Eigen::Vector3d &trans, const Eigen::Vector3d &vel,
             const Eigen::Vector3d &bg, const Eigen::Vector3d &ba)
    : rotation(r), translation(trans), velocity(vel), ba(ba), bg(bg) {
}
State::State(const State *state_temp, bool copy) {
    rotation = state_temp->rotation;
    translation = state_temp->translation;

    rotation_begin = state_temp->rotation_begin;
    translation_begin = state_temp->translation_begin;

    velocity = state_temp->velocity;
    ba = state_temp->ba;
    bg = state_temp->bg;

    velocity_begin = state_temp->velocity_begin;
    ba_begin = state_temp->ba_begin;
    bg_begin = state_temp->bg_begin;
}

void State::release() {
}

CloudFrame::CloudFrame(std::vector<point3D> &point_surf_, std::vector<point3D> &const_surf_, State *p_state) {
    point_surf.insert(point_surf.end(), point_surf_.begin(), point_surf_.end());
    const_surf.insert(const_surf.end(), const_surf_.begin(), const_surf_.end());

    // p_state = p_state_;
    p_state = new State(p_state, true);

    success = true;
}
CloudFrame::CloudFrame(CloudFrame *p_cloud_frame) {
    time_frame_begin = p_cloud_frame->time_frame_begin;
    time_frame_end = p_cloud_frame->time_frame_end;
    frame_id = p_cloud_frame->frame_id;

    // p_state = p_cloud_frame->p_state;
    p_state = new State(p_cloud_frame->p_state, true);
    point_surf.insert(point_surf.end(), p_cloud_frame->point_surf.begin(), p_cloud_frame->point_surf.end());
    const_surf.insert(const_surf.end(), p_cloud_frame->const_surf.begin(), p_cloud_frame->const_surf.end());

    dt_offset = p_cloud_frame->dt_offset;

    success = p_cloud_frame->success;
}
void CloudFrame::release() {
    std::vector<point3D>().swap(point_surf);
    std::vector<point3D>().swap(const_surf);

    p_state = nullptr;
}

void transformPoint(MotionCompensation compensation, point3D &point_temp, Eigen::Quaterniond &q_begin,
                    Eigen::Quaterniond &q_end, Eigen::Vector3d &t_begin, Eigen::Vector3d &t_end,
                    Eigen::Matrix3d &R_imu_lidar, Eigen::Vector3d &t_imu_lidar) {
    Eigen::Vector3d t;
    Eigen::Matrix3d R;
    double alpha_time = point_temp.alpha_time;
    switch (compensation) {
        case MotionCompensation::NONE:
        // 匀速
        case MotionCompensation::CONSTANT_VELOCITY:
            t = t_end;
            R = q_end.toRotationMatrix();
            break;
        case MotionCompensation::CONTINUOUS:
        // 插值
        case MotionCompensation::ITERATIVE:
            R = q_begin.slerp(alpha_time, q_end).normalized().toRotationMatrix();
            t = (1.0 - alpha_time) * t_begin + alpha_time * t_end;
            break;
    }
    point_temp.point = R * (R_imu_lidar * point_temp.raw_point + t_imu_lidar) + t;
}

void gridSampling(const std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling) {
    keypoints.resize(0);
    std::vector<point3D> frame_sub;
    // copy一份frame
    frame_sub.resize(frame.size());
    for (int i = 0; i < frame_sub.size(); ++i) {
        frame_sub[i] = frame[i];
    }
    subSampleFrame(frame_sub, size_voxel_subsampling);
    keypoints.reserve(frame_sub.size());
    for (int i = 0; i < frame_sub.size(); ++i) {
        keypoints.push_back(frame_sub[i]);
    }
}
void subSampleFrame(std::vector<point3D> &frame, double size_voxel) {
    std::unordered_map<Voxel, std::vector<point3D>, std::hash<Voxel>> grid;
    for (int i = 0; i < (int)frame.size(); ++i) {
        auto grid_x = static_cast<short>(frame[i].point[0] / size_voxel);
        auto grid_y = static_cast<short>(frame[i].point[1] / size_voxel);
        auto grid_z = static_cast<short>(frame[i].point[2] / size_voxel);
        grid[Voxel(grid_x, grid_y, grid_z)].push_back(frame[i]);
    }
    frame.resize(0);
    int step = 0;
    // 认为体素中第一个数据就是keypoints
    for (const auto &n : grid) {
        if (n.second.size() > 0) {
            frame.push_back(n.second[0]);
            step++;
        }
    }
}

double AngularDistance(const Eigen::Matrix3d &rota, const Eigen::Matrix3d &rotb) {
    double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
    norm = std::acos(norm) * 180 / M_PI;
    return norm;
}
double AngularDistance(const Eigen::Vector3d &qa, const Eigen::Vector3d &qb) {
    Eigen::Quaterniond q_a = Eigen::Quaterniond(Sophus::SO3::exp(qa).matrix());
    Eigen::Quaterniond q_b = Eigen::Quaterniond(Sophus::SO3::exp(qb).matrix());
    q_a.normalize();
    q_b.normalize();
    Eigen::Matrix3d rota = q_a.toRotationMatrix();
    Eigen::Matrix3d rotb = q_b.toRotationMatrix();

    double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
    norm = std::acos(norm) * 180 / M_PI;
    return norm;
}
double AngularDistance(const Eigen::Quaterniond &qa, const Eigen::Quaterniond &qb) {
    Eigen::Matrix3d rota = qa.toRotationMatrix();
    Eigen::Matrix3d rotb = qb.toRotationMatrix();

    double norm = ((rota * rotb.transpose()).trace() - 1) / 2;
    norm = std::acos(norm) * 180 / M_PI;
    return norm;
}

}  // namespace ctlio::slam