#include "lio_utils.hh"

namespace ctlio::slam {

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

void gridSampling(std::vector<point3D> &frame, std::vector<point3D> &keypoints, double size_voxel_subsampling) {
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