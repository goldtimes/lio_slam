#include "loopclosure/loopclosure.hh"

namespace lio {
void LoopClosureThread::init() {
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    kdtree_histories_pose.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    cloud_histories_pose.reset(new pcl::PointCloud<pcl::PointXYZ>());
    submap_filter.reset(new pcl::VoxelGrid<sensors::PointNormalType>());
    submap_filter->setLeafSize(loop_params_.submap_resolution, loop_params_.submap_resolution,
                               loop_params_.submap_resolution);
    icp.reset(new pcl::IterativeClosestPoint<sensors::PointNormalType, sensors::PointNormalType>);
    icp->setMaxCorrespondenceDistance(100);
    icp->setMaximumIterations(50);
    icp->setTransformationEpsilon(1e-6);
    icp->setEuclideanFitnessEpsilon(1e-6);
    icp->setRANSACIterations(0);
}

void LoopClosureThread::setLoopRate(std::shared_ptr<ros::Rate> loop_rate) {
    loop_rate_ = loop_rate;
}

void LoopClosureThread::setSharedData(std::shared_ptr<LoopSharedData> share_data_ptr) {
    shared_data_ = share_data_ptr;
}
sensors::PointNormalCloud::Ptr LoopClosureThread::getSubmaps(std::vector<Pose6D>& poses_list,
                                                             std::vector<sensors::PointNormalCloud::Ptr>& cloud_list,
                                                             int index, int search_num) {
    sensors::PointNormalCloud::Ptr cloud(new sensors::PointNormalCloud());
    int max_pose_size = poses_list.size();
    // 防止负数
    int min_index = std::max(0, index - search_num);
    // 防止超过max_pose_size的值
    int max_index = std::min(max_pose_size - 1, index + search_num);
    for (int i = min_index; i <= max_index; ++i) {
        Pose6D& p = poses_list[i];
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = p.global_rot;
        T.block<3, 1>(0, 3) = p.global_pos;
        sensors::PointNormalCloud::Ptr temp_cloud(new sensors::PointNormalCloud);
        pcl::transformPointCloud(*cloud_list[p.index], *temp_cloud, T);
        *cloud += *temp_cloud;
    }
    return cloud;
}

void LoopClosureThread::operator()() {
    while (ros::ok()) {
        loop_rate_->sleep();
        if (!loop_params_.active) {
            continue;
        }
        // 关键帧的姿态较少不进行回环
        if (shared_data_->keyposes.size() < loop_params_.loop_pose_min_size_thresh) {
            continue;
        }
        // 没有更新关键帧姿态，不进行回环
        if (!shared_data_->keypose_add) {
            continue;
        }
        // 清空标志为
        shared_data_->keypose_add = false;
        {
            std::lock_guard<std::mutex> lck(shared_data_->mutex);
            // 关键帧的id
            lastest_index = shared_data_->keyposes.size() - 1;
            tmp_keypose.clear();
            // copy keypose
            tmp_keypose.assign(shared_data_->keyposes.begin(), shared_data_->keyposes.end());
        }

        loopCheck();
        addOdomFactor();
        addLoopFactor();
        smoothAndUpdate();
    }
}

/**
 * @brief 检查是否有回环？
 */
void LoopClosureThread::loopCheck() {
    if (tmp_keypose.empty()) {
        return;
    }
    int curr_index = tmp_keypose.size() - 1;
    int pre_index = -1;
    cloud_histories_pose->clear();
    // ROS_INFO("tmp keypose size: %ld", tmp_keypose.size());
    // shared_data_->keypose保存了所有的轨迹数据
    for (const auto& pose : tmp_keypose) {
        pcl::PointXYZ point;
        point.x = pose.global_pos(0);
        point.y = pose.global_pos(1);
        point.z = pose.global_pos(2);
        cloud_histories_pose->push_back(point);
    }
    kdtree_histories_pose->setInputCloud(cloud_histories_pose);
    std::vector<int> idxs;
    std::vector<float> dist;
    // 找最后一个位姿半径10m以内的点
    kdtree_histories_pose->radiusSearch(cloud_histories_pose->back(), loop_params_.loop_pose_search_radius, idxs, dist,
                                        0);
    //找到可能的id
    for (int i = 0; i < idxs.size(); ++i) {
        int id = idxs[i];
        if (std::abs(tmp_keypose[id].time - tmp_keypose.back().time) > loop_params_.time_thresh &&
            std::abs(curr_index - id) >= loop_params_.loop_pose_min_size_thresh) {
            pre_index = id;
            break;
        }
    }
    if (pre_index == -1 || pre_index == curr_index) {
        return;
    }
    // 有可能的回环，然后我们调用icp
    sensors::PointNormalCloud::Ptr cur_cloud = getSubmaps(tmp_keypose, shared_data_->cloud_history, curr_index, 0);
    sensors::PointNormalCloud::Ptr submaps =
        getSubmaps(tmp_keypose, shared_data_->cloud_history, pre_index, loop_params_.submap_search_num);
    icp->setInputSource(cur_cloud);
    icp->setInputTarget(submaps);

    sensors::PointNormalCloud::Ptr aligned(new sensors::PointNormalCloud());
    icp->align(*aligned, Eigen::Matrix4f::Identity());
    // save aligned cloud
    float score = icp->getFitnessScore();
    if (!icp->hasConverged() || score > loop_params_.loop_icp_thresh) {
        return;
    }
    ROS_INFO("Detected Loop: [pre_idx:%d, cur_idx:%d, score:%f]\n", pre_index, curr_index, score);
    shared_data_->loop_history.emplace_back(pre_index, curr_index);
    loop_found = true;
    Eigen::Matrix4d T_pre_cur = icp->getFinalTransformation().cast<double>();
    Eigen::Matrix4d T_G_cur = Eigen::Matrix4d::Identity();
    T_G_cur.block<3, 3>(0, 0) = tmp_keypose[curr_index].global_rot;
    T_G_cur.block<3, 1>(0, 3) = tmp_keypose[curr_index].global_pos;
    Eigen::Matrix4d T_G_pre = Eigen::Matrix4d::Identity();
    T_G_pre.block<3, 3>(0, 0) = tmp_keypose[pre_index].global_rot;
    T_G_pre.block<3, 1>(0, 3) = tmp_keypose[pre_index].global_pos;
    // 变换矩阵的逆 而不是transpose()
    Eigen::Matrix4d T_12 = T_G_pre.inverse() * T_pre_cur * T_G_cur;
    Eigen::Matrix3d R12 = T_12.block<3, 3>(0, 0);
    Eigen::Vector3d t12 = T_12.block<3, 1>(0, 3);
    // std::cout << "R12: \n" << R12 << std::endl;
    // std::cout << "t12: " << t12.transpose() << std::endl;
    // Eigen::Matrix3d R12_tmp = tmp_keypose[pre_index].global_rot.transpose() * T_pre_cur.block<3, 3>(0, 0) *
    //                           tmp_keypose[curr_index].global_rot;
    // Eigen::Vector3d t12_tmp = tmp_keypose[pre_index].global_rot.transpose() *
    //                           (T_pre_cur.block<3, 3>(0, 0) * tmp_keypose[curr_index].global_pos +
    //                            T_pre_cur.block<3, 1>(0, 3) - tmp_keypose[pre_index].global_pos);
    // std::cout << "r12_tmp: \n" << R12_tmp << std::endl;
    // std::cout << "t12_tmp: " << t12_tmp.transpose() << std::endl;
    shared_data_->loop_pairs.emplace_back(pre_index, curr_index, score, R12, t12);
}
// 构建odom之间的factor
void LoopClosureThread::addOdomFactor() {
    for (int i = previous_index; i < lastest_index; ++i) {
        Pose6D& cur_pos = tmp_keypose[i];
        Pose6D& next_pos = tmp_keypose[i + 1];
        if (i == 0) {
            // 添加顶点信息
            initialized_estimate.insert(i,
                                        gtsam::Pose3(gtsam::Rot3(cur_pos.local_rot), gtsam::Point3(cur_pos.local_pos)));
            // 添加顶点噪声
            gtsam::noiseModel::Diagonal::shared_ptr noise =
                gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * 1e-12);
            // 添加到图优化模型中
            gtsam_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
                i, gtsam::Pose3(gtsam::Rot3(cur_pos.local_rot), gtsam::Point3(cur_pos.local_pos)), noise));
        }
        initialized_estimate.insert(i + 1,
                                    gtsam::Pose3(gtsam::Rot3(next_pos.local_rot), gtsam::Point3(next_pos.local_pos)));
        Eigen::Matrix3d R12 = cur_pos.local_rot.transpose() * next_pos.local_rot;
        Eigen::Vector3d t12 = cur_pos.local_rot.transpose() * (next_pos.local_pos - cur_pos.local_pos);
        gtsam::Vector6 noise_vector;
        noise_vector << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-6;
        gtsam::noiseModel::Diagonal::shared_ptr noise = gtsam::noiseModel::Diagonal::Variances(noise_vector);
        gtsam_graph.add(
            gtsam::BetweenFactor<gtsam::Pose3>(i, i + 1, gtsam::Pose3(gtsam::Rot3(R12), gtsam::Point3(t12)), noise));
    }
    previous_index = lastest_index;
}
void LoopClosureThread::addLoopFactor() {
    if (!loop_found) {
        return;
    }
    if (shared_data_->loop_pairs.empty()) {
        return;
    }
    for (LoopPair& lp : shared_data_->loop_pairs) {
        gtsam::Pose3 pose_between(gtsam::Rot3(lp.diff_rot), gtsam::Point3(lp.diff_vec));
        auto factor = gtsam::BetweenFactor<gtsam::Pose3>(
            lp.pre_idx, lp.cur_idx, pose_between,
            gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Ones() * lp.score));
        gtsam_graph.add(factor);
    }
    shared_data_->loop_pairs.clear();
}
void LoopClosureThread::smoothAndUpdate() {
    isam2->update(gtsam_graph, initialized_estimate);
    isam2->update();
    if (loop_found) {
        isam2->update();
        isam2->update();
        isam2->update();
        isam2->update();
        isam2->update();
        loop_found = false;
    }
    gtsam_graph.resize(0);
    initialized_estimate.clear();
    optimized_estimate = isam2->calculateBestEstimate();
    gtsam::Pose3 lastest_estimate = optimized_estimate.at<gtsam::Pose3>(lastest_index);
    tmp_keypose[lastest_index].global_rot = lastest_estimate.rotation().matrix().cast<double>();
    tmp_keypose[lastest_index].global_pos = lastest_estimate.translation().matrix().cast<double>();
    Eigen::Matrix3d offset_rot;
    Eigen::Vector3d offset_pos;
    tmp_keypose[lastest_index].getOffset(offset_rot, offset_pos);
    shared_data_->mutex.lock();
    int current_size = shared_data_->keyposes.size();
    shared_data_->offset_rot = offset_rot;
    shared_data_->offset_trans = offset_pos;
    shared_data_->mutex.unlock();
    for (int i = 0; i < lastest_index; ++i) {
        gtsam::Pose3 optimized_pose = optimized_estimate.at<gtsam::Pose3>(i);
        shared_data_->keyposes[i].global_rot = optimized_pose.rotation().matrix().cast<double>();
        shared_data_->keyposes[i].global_pos = optimized_pose.translation().matrix().cast<double>();
    }
    // ROS_INFO("lastest_index:%ld, current_size:%ld", lastest_index, current_size);
    for (int i = lastest_index; i < current_size; ++i) {
        shared_data_->keyposes[i].addOffset(offset_rot, offset_pos);
    }
}

}  // namespace lio