#include "loopclosure/loopclosure.hh"

namespace lio {
void LoopClosureThread::init() {
    gtsam::ISAM2Params isam2_params;
    isam2_params.relinearizeThreshold = 0.01;
    isam2_params.relinearizeSkip = 1;
    isam2 = std::make_shared<gtsam::ISAM2>(isam2_params);
    kdtree_histories.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
    cloud_histories.reset(new pcl::PointCloud<pcl::PointXYZ>());
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
}
void LoopClosureThread::addOdomFactor() {
}
void LoopClosureThread::addLoopFactor() {
}
void LoopClosureThread::smoothAndUpdate() {
}

}  // namespace lio