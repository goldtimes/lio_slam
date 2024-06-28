#include "voxel_map/voxel_map.hh"

namespace lio {
size_t HashUtil::operator()(const Eigen::Vector3d& point) {
    Eigen::Vector3d loc_xyz;
    for (size_t i = 0; i < 3; ++i) {
        loc_xyz(i) = point[i] * resolution_inv_;
        if (loc_xyz[i] < 0) {
            loc_xyz[i] -= 1.0;
        }
    }
    size_t x = static_cast<size_t>(loc_xyz[0]);
    size_t y = static_cast<size_t>(loc_xyz[1]);
    size_t z = static_cast<size_t>(loc_xyz[2]);
    return (((z * hash_p_) % max_n_ + y) * hash_p_) % max_n_ + x;
}

void Grid::setMinMax(size_t min, size_t max) {
    min_num = min;
    max_num = max;
}

void Grid::addPoint(const Eigen::Vector3d& point, bool insert) {
    points_num++;
    is_update = false;
    points_sum += point;
    centroid = points_sum / static_cast<double>(points_num);
    conv_sum += (point * point.transpose());
    if (points.size() >= 2 * max_num || !insert) return;
    points.push_back(point);
}

// 更新协方差
void Grid::updateConv() {
    // 已经更新过了
    if (is_update) {
        return;
    }
    is_update = true;
    // 体素中的点过小
    if (points_num < min_num) {
        is_valid = false;
        return;
    }
    // 更新协方差
    conv = (conv_sum - points_sum * centroid.transpose()) / (static_cast<double>(points_num) - 1);
    is_valid = true;
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(conv, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector3d val(1, 1, 1e-3);
    conv = svd.matrixU() * val.asDiagonal() * svd.matrixV().transpose();
    // conv_inv = conv.inverse();
}

void VoxelMap::initializeNearby() {
    nearby_.clear();
    switch (mode_) {
        case NEARBY1:
            /* code */
            nearby_.push_back(Eigen::Vector3d(0, 0, 0));
            break;
        case NEARBY7:
            nearby_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            nearby_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
            nearby_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
            nearby_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
            break;
        case NEARBY19:
            nearby_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            nearby_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
            nearby_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
            nearby_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
            nearby_.push_back(Eigen::Vector3d(resolution_, resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(resolution_, -resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(resolution_, 0.0, resolution_));
            nearby_.push_back(Eigen::Vector3d(resolution_, 0.0, -resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, resolution_, resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, resolution_, -resolution_));
            nearby_.push_back(Eigen::Vector3d(-resolution_, resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(-resolution_, -resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(-resolution_, 0.0, resolution_));
            nearby_.push_back(Eigen::Vector3d(-resolution_, 0.0, -resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, -resolution_, resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, -resolution_, -resolution_));
            break;
        default:
            nearby_.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            nearby_.push_back(Eigen::Vector3d(resolution_, 0.0, 0.0));
            nearby_.push_back(Eigen::Vector3d(-resolution_, 0.0, 0.0));
            nearby_.push_back(Eigen::Vector3d(0.0, resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(0.0, -resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(0.0, 0.0, resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, 0.0, -resolution_));
            nearby_.push_back(Eigen::Vector3d(resolution_, resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(resolution_, -resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(resolution_, 0.0, resolution_));
            nearby_.push_back(Eigen::Vector3d(resolution_, 0.0, -resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, resolution_, resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, resolution_, -resolution_));
            nearby_.push_back(Eigen::Vector3d(-resolution_, resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(-resolution_, -resolution_, 0.0));
            nearby_.push_back(Eigen::Vector3d(-resolution_, 0.0, resolution_));
            nearby_.push_back(Eigen::Vector3d(-resolution_, 0.0, -resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, -resolution_, resolution_));
            nearby_.push_back(Eigen::Vector3d(0.0, -resolution_, -resolution_));
            nearby_.push_back(Eigen::Vector3d(resolution_, resolution_, resolution_));
            nearby_.push_back(Eigen::Vector3d(resolution_, resolution_, -resolution_));
            nearby_.push_back(Eigen::Vector3d(resolution_, -resolution_, resolution_));
            nearby_.push_back(Eigen::Vector3d(resolution_, -resolution_, -resolution_));
            nearby_.push_back(Eigen::Vector3d(-resolution_, resolution_, resolution_));
            nearby_.push_back(Eigen::Vector3d(-resolution_, resolution_, -resolution_));
            nearby_.push_back(Eigen::Vector3d(-resolution_, -resolution_, resolution_));
            nearby_.push_back(Eigen::Vector3d(-resolution_, -resolution_, -resolution_));
            break;
    }
}

bool VoxelMap::isSameGrid(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    size_t p1_x = floor(p1.x() * resolution_inv);
    size_t p1_y = floor(p1.y() * resolution_inv);
    size_t p1_z = floor(p1.z() * resolution_inv);

    size_t p2_x = floor(p2.x() * resolution_inv);
    size_t p2_y = floor(p2.y() * resolution_inv);
    size_t p2_z = floor(p2.z() * resolution_inv);
    return p1_x == p2_x && p1_y == p2_y && p1_z == p2_z;
}

size_t VoxelMap::addCloud(const sensors::PointNormalCloud::Ptr cloud_ptr) {
    size_t error_count = 0;
    current_idx_.clear();
    // 遍历所有点
    for (sensors::PointNormalType& point : cloud_ptr->points) {
        // 计算坐标值
        Eigen::Vector3d pt_eigen(point.x, point.y, point.z);
        size_t grid_loc = hash_util_(pt_eigen);
        // 判断是否在voxel_map中
        current_idx_.push_back(grid_loc);
        auto iter = storage_.find(grid_loc);
        // 不存在
        if (iter == storage_.end()) {
            std::shared_ptr<Grid> grid = std::make_shared<Grid>(grid_capacity_);
            grid->addPoint(pt_eigen, true);
            // 放入LRU缓存中，放在头部
            cache_.push_front(grid_loc);
            storage_.insert({grid_loc, {cache_.begin(), grid}});
            if (storage_.size() >= capacity_) {
                // 删除尾部的缓存
                storage_.erase(cache_.back());
                cache_.pop_back();
            }
        } else {
            // 判断是否在同一个体素中，根据体素中的点云均值
            if (!isSameGrid(iter->second.second->centroid, pt_eigen)) {
                // 创建新的grid
                error_count++;
                cache_.erase(iter->second.first);
                std::shared_ptr<Grid> grid = std::make_shared<Grid>(grid_capacity_);
                grid->addPoint(pt_eigen, true);
                cache_.push_front(grid_loc);
                storage_[grid_loc].first = cache_.begin();
                storage_[grid_loc].second = grid;
            } else {
                // 同一个体素，更新voxelmap
                size_t num_in_grid = iter->second.second->points_num;
                if (num_in_grid < 50) {
                    bool insert = num_in_grid < grid_capacity_ ? true : false;
                    iter->second.second->addPoint(pt_eigen, insert);
                }
                // 将cache_中元素iter->second.first 移动到cache_的头部，表示体素添加新的点
                cache_.splice(cache_.begin(), cache_, iter->second.first);
                storage_[grid_loc].first = cache_.begin();
            }
        }
    }
    // 遍历当前的体素，是否更新协防差
    for (auto& idx : current_idx_) {
        storage_[idx].second->updateConv();
    }
    return error_count;
}

void VoxelMap::reset() {
    cache_.clear();
    storage_.clear();
}

bool VoxelMap::searchKnn(const Eigen::Vector3d& point, size_t K, double range_threshold,
                         std::vector<Eigen::Vector3d>& results) {
    // 存放点和距离
    std::vector<std::pair<Eigen::Vector3d, double>> candidates;
    double range2 = range_threshold * range_threshold;
    for (const Eigen::Vector3d& near : nearby_) {
        Eigen::Vector3d near_by = point + near;
        size_t hash_idx = hash_util_(near_by);
        auto iter = storage_.find(hash_idx);
        if (iter != storage_.end() && isSameGrid(near_by, iter->second.second->centroid)) {
            // 点到周围的体素的距离小于range
            for (const Eigen::Vector3d& pt : iter->second.second->points) {
                double dist = (point - pt).squaredNorm();
                if (dist < range_threshold) {
                    // candidates.push_back({near_by, hash_idx});
                    candidates.emplace_back(pt, dist);
                }
            }
        }
    }
    if (candidates.empty()) {
        return false;
    }
    // 利用std库找到第k小的数
    if (candidates.size() > K) {
        // 经过nth_element, K之前的元素都小于第K个元素，K之后的元素都大于第k个元素
        std::nth_element(candidates.begin(), candidates.begin() + K - 1, candidates.end(),
                         [](const auto& p1, const auto& p2) { return p1.second < p2.second; });
        candidates.resize(K);
    }
    // 在排序一次
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& p1, const auto& p2) { return p1.second < p2.second; });
    results.clear();
    for (const auto& it : candidates) {
        results.emplace_back(it.first);
    }
    return true;
}

bool VoxelMap::getCentroidAndConvariance(size_t hash_idx, Eigen::Vector3d& centroid, Eigen::Matrix3d& conv) {
    auto iter = storage_.find(hash_idx);
    if (iter != storage_.end() && iter->second.second->is_valid) {
        centroid = iter->second.second->centroid;
        conv = iter->second.second->conv;
        return true;
    }
    return false;
}

bool VoxelMap::getCentroidAndConvariance(const Eigen::Vector3d& point, Eigen::Vector3d& centroid,
                                         Eigen::Matrix3d& conv) {
    size_t hash_idx = hash_util_(point);
    return getCentroidAndConvariance(hash_idx, centroid, conv);
}

void FastGrid::addPoint(const Eigen::Vector3d& point) {
    num += 1;
    centroid += ((point - centroid) / static_cast<double>(num));
}

FastVoxelMap::FastVoxelMap(double resolution) : resolution_(resolution), hash_util_(resolution) {
    // 初始化周围的搜索范围
    search_range_.clear();
    for (int x_gain = -1; x_gain <= 1; ++x_gain) {
        for (int y_gain = -1; y_gain <= 1; ++y_gain) {
            for (int z_gain = -1; z_gain <= 1; ++z_gain) {
                search_range_.emplace_back(
                    // 周围q个点
                    Eigen::Vector3d(x_gain * resolution, y_gain * resolution, z_gain * resolution));
            }
        }
    }
}
// 对当前的点云进行滤波？
void FastVoxelMap::filter(sensors::PointNormalCloud::Ptr cloud_ptr, std::vector<PointWithCov>& outs) {
    if (cloud_ptr->empty()) {
        return;
    }
    outs.clear();
    grid_array_.clear();
    grid_map_.clear();
    grid_array_.reserve(cloud_ptr->size());
    for (const sensors::PointNormalType& point : cloud_ptr->points) {
        Eigen::Vector3d pt_eigen(point.x, point.y, point.z);
        size_t idx = hash_util_(pt_eigen);
        auto iter = grid_map_.find(idx);
        if (iter == grid_map_.end()) {
            std::shared_ptr<FastGrid> fast_grid = std::make_shared<FastGrid>();
            fast_grid->addPoint(pt_eigen);
            grid_map_.insert({idx, fast_grid});
            grid_array_.push_back(fast_grid);
        } else {
            iter->second->addPoint(pt_eigen);
        }
    }
    // 遍历所有的grid
    for (const auto& grid : grid_array_) {
        Eigen::Vector3d points_sum = Eigen::Vector3d::Zero();
        Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
        size_t points_num = 0;
        // 遍历周围的几个体素
        for (const Eigen::Vector3d& p : search_range_) {
            Eigen::Vector3d near = grid->centroid + p;
            auto hash_idx = hash_util_(near);
            auto iter = grid_map_.find(hash_idx);
            if (iter != grid_map_.end()) {
                // 质心相加
                points_sum += iter->second->centroid;
                cov += iter->second->centroid * iter->second->centroid.transpose();
                points_num++;
            }
        }
        if (points_num >= 6) {
            Eigen::Vector3d centroid = points_sum / static_cast<double>(points_num);
            cov = (cov - points_sum * centroid.transpose()) / (static_cast<double>(points_num) - 1.0);
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Vector3d values(1, 1, 1e-3);
            cov = svd.matrixU() * values.asDiagonal() * svd.matrixV().transpose();
        }
        outs.emplace_back(grid->centroid, cov);
    }
}
}  // namespace lio