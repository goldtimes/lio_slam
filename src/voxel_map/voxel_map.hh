#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>
#include "sensors/point_types.hh"

namespace lio {
struct PointWithCov {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector3d point;
    Eigen::Matrix3d point_cov;
    PointWithCov(const Eigen::Vector3d& p, const Eigen::Matrix3d& cov) : point(p), point_cov(cov) {
    }
};
// 计算点的hash值
class HashUtil {
   public:
    HashUtil() = default;
    HashUtil(double resolution) : resolution_inv_(1.0 / resolution) {
    }
    HashUtil(double resolution, size_t hash_p, size_t max_n)
        : resolution_inv_(1.0 / resolution), hash_p_(hash_p), max_n_(max_n) {
    }
    size_t operator()(const Eigen::Vector3d& point);

   private:
    size_t hash_p_ = 116101;
    size_t max_n_ = 1e10;
    double resolution_inv_ = 1.0;
};

struct Grid {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Grid(size_t num) : max_num(num) {
        points.reserve(2 * max_num);
    }
    size_t hash = 0;
    size_t max_num = 20;
    size_t min_num = 6;
    size_t point_num = 0;
    bool is_update = false;
    bool is_valid = false;
    std::vector<Eigen::Vector3d> points;
    // 质心
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    Eigen::Matrix3d conv = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d conv_inv = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d conv_sum = Eigen::Matrix3d::Zero();
    Eigen::Vector3d points_sum = Eigen::Matrix3d::Zero();
    void setMinMax(size_t min_n, size_t max_n);
    void updateConv();
    void addPoint(const Eigen::Vector3d& point, bool insert);
};

class VoxelMap {
   public:
    enum NEARBY {
        NEARBY1,
        NEARBY7,
        NEARBY19,
        NEARBY26,
    };
    VoxelMap(double resolution, size_t capacity, size_t grid_capacity, NEARBY mode)
        : resolution_(resolution),
          resolution_inv(1.0 / resolution),
          capacity_(capacity),
          grid_capacity_(grid_capacity),
          mode_(mode),
          hash_util_(resolution) {
        initializeNearby();
    }

    bool isSameGrid(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    size_t addCloud(const sensors::PointNormalCloud::Ptr cloud_ptr);
    void reset();
    bool searchKnn(const Eigen::Vector3d& point, size_t K, double range_threshold,
                   std::vector<Eigen::Vector3d>& results);
    bool getCentroidAndConvariance(size_t hash_idx, Eigen::Vector3d& centroid, Eigen::Matrix3d& conv);
    bool getCentroidAndConvariance(const Eigen::Vector3d& point, Eigen::Vector3d& centroid, Eigen::Matrix3d& conv);

    std::vector<Eigen::Vector3d>& searchRange() {
        return nearby_;
    }

   private:
    NEARBY mode_ = NEARBY::NEARBY7;
    double resolution_ = 1.0;
    double resolution_inv = 1.0;
    size_t capacity_ = 5 * 1e6;
    size_t grid_capacity_ = 20;
    HashUtil hash_util_;
    // 存储点的体素坐标，方便后续删除的操作
    std::list<size_t> cache_;
    // 存储周围的点
    std::vector<Eigen::Vector3d> nearby_;
    std::vector<size_t> current_idx_;
    // 体素map
    std::unordered_map<size_t, std::pair<std::list<size_t>::iterator, std::shared_ptr<Grid>>> storage_;

   private:
    void initializeNearby();
};

struct FastGrid {
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    size_t num = 0;
    void addPoint(const Eigen::Vector3d& point);
};

class FastVoxelMap {
   public:
    FastVoxelMap(double resolution);
    void filter(sensors::PointNormalCloud::Ptr cloud_ptr, std::vector<PointWithCov>& outs);

   private:
    HashUtil hash_util_;
    double resolution_;
    std::vector<Eigen::Vector3d> search_range_;
    std::vector<std::shared_ptr<FastGrid>> grid_array_;
    std::unordered_map<size_t, std::shared_ptr<FastGrid>> grid_map_;
};
}  // namespace lio