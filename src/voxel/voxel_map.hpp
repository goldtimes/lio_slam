#pragma once
#include <math.h>
#include <tsl/robin_map.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>
#include "sensors/point_types.hh"

struct Voxel {
    Voxel() = default;
    Voxel(short x, short y, short z) : x(x), y(y), z(z) {
    }

    bool operator==(const Voxel& voxel) const {
        return x == voxel.x && y == voxel.y && z == voxel.z;
    }

    inline bool operator<(const Voxel& voxel) const {
        return x < voxel.x || (x == voxel.x && y < voxel.y) || (x == voxel.x && y == voxel.y && z == voxel.z);
    }

    inline static Voxel coordinates(const Eigen::Vector3d& point, double voxel_size) {
        return Voxel(short(point.x() / voxel_size), short(point.y() / voxel_size), short(point.z() / voxel_size));
    }
    short x;
    short y;
    short z;
};

struct VoxelBlock {
   public:
    explicit VoxelBlock(int num_points = 20) : num_points_(num_points) {
        points.reserve(num_points_);
    }
    bool IsFull() const {
        return num_points_ == points.size();
    }

    void AddPoint(const Eigen::Vector3d& point) {
        assert(num_points_ > points.size());
        points.push_back(point);
    }

    inline int NumPoints() const {
        return points.size();
    }

    inline int Capacity() {
        return num_points_;
    }

   public:
    std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> points;

   private:
    int num_points_;
};

using VoxelHashMap = tsl::robin_map<Voxel, VoxelBlock>;

namespace std {
template <>
struct hash<Voxel> {
    std::size_t operator()(const Voxel& voxel) const {
        const size_t kP1 = 73856093;
        const size_t kP2 = 19349669;
        const size_t kP3 = 83492791;
        return voxel.x * kP1 + voxel.y * kP2 + voxel.z * kP3;
    }
};
}  // namespace std