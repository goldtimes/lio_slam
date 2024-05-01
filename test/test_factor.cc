#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <map>
#include <unordered_map>
#include <vector>
#include "lidar_factor/analytical_factor.hh"
#include "lidar_factor/lidar_factor.hh"
#include "lidar_factor/pose_parameterization.hh"
#include "sophus/so3.h"
#include "tools/tool_color_printf.hpp"

using namespace std;

namespace Eigen {
template <typename T>
using aligen_vector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename T>
using aligen_queue = std::deque<T, Eigen::aligned_allocator<T>>;

template <typename K, typename V>
using aligen_map = std::map<K, V, std::less<K>, Eigen::aligned_allocator<std::pair<K const, V>>>;

template <typename K, typename V>
using aligen_unordered_map = std::unordered_map<K, V, std::less<K>, Eigen::aligned_allocator<std::pair<K const, V>>>;
}  // namespace Eigen
class FactorTest {
   public:
    FactorTest() {
        parameterization = new RotationParameterization();
        autodiff_parameterization = new ceres::EigenQuaternionParameterization();
        solver_options.max_num_iterations = 1;
        solver_options.num_threads = 1;
        solver_options.minimizer_progress_to_stdout = false;
        solver_options.trust_region_strategy_type = ceres::TrustRegionStrategyType::LEVENBERG_MARQUARDT;
        double lidar_cov = 1;
        ctlio::slam::LidarPlaneNormFactor::t_il = Eigen::Vector3d::Zero();
        ctlio::slam::LidarPlaneNormFactor::q_il = Eigen::Quaterniond::Identity();
        ctlio::slam::LidarPlaneNormFactor::sqrt_info = std::sqrt(1 / lidar_cov);
    }
    ~FactorTest() {
    }

    void TestPointToPlaneFactorAutoDiff(Eigen::aligen_map<double *, Eigen::MatrixXd> &jacobians) {
        Eigen::Vector3d norm_vector(0.3, 1.5, -2);
        norm_vector.normalize();
        Eigen::Vector3d vector_neig(1, 3, 5);
        double norm_offset = -norm_vector.dot(vector_neig);
        Eigen::Vector3d point_end(10, 12, 14);
        double weight = 1;

        Eigen::Quaterniond end_quat = Eigen::Quaterniond(0.2, 0.6, 1.3, -0.9);
        end_quat.normalize();
        std::cout << "normal: " << norm_vector.transpose() << std::endl;
        Eigen::Vector3d end_t(11, 13, 15);

        auto *cost_function = ctlio::slam::FunctorPointToPlane::Create(vector_neig, point_end, norm_vector, weight);
        ceres::Problem problem(problem_options);
        std::vector<double *> vec;
        vec.emplace_back(end_t.data());
        vec.emplace_back(end_t.data());
        problem.AddParameterBlock(&end_quat.x(), 4, autodiff_parameterization);
        problem.AddParameterBlock(&end_t.x(), 3);
        problem.AddResidualBlock(cost_function, nullptr, &end_t.x(), &end_quat.x());
        // GetJacobian(vec, problem, cost_function->num_residuals(), jacobians);
    }

   private:
    ceres::LocalParameterization *parameterization;
    ceres::LocalParameterization *autodiff_parameterization;
    ceres::Problem::Options problem_options;
    ceres::Solver::Options solver_options;
};