#pragma once

#include <algorithm>
#include <numeric>
#include "eigen_types.hh"

namespace ctlio::slam::math {

constexpr double kDEG2RAD = M_PI / 180.0;  // deg->rad
constexpr double kRAD2DEG = 180.0 / M_PI;
constexpr size_t kINVALID_ID = std::numeric_limits<size_t>::max();

/**
 * @brief 计算一个容器中的均值和对角形式的协方差
 * @param C 容器类型
 * @param D 结果类型
 * @param Getter 获取数据函数
 */
template <typename C, typename D, typename Getter>
void ComputeMeanAndCovDiag(const C& datas, D& mean, D& cov_diag, Getter&& getter) {
    size_t len = datas.size();
    //clang-format off
    mean = std::accumulate(datas.begin(), datas.end(), D::Zero().eval(),
                           [&getter](const D& sum, const auto& data) -> D { return sum + getter(data); }) /
           len;

    cov_diag = std::accumulate(datas.begin(), datas.end(), D::Zero().eval(),
                               [&mean, &getter](const D& sum, const auto& data) -> D {
                                   return sum + (getter(data) - mean).cwiseAbs2().eval();
                               }) /
               (len - 1);

    // clang-format on
}

template <typename S>
inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<S>& value) {
    return Eigen::Matrix<S, 3, 1>(value[0], value[1], value[2]);
}

template <typename S>
inline Eigen::Matrix<S, 3, 3> Mat3FromArray(const std::vector<S>& value) {
    Eigen::Matrix<S, 3, 3> mat3d;
    mat3d << value[0], value[1], value[2], value[3], value[4], value[5], value[6], value[7], value[8];
    return mat3d;
}
}  // namespace ctlio::slam::math