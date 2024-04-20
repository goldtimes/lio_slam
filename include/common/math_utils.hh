#pragma once

#include <algorithm>
#include <numeric>
#include "eigen_types.hh"

namespace ctlio::slam::math {

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
}  // namespace ctlio::slam::math