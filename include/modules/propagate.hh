/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-04 12:02:14
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-04 12:07:51
 * @FilePath: /lio_ws/src/ieskf_slam/include/modules/propagate.hh
 * @Description: 前向传播
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
#include "ieskf/ieskf.hh"
#include "type/measure_group.hh"

namespace IESKF_SLAM {
// 前向后向传播
class FrontBackPropagate {
   public:
    FrontBackPropagate() = default;
    ~FrontBackPropagate() = default;
    void propagate(MeasureGroup& group, IESKF::Ptr ieskf_ptr);

   public:
    double imu_scale;
    IMU last_imu;
};
}  // namespace IESKF_SLAM