/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-03 23:52:03
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-03 23:53:38
 * @FilePath: /lio_ws/src/ieskf_slam/src/ieskf/ieskf.cc
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "ieskf/ieskf.hh"

namespace IESKF_SLAM {
IESKF::IESKF(const std::string& config_path, const std::string& prefix) : ModuleBase(config_path, prefix, "IESKF") {
}

bool IESKF::Predict(const IMU& imu, double dt) {
}

bool IESKF::Update() {
}

}  // namespace IESKF_SLAM