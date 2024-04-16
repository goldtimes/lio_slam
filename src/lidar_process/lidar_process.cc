#include "lidar_process.hh"
#include <glog/logging.h>

namespace ctlio::slam {

void LidarProcess::LoadFromYaml(const std::string& yaml_file) {
    YAML::Node config_node = YAML::LoadFile(yaml_file);
    int lidar_type = config_node["lidar_process"]["lidar_type"].as<int>();
    point_filter_num = config_node["lidar_process"]["point_filter_num"].as<int>();
    blind = config_node["lidar_process"]["blind"].as<double>();
    if (lidar_type == 1) {
        lidar_type_ = LidarType::AVIA;
        LOG(INFO) << "Using AVIA Lidar";
    } else if (lidar_type == 2) {
        lidar_type_ = LidarType::VELO32;
        LOG(INFO) << "Using AVIA VELO32";
    } else if (lidar_type == 3) {
        lidar_type_ == LidarType::OUST64;
        LOG(INFO) << "Using AVIA OUST64";
    } else if (lidar_type == 4) {
        lidar_type_ == LidarType::ROBOSENSE16;
        LOG(INFO) << "Using AVIA ROBOSENSE16";
    } else if (lidar_type == 5) {
        lidar_type_ == LidarType::PANDAR;
        LOG(INFO) << "Using AVIA PANDAR";
    } else if (lidar_type == 6) {
        lidar_type_ == LidarType::LEISHEN;
        LOG(INFO) << "Using AVIA LEISHEN";
    } else {
        LOG(ERROR) << "unknown lidar_type";
    }
}
}  // namespace ctlio::slam