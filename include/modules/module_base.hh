/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-01 00:30:12
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-13 13:30:45
 * @FilePath: /lio_ws/src/ieskf_slam/include/modules/module_base.hh
 * @Description: 几乎所有的模块都需要通过配置文件来修改参数，所以定义一个基类
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>

namespace IESKF_SLAM {
class ModuleBase {
   private:
    YAML::Node config_node_;
    std::string module_name_;

   protected:
    ModuleBase(const std::string& config_path, const std::string& prefix, const std::string& module_name = "default") {
        module_name_ = module_name;
        if (config_path != "") {
            try {
                // LoadFile 返回一个node
                std::cout << "config_path: " << config_path << std::endl;
                config_node_ = YAML::LoadFile(config_path);
            } catch (YAML::Exception& e) {
                std::cerr << "module_name:" << module_name << ",get error msg:" << e.what() << std::endl;
            }
            if (prefix != "" && config_node_[prefix]) {
                config_node_ = config_node_[prefix];
            }
        }
    }
    /**
     * @brief 读取参数
     */
    template <typename T>
    void readParam(const std::string& key, T& value, T default_value) {
        if (config_node_[key]) {
            value = config_node_[key].as<T>();
        } else {
            value = default_value;
        }
    }
};
}  // namespace IESKF_SLAM
