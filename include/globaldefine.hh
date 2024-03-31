/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-03-31 22:37:17
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-03-31 22:44:21
 * @FilePath: /lio_ws/src/ieskf_slam/include/globaldefine.hh
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#pragma once

// 通过cmake 定义该变量
#include <string>
#ifndef PROJECT_DIR
#define PROJECT_DIR " "

#endif

/// @brief 定义工作目录
const std::string WORK_DIR = PROJECT_DIR;
/// @brief 配置文件的目录
const std::string CONFIG_DIR = WORK_DIR + "/config/";