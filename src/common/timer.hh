/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-04-17 23:02:39
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-18 00:08:08
 * @FilePath: /lio_ws/src/ieskf_slam/src/common/timer.hh
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#pragma once
#include <chrono>
#include <map>
#include <vector>

namespace ctlio::slam {
/**
 * @brief 因为slam的实时性,需要一个统计函数运行时间的工具
 */
class Timer {
   public:
    struct TimerRecord {
        TimerRecord() = default;
        TimerRecord(const std::string& func, const double used_time) {
            func_name = func;
            time_usage_in_ms_.emplace_back(used_time);
        }

        std::string func_name;
        std::vector<double> time_usage_in_ms_;
    };

    static void PrintAll();
    static void DumpIntoFile(const std::string& file_name);
    static double GetFuncMeanTime(const std::string& func_name);
    static void ClearRecords() {
        records_.clear();
    }

    /**
     * @brief 记录某个函数每次调用时间
     */
    template <typename F>
    static void Evaluate(F&& func, const std::string& func_name) {
        auto start_time = std::chrono::steady_clock::now();
        std::forward<F>(func)();
        auto end_time = std::chrono::steady_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(end_time - start_time).count() * 1000;
        if (records_.find(func_name) != records_.end()) {
            records_[func_name].time_usage_in_ms_.emplace_back(time_used);
        } else {
            records_.insert({func_name, TimerRecord(func_name, time_used)});
        }
    }

   private:
    std::string func_name_;
    // 程序运行期间应该只能有一个records
    static std::map<std::string, TimerRecord> records_;
};

class TicToc {};
};  // namespace ctlio::slam
