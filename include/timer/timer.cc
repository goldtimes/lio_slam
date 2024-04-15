#include "timer.hh"
#include <glog/logging.h>
#include <fstream>
#include <iostream>
#include <numeric>

namespace ctlio::slam {
// 静态变量初始化
std::map<std::string, Timer::TimerRecord> Timer::records_;

void Timer::PrintAll() {
    std::cout << ">>> ===== Printing run time =====" << std::endl;
    for (const auto& r : records_) {
        double sum = 0.0;
        std::cout << "> [ " << r.first << " ] average time usage: "
                  << std::accumulate(r.second.time_usage_in_ms_.begin(), r.second.time_usage_in_ms_.end(), sum) /
                         r.second.time_usage_in_ms_.size()
                  << " ms , called times: " << r.second.time_usage_in_ms_.size() << std::endl;
    }
    std::cout << ">>> ===== Printing run time end =====" << std::endl;
}

void Timer::DumpIntoFile(const std::string& file_name) {
    std::ofstream ofile(file_name, std::ios::out);
    if (!ofile.is_open()) {
        std::cout << "Failed to open file: " << file_name << std::endl;
        return;
    }

    std::cout << "Dump Time Records into file: " << file_name << std::endl;

    // 先统计记录中的函数被调用的最大次数
    int max_length = 0;
    for (const auto& r : records_) {
        // 打印各函数名
        ofile << r.first << ",";
        if (r.second.time_usage_in_ms_.size() > max_length) {
            max_length = r.second.time_usage_in_ms_.size();
        }
    }
    ofile << std::endl;
    for (size_t i = 0; i < max_length; ++i) {
        for (const auto& r : records_) {
            if (i < r.second.time_usage_in_ms_.size()) {
                ofile << r.second.time_usage_in_ms_[i] << ",";
            } else {
                ofile << ",";
            }
        }
        ofile << std::endl;
    }
    ofile.close();
}

double Timer::GetFuncMeanTime(const std::string& func_name) {
    if (records_.find(func_name) == records_.end()) {
        return 0.0;
    }
    double sum = 0.0;
    TimerRecord record = records_[func_name];
    return std::accumulate(record.time_usage_in_ms_.begin(), record.time_usage_in_ms_.end(), sum) /
           static_cast<double>(record.time_usage_in_ms_.size());
}

}  // namespace ctlio::slam