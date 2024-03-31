/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-03-31 22:57:53
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-03-31 23:13:38
 * @FilePath: /lio_ws/src/ieskf_slam/include/type/timestamped.hh
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */

#pragma once
#include <iostream>

namespace IESKF_SLAM{
    class TimeStamp{
        public:
            TimeStamp(uint64_t nsec = 0){
                nsec_ = nsec;
                sec_ = static_cast<double>(nsec_) / 1e9;
            }

            void fromSec(double isec){
                sec_ = isec;
                nsec_ = static_cast<uint64_t>(isec * 1e9);
            }

            void fromNsec(uint64_t insec = 0){
                nsec_ = insec;
                sec_ = static_cast<double>(nsec_) / 1e9; 
            }

            const uint64_t& nsec() const {return nsec_; }
            
            const double& sec() const {return sec_; }

        private:
            uint64_t nsec_;
            double sec_;
    };
}