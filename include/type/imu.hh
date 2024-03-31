/*
 * @Author: lihang 1019825699@qq.com
 * @Date: 2024-03-31 22:52:31
 * @LastEditors: lihang 1019825699@qq.com
 * @LastEditTime: 2024-04-01 00:10:40
 * @FilePath: /lio_ws/src/ieskf_slam/include/type/imu.hh
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */

#pragma once

#include <eigen3/Eigen/Dense>
#include "timestamped.hh"

namespace IESKF_SLAM{
    class IMU {
        public:
            Eigen::Vector3d acc = Eigen::Vector3d::Zero();
            Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
            TimeStamp time_stamp;

            void clear(){
                acc = Eigen::Vector3d::Zero();
                gyro = Eigen::Vector3d::Zero();
                time_stamp = 0;
            }

            IMU operator + (const IMU& other) {
                IMU res;
                res.acc = acc + other.acc;
                res.gyro = gyro + other.gyro;
                return res;
            }

            IMU operator * (double k) {
                IMU res;
                res.acc = acc * k;
                res.gyro = gyro * k;
                return res;
            }

            IMU operator / (double k) {
                IMU res;
                res.acc = acc / k;
                res.gyro = gyro / k;
                return res;
            }
            /**
             * @brief 重载输出符号
            */
            friend std::ostream& operator<<(std::ostream& ostream, const IMU& imu){
                ostream<<"imu_time: "<<imu.time_stamp.sec()<<" s | imu_acc: "<<imu.acc.transpose()<<" | imu_gro: "<< imu.gyro.transpose() ;
                return ostream;
            }
    };
}