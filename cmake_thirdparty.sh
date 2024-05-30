#!/bin/bash

# 这里我们生成了一些g2o的动态库文件，后面在cmake中可以引用
cd ./thirdparty/g2o
mkdir build && cd build
cmake ..
make -j8

# 编译ceres-solver
cd ./thirdparty/ceres-solver
mkdir build && cd build
cmake ..
sudo make install -j8