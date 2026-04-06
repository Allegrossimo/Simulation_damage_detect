#!/bin/bash
echo "========================================="
echo "安装AirSim ROS依赖"
echo "========================================="

# 更新包列表
sudo apt-get update

# 安装必要的依赖
sudo apt-get install -y \
    ros-noetic-image-transport \
    ros-noetic-cv-bridge \
    ros-noetic-sensor-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-tf \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs

# 安装Python依赖
pip3 install --upgrade \
    ultralytics \
    opencv-python \
    numpy \
    scipy \
    pyyaml

echo "依赖安装完成!"
