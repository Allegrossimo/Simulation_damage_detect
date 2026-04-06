#!/bin/bash
# AirSim环境配置脚本

echo "========================================="
echo "配置AirSim ROS环境"
echo "========================================="

# 设置基础路径
export PROMETHEUS_PATH=~/Prometheus
export PX4_PATH=~/prometheus_px4
export AIRSIM_ROS_PATH=/home/amov/Desktop/basic/damage_detect_ws

# 设置ROS环境
source /opt/ros/noetic/setup.bash
source $PROMETHEUS_PATH/devel/setup.bash
source /home/amov/prometheus_mavros/devel/setup.bash

# 设置PX4环境
if [ -f "$PX4_PATH/Tools/setup_gazebo.bash" ]; then
    source $PX4_PATH/Tools/setup_gazebo.bash $PX4_PATH $PX4_PATH/build/amovlab_sitl_default
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$PX4_PATH/Tools/sitl_gazebo
fi

# 设置当前工作空间
source $AIRSIM_ROS_PATH/devel/setup.bash

# 设置AirSim IP地址（根据实际情况修改）
export PX4_SIM_HOST_ADDR=${PX4_SIM_HOST_ADDR:-"127.0.0.1"}

echo "环境配置完成!"
echo "ROS版本: $ROS_DISTRO"
echo "工作空间: $AIRSIM_ROS_PATH"
echo "PX4路径: $PX4_PATH"
echo "AirSim主机: $PX4_SIM_HOST_ADDR"

# 显示环境变量
echo ""
echo "ROS_PACKAGE_PATH:"
echo $ROS_PACKAGE_PATH | tr ':' '\n' | head -5
