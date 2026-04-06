#!/bin/bash
# 完整启动脚本 - 同时启动AirSim和病害检测

echo "========================================="
echo "启动道路病害检测系统 (集成AirSim)"
echo "========================================="

# 设置环境变量
export PX4_SIM_HOST_ADDR=${PX4_SIM_HOST_ADDR:-"127.0.0.1"}
export PROMETHEUS_PATH=~/Prometheus
export PX4_PATH=~/prometheus_px4

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $SCRIPT_DIR

# 启动AirSim节点
echo "启动AirSim ROS节点..."
gnome-terminal --title="AirSim Node" -- bash -c "
source /opt/ros/noetic/setup.bash;
source $PROMETHEUS_PATH/devel/setup.bash;
source /home/amov/prometheus_mavros/devel/setup.bash;
source $PX4_PATH/Tools/setup_gazebo.bash $PX4_PATH $PX4_PATH/build/amovlab_sitl_default;
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_PATH;
export ROS_PACKAGE_PATH=\$ROS_PACKAGE_PATH:$PX4_PATH/Tools/sitl_gazebo;
source $SCRIPT_DIR/devel/setup.bash;
roslaunch airsim_ros_pkgs airsim_node.launch output:=screen host:=$PX4_SIM_HOST_ADDR;
exec bash" &

# 等待AirSim启动
echo "等待AirSim启动 (5秒)..."
sleep 5

# 启动病害检测节点
echo "启动病害检测节点..."
gnome-terminal --title="Defect Detection" -- bash -c "
source $SCRIPT_DIR/devel/setup.bash;
roslaunch damage_detect damage_detect.launch;
exec bash" &

echo "========================================="
echo "系统启动完成!"
echo "查看检测结果: nc -u -l 8891"
echo "查看图像话题: rostopic hz /airsim_node/uav1/front_left/Scene"
echo "========================================="

# 启动UDP监听器
echo "启动UDP监听器 (显示检测结果)..."
gnome-terminal --title="UDP Monitor" -- bash -c "
echo '等待检测结果...';
nc -u -l 8891;
exec bash" &

echo "所有节点已启动!"
