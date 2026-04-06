#!/bin/bash
echo "========================================="
echo "检查环境变量和ROS话题"
echo "========================================="

# 检查ROS环境
echo "1. ROS环境:"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   ROS_MASTER_URI: $ROS_MASTER_URI"

# 检查工作空间
echo ""
echo "2. 工作空间:"
echo "   DAMAGE_WS: /home/amov/Desktop/basic/damage_detect_ws"
echo "   AIRSIM_WS: /home/amov/Desktop/Prometheus/Modules/tutorial_demo/basic/catkin_ws"

# 检查话题
echo ""
echo "3. 检查ROS话题:"
echo "   等待5秒收集话题信息..."
sleep 2

# 检查是否有AirSim话题
if rostopic list 2>/dev/null | grep -q "airsim_node"; then
    echo "   ✓ AirSim话题存在:"
    rostopic list | grep airsim_node | head -5
else
    echo "   ✗ 未检测到AirSim话题"
fi

# 检查图像话题频率
echo ""
echo "4. 检查图像话题频率:"
if rostopic info /airsim_node/uav1/front_left/Scene 2>/dev/null | grep -q "Type:"; then
    echo "   ✓ 图像话题存在"
    echo "   正在统计频率（请等待5秒）..."
    timeout 5 rostopic hz /airsim_node/uav1/front_left/Scene 2>/dev/null || echo "   无消息或超时"
else
    echo "   ✗ 图像话题不存在"
fi

echo ""
echo "========================================="
