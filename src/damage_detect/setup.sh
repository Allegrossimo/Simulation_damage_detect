#!/bin/bash
echo "========================================="
echo "道路病害检测模块安装脚本"
echo "工作空间: /home/amov/Desktop/basic/damage_detect_ws"
echo "========================================="

# 设置工作空间路径
WORKSPACE_PATH="/home/amov/Desktop/basic/damage_detect_ws"
MODULE_PATH="$WORKSPACE_PATH/src/damage_detect"

# 检查工作空间
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "错误: 工作空间不存在: $WORKSPACE_PATH"
    exit 1
fi

# 进入工作空间
cd $WORKSPACE_PATH

# 安装Python依赖
echo ""
echo "安装Python依赖..."
pip3 install ultralytics opencv-python numpy scipy

# 创建必要的目录
echo ""
echo "创建目录结构..."
mkdir -p $MODULE_PATH/models
mkdir -p $MODULE_PATH/config
mkdir -p $MODULE_PATH/launch
mkdir -p $MODULE_PATH/scripts
mkdir -p $MODULE_PATH/src
mkdir -p $MODULE_PATH/include

# 设置脚本权限
echo ""
echo "设置脚本权限..."
chmod +x $MODULE_PATH/scripts/*.py 2>/dev/null || true
chmod +x $MODULE_PATH/test_config.py 2>/dev/null || true

# 编译
echo ""
echo "编译ROS包..."
catkin_make

# 检查编译结果
if [ $? -eq 0 ]; then
    echo "✓ 编译成功"
else
    echo "✗ 编译失败"
    exit 1
fi

# 设置环境变量
echo ""
echo "设置环境变量..."
source devel/setup.bash

# 运行配置测试
echo ""
echo "运行配置测试..."
python3 $MODULE_PATH/test_config.py

echo ""
echo "========================================="
echo "安装完成！"
echo ""
echo "启动命令:"
echo "  cd $WORKSPACE_PATH"
echo "  source devel/setup.bash"
echo "  roslaunch damage_detect damage_detect.launch"
echo "========================================="
