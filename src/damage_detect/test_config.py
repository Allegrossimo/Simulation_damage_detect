#!/usr/bin/env python3
"""测试配置是否正确 - 适配新工作空间"""

import os
import sys
import subprocess

def test_python_dependencies():
    """测试Python依赖"""
    print("\n=== 测试Python依赖 ===")
    
    dependencies = [
        ('ultralytics', 'YOLO'),
        ('cv2', 'OpenCV'),
        ('numpy', 'NumPy'),
        ('scipy', 'SciPy'),
        ('rospy', 'ROS Python'),
        ('cv_bridge', 'CV Bridge'),
        ('sensor_msgs', 'Sensor Messages')
    ]
    
    all_ok = True
    for module, name in dependencies:
        try:
            __import__(module)
            print(f"✓ {name} 已安装")
        except ImportError:
            print(f"✗ {name} 未安装")
            all_ok = False
    
    return all_ok

def test_file_structure():
    """测试文件结构"""
    print("\n=== 测试文件结构 ===")
    
    base_path = '/home/amov/Desktop/basic/damage_detect_ws/src/damage_detect'
    
    required_files = [
        'src/airsim_recorder_node.cpp',
        'scripts/defect_detector_node.py',
        'scripts/tracker.py',
        'scripts/utils.py',
        'launch/damage_detect.launch',
        'CMakeLists.txt',
        'package.xml'
    ]
    
    all_ok = True
    for file in required_files:
        full_path = os.path.join(base_path, file)
        if os.path.exists(full_path):
            print(f"✓ {file} 存在")
        else:
            print(f"✗ {file} 不存在")
            all_ok = False
    
    return all_ok

def test_ros_environment():
    """测试ROS环境"""
    print("\n=== 测试ROS环境 ===")
    
    try:
        # 检查ROS环境变量
        ros_version = os.environ.get('ROS_DISTRO')
        if ros_version:
            print(f"✓ ROS版本: {ros_version}")
        else:
            print("✗ ROS环境未设置")
            return False
        
        return True
    except Exception as e:
        print(f"✗ ROS环境错误: {e}")
        return False

def test_compile():
    """测试编译"""
    print("\n=== 测试编译 ===")
    
    workspace_path = '/home/amov/Desktop/basic/damage_detect_ws'
    
    try:
        # 编译包
        cmd = f'cd {workspace_path} && catkin_make'
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        
        if result.returncode == 0:
            print("✓ 编译成功")
            return True
        else:
            print("✗ 编译失败")
            print(result.stderr)
            return False
    except Exception as e:
        print(f"✗ 编译错误: {e}")
        return False

def main():
    print("="*60)
    print("道路病害检测模块配置测试")
    print(f"工作空间: /home/amov/Desktop/basic/damage_detect_ws")
    print("="*60)
    
    # 运行所有测试
    tests = [
        test_python_dependencies,
        test_file_structure,
        test_ros_environment,
        test_compile
    ]
    
    results = []
    for test in tests:
        try:
            results.append(test())
        except Exception as e:
            print(f"测试失败: {e}")
            results.append(False)
    
    print("\n" + "="*60)
    print("测试总结:")
    print("="*60)
    
    passed = sum(results)
    total = len(results)
    
    if passed == total:
        print(f"✓ 所有测试通过 ({passed}/{total})")
        print("\n可以启动节点:")
        print("cd /home/amov/Desktop/basic/damage_detect_ws")
        print("source devel/setup.bash")
        print("roslaunch damage_detect damage_detect.launch")
    else:
        print(f"✗ {total-passed}/{total} 个测试失败")
        print("\n请检查并修复上述问题")
    
    print("="*60)

if __name__ == '__main__':
    main()
