#!/usr/bin/env python3
import os
import sys

# 添加脚本路径
sys.path.insert(0, '/home/amov/Desktop/basic/damage_detect_ws/src/damage_detect/scripts')

from utils import get_camera_pose_from_txt

# 测试位姿文件
pose_file = '/tmp/damage_detect/airsim_rec.txt'

if os.path.exists(pose_file):
    print(f"位姿文件存在: {pose_file}")
    
    # 读取文件并显示前几行
    with open(pose_file, 'r') as f:
        lines = f.readlines()
    
    print(f"总行数: {len(lines)}")
    print("\n前3行:")
    for i in range(min(3, len(lines))):
        print(f"  {lines[i].strip()}")
    
    print("\n最后3行:")
    for i in range(max(0, len(lines)-3), len(lines)):
        print(f"  {lines[i].strip()}")
    
    # 测试读取最后一张图片的位姿
    if len(lines) > 1:
        last_line = lines[-1].strip()
        parts = last_line.split('\t')
        if len(parts) >= 10:
            image_name = parts[9]
            print(f"\n测试读取图片: {image_name}")
            
            drone_pos, drone_quat = get_camera_pose_from_txt(pose_file, image_name)
            if drone_pos is not None:
                print(f"  ✓ 成功读取位姿")
                print(f"    位置: {drone_pos}")
                print(f"    四元数: {drone_quat}")
            else:
                print(f"  ✗ 读取失败")
else:
    print(f"位姿文件不存在: {pose_file}")
