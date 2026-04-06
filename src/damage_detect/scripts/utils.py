#!/usr/bin/env python3
import numpy as np
import os
from scipy.spatial.transform import Rotation as R

def get_camera_pose_from_txt(txt_path, image_name):
    """从txt文件中读取指定图像的相机位姿"""
    if not os.path.exists(txt_path):
        return None, None
    
    try:
        with open(txt_path, 'r') as f:
            lines = f.readlines()
        
        # 从最后一行开始读取（最新的数据）
        for line in reversed(lines[1:]):  # 跳过表头，从后往前读
            data = line.strip().split('\t')
            if len(data) < 10:
                continue
            img_name = data[9]  # ImageFile列
            # 直接比较文件名
            if img_name == image_name:
                drone_pos = np.array([
                    float(data[2]), float(data[3]), float(data[4])
                ])
                drone_quat = [
                    float(data[6]), float(data[7]), float(data[8]), float(data[5])
                ]
                return drone_pos, drone_quat
            # 添加调试输出
            # print(f"Comparing: {img_name} vs {image_name}")
    except Exception as e:
        print(f"读取位姿文件错误: {e}")
    
    return None, None

def calculate_camera_pose(drone_pos, drone_quat, camera_offset, camera_euler):
    """计算相机位姿"""
    try:
        rot_drone = R.from_quat(drone_quat)
        offset = np.array([camera_offset['X'], camera_offset['Y'], camera_offset['Z']])
        camera_pos = drone_pos + rot_drone.apply(offset)
        
        rot_camera_relative = R.from_euler('xyz',
                                           [camera_euler['pitch'],
                                            camera_euler['roll'],
                                            camera_euler['yaw']],
                                           degrees=True)
        rot_camera = rot_drone * rot_camera_relative
        camera_quat = rot_camera.as_quat()
        
        return camera_pos, camera_quat
    except Exception as e:
        print(f"计算相机位姿错误: {e}")
        return None, None

def pixel_to_world(u, v, image_width, image_height, fov_degrees, 
                   camera_pos, camera_quat, z_road):
    """像素坐标转世界坐标"""
    try:
        # 计算焦距（像素单位）
        fov_rad = np.deg2rad(fov_degrees)
        focal_length = (image_width / 2.0) / np.tan(fov_rad / 2.0)
        
        # 图像中心
        cx, cy = image_width / 2.0, image_height / 2.0
        
        # 相机坐标系：前右下 (X向前, Y向右, Z向下)
        dir_c_x = 1.0
        dir_c_y = (cx - u) / focal_length
        dir_c_z = (cy - v) / focal_length
        
        ray_cam = np.array([dir_c_x, dir_c_y, dir_c_z])
        ray_cam = ray_cam / np.linalg.norm(ray_cam)
        
        # 旋转到世界坐标系
        rot = R.from_quat(camera_quat)
        ray_world = rot.apply(ray_cam)
        
        # 计算与地面的交点
        if abs(ray_world[2]) < 1e-6:
            return None, None
        
        t = (z_road - camera_pos[2]) / ray_world[2]
        
        if t < 0:
            return None, None
        
        p_intersect = camera_pos + t * ray_world
        
        # ========== 手动添加固定偏移修正 ==========
        # 根据实际偏差调整这些数值（单位：米）
        # 如果标记偏X正向，增加正值；偏X负向，增加负值
        # 如果标记偏Y正向，增加正值；偏Y负向，增加负值
        OFFSET_X = -1.2  # 修改这个值来修正X方向偏差
        OFFSET_Y = -1.7  # 修改这个值来修正Y方向偏差
        OFFSET_Z = 0.0  # 修改这个值来修正Z方向偏差
        # ========================================
        
        p_intersect[0] += OFFSET_X
        p_intersect[1] += OFFSET_Y
        p_intersect[2] += OFFSET_Z
        
        distance = np.linalg.norm(p_intersect - camera_pos)
        
        return p_intersect, distance
    except Exception as e:
        print(f"坐标转换错误: {e}")
        return None, None

def world_to_pixel(point_3d, image_width, image_height, fov_degrees,
                   camera_pos, camera_quat):
    """将世界坐标转换为像素坐标"""
    try:
        # 转换到相机坐标系
        point_cam = point_3d - camera_pos
        rot = R.from_quat(camera_quat)
        point_cam = rot.inv().apply(point_cam)
        
        if point_cam[0] <= 0:
            return None, None
        
        # 计算焦距
        fov_rad = np.deg2rad(fov_degrees)
        focal_length = (image_width / 2.0) / np.tan(fov_rad / 2.0)
        
        # 投影
        u = (point_cam[1] / point_cam[0]) * focal_length + image_width / 2.0
        v = (point_cam[2] / point_cam[0]) * focal_length + image_height / 2.0
        
        return int(u), int(v)
    except Exception as e:
        print(f"坐标转换错误: {e}")
        return None, None
