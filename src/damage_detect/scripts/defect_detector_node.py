#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
import os
import sys
import time
import socket
from collections import defaultdict
import threading
import queue

# 添加当前目录到Python路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 导入自定义模块
from tracker import DamageTracker
from utils import get_camera_pose_from_txt, calculate_camera_pose, pixel_to_world
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO

class DefectDetectorNode:
    def __init__(self):
        rospy.init_node('defect_detector', anonymous=True)
        
        # 加载参数
        self.load_params()
        
        # 初始化UDP客户端
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 初始化YOLO模型
        if os.path.exists(self.model_path):
            self.model = YOLO(self.model_path)
            rospy.loginfo(f"加载模型: {self.model_path}")
        else:
            rospy.logerr(f"模型文件不存在: {self.model_path}")
            rospy.loginfo("使用默认yolov8n.pt模型")
            self.model = YOLO("yolov8n.pt")
        
        # 初始化跟踪器
        self.tracker = DamageTracker(
            window_duration=self.window_duration,
            cluster_distance=self.cluster_distance,
            vote_threshold=self.vote_threshold
        )
        
        # 初始化CV Bridge
        self.bridge = CvBridge()
        
        # 检测线程
        self.detect_thread = threading.Thread(target=self.detection_loop)
        self.detect_thread.daemon = True
        self.running = True
        
        # 图像队列
        self.image_queue = queue.Queue(maxsize=100)
        
        # 启动检测线程
        self.detect_thread.start()
        
        # 订阅图像话题
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        rospy.loginfo("="*60)
        rospy.loginfo("病害检测节点已启动")
        rospy.loginfo(f"检测策略: 每{self.window_duration}秒检测{self.frames_per_window}帧, 至少{self.vote_threshold}帧确认")
        rospy.loginfo(f"空间聚类阈值: {self.cluster_distance}m")
        rospy.loginfo(f"UDP目标: {self.windows_ip}:{self.windows_port}")
        rospy.loginfo("="*60)
    
    def load_params(self):
        """加载参数"""
        # 模型参数
        self.model_path = rospy.get_param('~model_path', 
            '/home/amov/Desktop/basic/damage_detect_ws/src/damage_detect/models/best.pt')
        self.conf_threshold = rospy.get_param('~conf_threshold', 0.3)
        
        # 检测策略参数
        self.window_duration = rospy.get_param('~window_duration', 0.5)
        self.frames_per_window = rospy.get_param('~frames_per_window', 3)
        self.vote_threshold = rospy.get_param('~vote_threshold', 2)
        self.cluster_distance = rospy.get_param('~cluster_distance', 1.5)
        
        # 相机参数
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        self.fov_degrees = rospy.get_param('~fov_degrees', 80.0)
        self.road_z = rospy.get_param('~road_z', 0.0)
        
        # 相机偏移
        self.camera_offset = {
            'X': rospy.get_param('~camera_offset_x', 0),
            'Y': rospy.get_param('~camera_offset_y', 0),
            'Z': rospy.get_param('~camera_offset_z', 0.0)
        }
        self.camera_euler = {
            'pitch': rospy.get_param('~camera_pitch', 0),
            'roll': rospy.get_param('~camera_roll', 0),
            'yaw': rospy.get_param('~camera_yaw', 0)
        }
        
        # ROS话题
        self.image_topic = rospy.get_param('~image_topic', 
            '/airsim_node/uav1/front_left/Scene')
        self.pose_file = rospy.get_param('~pose_file',
            '/tmp/damage_detect/airsim_rec.txt')
        
        # 网络配置
        self.windows_ip = rospy.get_param('~windows_ip', '10.193.133.230')
        self.windows_port = rospy.get_param('~windows_port', 8891)
        
        # 病害类型映射（数字代号）
        self.defect_types = {
            0: 0,  # 横向裂缝
            1: 1,  # 纵向裂缝
            2: 2,  # 龟裂
            3: 3,  # 坑槽
            4: 4   # 裂缝
        }
        
        # 输出格式（数字代号）
        self.output_format = "MsgType:1 DamageType:{} D_x:{:.3f} D_y:{:.3f} D_z:{:.3f}"
    
    def image_callback(self, msg):
        """图像回调函数 - 只将图像放入队列"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 获取时间戳
            timestamp = msg.header.stamp.to_nsec()
            
            # 放入队列
            if self.image_queue.qsize() < self.frames_per_window * 2:
                self.image_queue.put({
                    'image': cv_image,
                    'timestamp': timestamp,
                    'frame_id': msg.header.seq
                })
                
        except CvBridgeError as e:
            rospy.logerr(f"图像转换错误: {e}")
        except Exception as e:
            rospy.logerr(f"图像回调错误: {e}")
    
    def detection_loop(self):
        """检测循环 - 每0.5秒检测3帧"""
        last_detect_time = time.time()
        
        while self.running and not rospy.is_shutdown():
            current_time = time.time()
            
            # 收集帧直到达到时间窗口
            if current_time - last_detect_time >= self.window_duration:
                # 从队列中取出指定数量的帧
                frames_to_detect = []
                while len(frames_to_detect) < self.frames_per_window and not self.image_queue.empty():
                    try:
                        frame_data = self.image_queue.get_nowait()
                        frames_to_detect.append(frame_data)
                    except queue.Empty:
                        break
                
                # 如果收集到了足够的帧，进行检测
                if len(frames_to_detect) >= self.vote_threshold:
                    self.detect_frames(frames_to_detect)
                
                last_detect_time = current_time
            
            # 短暂休眠
            time.sleep(0.01)
    
    def detect_frames(self, frames):
        """检测多帧图像并进行投票"""
        all_detections = []
        
        for frame_data in frames:
            # 生成图片名称
            image_name = f"img_uav1_{frame_data['timestamp']}.png"
            
            camera_pos, camera_quat = self.get_current_pose(image_name)
            
            if camera_pos is None:
                rospy.logwarn_throttle(5, f"无法获取相机位姿: {image_name}")
                continue
            
            # YOLO检测
            try:
                results = self.model(frame_data['image'], conf=self.conf_threshold, verbose=False)
                
                if results[0].boxes is not None:
                    boxes = results[0].boxes.xyxy.cpu().numpy()
                    classes = results[0].boxes.cls.cpu().numpy().astype(int)
                    confs = results[0].boxes.conf.cpu().numpy()
                    
                    for box, cls, conf in zip(boxes, classes, confs):
                        # 计算检测框中心点
                        x1, y1, x2, y2 = map(int, box)
                        center_u = (x1 + x2) // 2
                        center_v = (y1 + y2) // 2
                        
                        # 转换为世界坐标
                        world_pos, distance = pixel_to_world(
                            center_u, center_v,
                            self.image_width, self.image_height,
                            self.fov_degrees,
                            camera_pos, camera_quat,
                            self.road_z
                        )
                        
                        if world_pos is not None and cls in self.defect_types:
                            all_detections.append({
                                'pos': world_pos,
                                'type': self.defect_types[cls],
                                'conf': conf,
                                'timestamp': frame_data['timestamp']
                            })
            except Exception as e:
                rospy.logerr(f"YOLO检测错误: {e}")
        
        # 进行投票和聚类
        if all_detections:
            self.vote_and_cluster(all_detections)
    
    def vote_and_cluster(self, detections):
        """投票和聚类"""
        # 按类型分组
        detections_by_type = defaultdict(list)
        for det in detections:
            detections_by_type[det['type']].append(det)
        
        # 对每种类型进行聚类
        for defect_type, type_detections in detections_by_type.items():
            # 空间聚类
            clusters = self.cluster_detections(type_detections, self.cluster_distance)
            
            # 投票
            for cluster in clusters:
                if len(cluster) >= self.vote_threshold:
                    # 计算平均位置
                    avg_pos = np.mean([d['pos'] for d in cluster], axis=0)
                    
                    # 发送检测结果
                    self.send_damage_result(defect_type, avg_pos)
    
    def cluster_detections(self, detections, distance_threshold):
        """空间聚类"""
        if not detections:
            return []
        
        clusters = []
        for det in detections:
            matched = False
            for cluster in clusters:
                center = np.mean([d['pos'] for d in cluster], axis=0)
                if np.linalg.norm(det['pos'] - center) < distance_threshold:
                    cluster.append(det)
                    matched = True
                    break
            
            if not matched:
                clusters.append([det])
        
        return clusters
    
    def send_damage_result(self, defect_type, position):
        """通过UDP发送检测结果"""
        msg = self.output_format.format(
            defect_type,
            position[0],
            position[1],
            position[2]
        )
        
        try:
            self.udp_socket.sendto(msg.encode(), (self.windows_ip, self.windows_port))
            rospy.loginfo(f"发送病害: {msg}")
        except Exception as e:
            rospy.logerr(f"UDP发送失败: {e}")
    
    def get_current_pose(self, image_name):
        """获取当前图像的相机位姿"""
        if not os.path.exists(self.pose_file):
            return None, None
        
        try:
            # 首先尝试精确匹配
            drone_pos, drone_quat = get_camera_pose_from_txt(self.pose_file, image_name)
            
            # 如果精确匹配失败，使用最新的位姿
            if drone_pos is None:
                with open(self.pose_file, 'r') as f:
                    lines = f.readlines()
                    if len(lines) > 1:
                        last_line = lines[-1].strip()
                        parts = last_line.split('\t')
                        if len(parts) >= 10:
                            drone_pos = np.array([
                                float(parts[2]), float(parts[3]), float(parts[4])
                            ])
                            drone_quat = [
                                float(parts[6]), float(parts[7]), float(parts[8]), float(parts[5])
                            ]
            
            if drone_pos is not None:
                camera_pos, camera_quat = calculate_camera_pose(
                    drone_pos, drone_quat, self.camera_offset, self.camera_euler
                )
                return camera_pos, camera_quat
                
        except Exception as e:
            rospy.logdebug(f"读取位姿失败: {e}")
        
        return None, None
    
    def shutdown(self):
        """关闭节点"""
        self.running = False
        self.udp_socket.close()
        rospy.loginfo("检测节点已关闭")

def main():
    try:
        node = DefectDetectorNode()
        rospy.on_shutdown(node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"节点运行错误: {e}")

if __name__ == '__main__':
    main()
