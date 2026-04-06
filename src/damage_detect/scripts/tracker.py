#!/usr/bin/env python3
import numpy as np
from collections import defaultdict
import time

class DamageTracker:
    def __init__(self, window_duration=0.5, cluster_distance=2, vote_threshold=2):
        """
        病害时间窗口跟踪器
        window_duration: 时间窗口长度(秒)
        cluster_distance: 空间聚类距离阈值(米)
        vote_threshold: 投票阈值(至少几帧检测到)
        """
        self.window_duration = window_duration
        self.cluster_distance = cluster_distance
        self.vote_threshold = vote_threshold
        
        # 存储当前窗口内的检测结果
        self.current_window = []  # 每个元素: {'pos': [x,y,z], 'type': int, 'timestamp': float}
        self.window_start_time = None
        
        # 存储已确认的病害
        self.confirmed_defects = []  # 每个元素: {'pos': [x,y,z], 'type': int, 'count': int}
        
    def add_detection(self, pos, defect_type, timestamp):
        """添加一个检测结果"""
        if pos is None:
            return
        
        # 检查是否需要开始新窗口
        if self.window_start_time is None:
            self.window_start_time = timestamp
            self.current_window = []
        
        # 如果时间窗口过期，处理当前窗口
        if timestamp - self.window_start_time > self.window_duration:
            self._process_window()
            self.window_start_time = timestamp
            self.current_window = []
        
        # 添加到当前窗口
        self.current_window.append({
            'pos': np.array(pos),
            'type': defect_type,
            'timestamp': timestamp
        })
    
    def _process_window(self):
        """处理当前时间窗口内的检测结果"""
        if not self.current_window:
            return
        
        # 按病害类型分组处理
        defects_by_type = defaultdict(list)
        for d in self.current_window:
            defects_by_type[d['type']].append(d)
        
        # 对每种病害类型进行空间聚类
        for defect_type, detections in defects_by_type.items():
            clusters = self._cluster_detections(detections, self.cluster_distance)
            
            # 对每个聚类检查是否达到投票阈值
            for cluster in clusters:
                if len(cluster) >= self.vote_threshold:
                    # 计算平均位置
                    avg_pos = np.mean([p['pos'] for p in cluster], axis=0)
                    
                    # 检查是否与已确认的病害重复
                    is_duplicate = False
                    for confirmed in self.confirmed_defects:
                        if confirmed['type'] == defect_type:
                            dist = np.linalg.norm(avg_pos - confirmed['pos'])
                            if dist < self.cluster_distance:
                                # 更新已确认病害的位置（移动平均）
                                confirmed['pos'] = (confirmed['pos'] * confirmed['count'] + avg_pos) / (confirmed['count'] + 1)
                                confirmed['count'] += len(cluster)
                                confirmed['last_seen'] = time.time()
                                is_duplicate = True
                                break
                    
                    if not is_duplicate:
                        self.confirmed_defects.append({
                            'pos': avg_pos,
                            'type': defect_type,
                            'count': len(cluster),
                            'first_seen': time.time(),
                            'last_seen': time.time()
                        })
    
    def _cluster_detections(self, detections, distance_threshold):
        """空间聚类"""
        if not detections:
            return []
        
        clusters = []
        for det in detections:
            matched = False
            for cluster in clusters:
                # 计算与聚类中心的距离
                center = np.mean([d['pos'] for d in cluster], axis=0)
                if np.linalg.norm(det['pos'] - center) < distance_threshold:
                    cluster.append(det)
                    matched = True
                    break
            
            if not matched:
                clusters.append([det])
        
        return clusters
    
    def get_confirmed_defects(self):
        """获取已确认的病害列表"""
        # 清理过期的病害（超过10秒未出现）
        current_time = time.time()
        self.confirmed_defects = [d for d in self.confirmed_defects 
                                  if current_time - d['last_seen'] < 10.0]
        return self.confirmed_defects
    
    def clear_old_defects(self, timeout=10.0):
        """清除超时未出现的病害"""
        current_time = time.time()
        self.confirmed_defects = [d for d in self.confirmed_defects 
                                  if current_time - d['last_seen'] < timeout]
