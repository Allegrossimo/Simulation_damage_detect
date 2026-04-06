#!/usr/bin/env python3
"""测试AirSim连接"""

import rospy
from sensor_msgs.msg import Image
import sys

class AirSimTest:
    def __init__(self):
        rospy.init_node('airsim_test', anonymous=True)
        self.image_received = False
        
        # 订阅图像话题
        self.sub = rospy.Subscriber('/airsim_node/uav1/front_left/Scene', 
                                    Image, self.image_callback)
        
        rospy.loginfo("等待AirSim图像话题...")
        
    def image_callback(self, msg):
        if not self.image_received:
            self.image_received = True
            rospy.loginfo(f"收到第一帧图像! 尺寸: {msg.width}x{msg.height}")
            rospy.loginfo("AirSim连接正常")
    
    def run(self):
        rate = rospy.Rate(10)
        timeout = 30  # 30秒超时
        
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and not self.image_received:
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logerr(f"超时: {timeout}秒内未收到图像消息")
                break
            rate.sleep()
        
        if self.image_received:
            rospy.loginfo("测试完成!")
            return 0
        else:
            rospy.logerr("测试失败: 未收到AirSim图像")
            return 1

if __name__ == '__main__':
    try:
        test = AirSimTest()
        sys.exit(test.run())
    except rospy.ROSInterruptException:
        pass
