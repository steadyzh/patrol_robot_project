#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image          # 导入图像消息类型
from std_msgs.msg import Int32MultiArray   # 导入数组消息类型
from geometry_msgs.msg import Twist
# 导入分拆出去的模块
from core.vision_processor import VisionProcessor
# from core.navigation import NavigationManager (以后加)

class PatrolSystem:
    def __init__(self):
        rospy.init_node('patrol_main_node')
        
        # 1. 初始化各子模块
        self.vision = VisionProcessor()
        # self.nav = NavigationManager() 
        
        # 2. 订阅话题（连接传感器）
        rospy.Subscriber("/box", Int32MultiArray, self.vision.box_callback)
        rospy.Subscriber("/repaired_image", Image, self.vision.image_callback)
        
        # 3. 发布话题（连接执行器）
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        # 4. 系统状态
        self.state = "PATROL" # 状态: PATROL(巡逻), ALARM(报警), IDLE(待机)

    def run(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            self.decision_making()
            rate.sleep()

    def decision_making(self):
        """核心决策逻辑：以后加功能主要改这里"""
        
        # 场景 A: 发现入侵者
        if self.vision.person_detected:
            if self.state != "ALARM":
                rospy.logwarn("【状态切换】PATROL -> ALARM")
                self.state = "ALARM"
                self.vision.save_evidence() # 拍照
            
            self.stop_robot()
            # self.audio.play_warning() # 以后可以在这加语音
            
        # 场景 B: 环境安全
        else:
            if self.state == "ALARM":
                rospy.loginfo("【状态切换】ALARM -> PATROL")
                self.state = "PATROL"
            
            # self.nav.continue_patrol() # 继续巡逻

    def stop_robot(self):
        msg = Twist()
        self.cmd_pub.publish(msg)

if __name__ == '__main__':
    try:
        system = PatrolSystem()
        system.run()
    except rospy.ROSInterruptException:
        pass