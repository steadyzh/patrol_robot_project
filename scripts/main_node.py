#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from core.vision_processor import VisionProcessor

class PatrolSystem:
    def __init__(self):
        rospy.init_node('patrol_main_node')
        
        # 1. 注册关闭时的回调函数（安全机制）
        rospy.on_shutdown(self.shutdown_hook)
        
        # 2. 初始化模块
        self.vision = VisionProcessor()
        self.evidence_saved = False 
        
        # 3. 订阅话题
        # 订阅 YOLO 发出的 /yolo_box
        rospy.Subscriber("/yolo_box", Int32MultiArray, self.vision.box_callback)
        # 订阅 清晰图像
        rospy.Subscriber("/debug_image", Image, self.vision.image_callback)
        
        # 4. 发布指令
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.state = "PATROL" 
        rospy.loginfo("巡检系统主程序已启动...")

    def run(self):
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            self.decision_making()
            rate.sleep()

    def decision_making(self):
        # 场景 A: 发现入侵者
        if self.vision.person_detected:
            if self.state != "ALARM":
                rospy.logwarn("【状态切换】PATROL -> ALARM (发现入侵者)")
                self.state = "ALARM"
                self.evidence_saved = False 
            
            # 停车
            self.stop_robot()
            
            # 存证
            if not self.evidence_saved:
                if self.vision.latest_image is not None:
                    self.vision.save_evidence()
                    self.evidence_saved = True 
                else:
                    rospy.logwarn_throttle(1, "报警触发但无图，等待下一帧...")
            
        # 场景 B: 环境安全
        else:
            if self.state == "ALARM":
                rospy.loginfo("【状态切换】ALARM -> PATROL (警报解除)")
                self.state = "PATROL"
            
            # 这里以后可以加巡航代码
            # self.nav.patrol()

    def stop_robot(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def shutdown_hook(self):
        rospy.logwarn("正在关闭系统... 发送紧急停车指令！")
        self.stop_robot()

if __name__ == '__main__':
    try:
        system = PatrolSystem()
        system.run()
    except rospy.ROSInterruptException:
        pass