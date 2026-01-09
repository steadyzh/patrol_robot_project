#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

class VisionProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.latest_image = None
        self.person_detected = False
        self.target_info = {} # 存坐标

    def image_callback(self, msg):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr("CV Bridge Error: %s", e)

    def box_callback(self, msg):
        data = msg.data
        if len(data) > 0:
            self.person_detected = True
            # 解析坐标 [id, label, x, y, w, h]
            self.target_info = {
                'x': data[2],
                'y': data[3],
                'w': data[4],
                'h': data[5]
            }
        else:
            self.person_detected = False

    def save_evidence(self):
        """保存当前画面作为证据"""
        if self.latest_image is not None:
            filename = "evidence_{}.jpg".format(rospy.Time.now())
            cv2.imwrite(filename, self.latest_image)
            rospy.loginfo("已保存证据: %s", filename)