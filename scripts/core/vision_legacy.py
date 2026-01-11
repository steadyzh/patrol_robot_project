#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 复现厂家逻辑的脚本 (Python 2/3 均可)

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

class LegacyVisualizer:
    def __init__(self):
        rospy.init_node('legacy_visualizer')
        self.latest_img = None
        # 订阅旧数据 (注意：Bag播放时不要重映射这个话题)
        rospy.Subscriber("/box", Int32MultiArray, self.box_cb)
        rospy.Subscriber("/repaired_image", Image, self.img_cb)
        # 发布对比图
        self.pub = rospy.Publisher('/patrol/debug_legacy', Image, queue_size=1)

    def img_cb(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        self.latest_img = np_arr.reshape((msg.height, msg.width, 3))

    def box_cb(self, msg):
        if self.latest_img is None: return
        data = msg.data
        debug = self.latest_img.copy()
        
        # 厂家的简陋逻辑
        if len(data) >= 2:
            num = data[1]
            coords = data[2:]
            for i in range(num):
                # 简单画个框，不做优化
                base = i*4
                if base+3 < len(coords):
                    x1, y1, x2, y2 = coords[base], coords[base+1], coords[base+2], coords[base+3]
                    cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(debug, "Legacy", (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        
        # 手动打包发布
        out_msg = Image()
        out_msg.header = rospy.Header()
        out_msg.height, out_msg.width, _ = debug.shape
        out_msg.encoding = "bgr8"
        out_msg.step = debug.shape[1]*3
        out_msg.data = debug.tobytes()
        self.pub.publish(out_msg)

if __name__ == '__main__':
    LegacyVisualizer()
    rospy.spin()