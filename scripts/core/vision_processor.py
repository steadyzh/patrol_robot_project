#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import os
import datetime
from sensor_msgs.msg import Image

class VisionProcessor:
    def __init__(self):
        self.latest_image = None
        self.person_detected = False
        self.detected_objects = [] 

    def image_callback(self, msg):
        try:
            # 接收 YOLO 画好框的图
            np_arr = np.fromstring(msg.data, np.uint8)
            self.latest_image = np_arr.reshape((msg.height, msg.width, 3))
        except Exception as e:
            pass

    def box_callback(self, msg):
        data = msg.data
        self.detected_objects = []
        if len(data) < 2: 
            self.person_detected = False
            return
        count = data[1]
        if len(data) == 2 + count * 5 and count > 0:
            self.person_detected = True
            coords = data[2:]
            for i in range(count):
                base = i * 5
                obj = {
                    'x1': coords[base], 'y1': coords[base+1],
                    'x2': coords[base+2], 'y2': coords[base+3],
                    'id': coords[base+4]
                }
                self.detected_objects.append(obj)
        else:
            self.person_detected = False

    def save_evidence(self):
        if self.latest_image is None:
            rospy.logwarn("存图失败：当前无图像数据")
            return 

        try:
            home_path = os.path.expanduser("~") 
            save_dir = os.path.join(home_path, "patrol_evidences")
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)

            # === 改进：使用通用时间格式命名 ===
            now_str = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            ids = [str(obj['id']) for obj in self.detected_objects]
            id_str = "_".join(ids) if ids else "Unknown"
            
            filename = "Alert_{}_ID{}.jpg".format(now_str, id_str)
            full_path = os.path.join(save_dir, filename)
            
            # 直接保存收到的图（已经是带框带FPS的了）
            cv2.imwrite(full_path, self.latest_image)
            rospy.loginfo("【已存证】 %s", full_path)
                
        except Exception as e:
            rospy.logerr("存图异常: %s", e)