#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import os
from sensor_msgs.msg import Image

class VisionProcessor:
    def __init__(self):
        self.latest_image = None
        self.person_detected = False
        self.detected_objects = [] 
        # 注意：这里不再需要 CvBridge 和 debug_pub，因为画图工作交给 YOLO 了

    def image_callback(self, msg):
        try:
            # 仅保存原始图片用于存证
            np_arr = np.fromstring(msg.data, np.uint8)
            self.latest_image = np_arr.reshape((msg.height, msg.width, 3))
        except Exception as e:
            rospy.logerr_throttle(10, "Processor Image Decode Error: %s", e)

    def box_callback(self, msg):
        """
        解析 /yolo_box
        格式: [lock_id, count,  x1, y1, x2, y2, id,  x1, y1, x2, y2, id ...]
        """
        data = msg.data
        self.detected_objects = []
        
        if len(data) < 2: 
            self.person_detected = False
            return
            
        count = data[1]
        
        # 简单校验：头部2位 + 每个人5位
        if len(data) == 2 + count * 5 and count > 0:
            self.person_detected = True
            
            coords = data[2:]
            for i in range(count):
                base = i * 5
                obj = {
                    'x1': coords[base],
                    'y1': coords[base+1],
                    'x2': coords[base+2],
                    'y2': coords[base+3],
                    'id': coords[base+4]
                }
                self.detected_objects.append(obj)
        else:
            self.person_detected = False

    def save_evidence(self):
        """只保存原始画面，带上文件名信息即可"""
        if self.latest_image is None:
            rospy.logwarn("存证失败：无图像数据")
            return 

        try:
            home_path = os.path.expanduser("~") 
            save_dir = os.path.join(home_path, "patrol_evidences")
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)

            timestamp = rospy.Time.now().to_nsec()
            
            # 获取入侵者ID列表
            ids = [str(obj['id']) for obj in self.detected_objects]
            id_str = "_".join(ids) if ids else "Unknown"
            
            filename = "Alert_ID{}_{}.jpg".format(id_str, timestamp)
            full_path = os.path.join(save_dir, filename)
            
            # 可以在存证图片上简单画个框（可选），为了证据直观
            save_img = self.latest_image.copy()
            for obj in self.detected_objects:
                cv2.rectangle(save_img, (obj['x1'], obj['y1']), (obj['x2'], obj['y2']), (0,0,255), 2)
            
            cv2.imwrite(full_path, save_img)
            rospy.loginfo("【已存证】%s", full_path)
                
        except Exception as e:
            rospy.logerr("保存文件出错: %s", e)