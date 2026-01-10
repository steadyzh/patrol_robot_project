#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# 注意：这个文件必须用 python3 运行！

import rospy
import cv2
import torch
import numpy as np
import os
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector_node', anonymous=True)
        
        # 1. 加载 YOLOv5 模型 (第一次运行会自动下载，约14MB)
        rospy.loginfo("正在加载 YOLOv5 模型 (可能需要几分钟下载)...")
        # 使用 ultralytics 官方仓库
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        self.model.classes = [0] # 0 = person
        self.model.conf = 0.4    # 置信度
        
        # 2. 订阅 /repaired_image
        self.sub = rospy.Subscriber("/repaired_image", Image, self.image_callback)
        
        # 3. 发布 /box (模拟厂家格式，骗过 main_node)
        self.box_pub = rospy.Publisher('/box', Int32MultiArray, queue_size=1)
        
        # 4. 发布带框的图给 Rviz
        self.debug_pub = rospy.Publisher('/patrol/debug_image', Image, queue_size=1)
        
        rospy.loginfo("YOLOv5 检测器启动成功！")

    def image_callback(self, msg):
        try:
            # Py3 解码
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = img_data.reshape((msg.height, msg.width, 3))
            
            # 推理 (OpenCV BGR -> RGB)
            results = self.model(cv_img[..., ::-1])
            
            # 解析
            df = results.pandas().xyxy[0]
            
            # 构造厂家格式: [-1, count, x1,y1,x2,y2, ...]
            box_data = [-1, len(df)]
            
            # 画图副本
            debug_img = cv_img.copy()
            
            for i, row in df.iterrows():
                x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                conf = row['confidence']
                
                # 填充数据
                box_data.extend([x1, y1, x2, y2])
                
                # 画框
                label = f"Person {conf:.2f}"
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.rectangle(debug_img, (x1, y1-20), (x1+100, y1), (0, 0, 255), -1)
                cv2.putText(debug_img, label, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)
            
            # 发布 /box
            box_msg = Int32MultiArray(data=box_data)
            self.box_pub.publish(box_msg)
            
            # 发布调试图
            self.publish_ros_image(debug_img)
            
        except Exception as e:
            print(f"Error: {e}")

    def publish_ros_image(self, img):
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "bgr8"
        msg.step = len(img[0]) * 3
        msg.data = img.tobytes()
        self.debug_pub.publish(msg)

if __name__ == '__main__':
    YoloDetector()
    rospy.spin()