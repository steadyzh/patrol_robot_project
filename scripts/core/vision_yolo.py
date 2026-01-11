#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import torch
import numpy as np
import math
import random
import time
import datetime
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

class EuclideanDistTracker:
    def __init__(self):
        self.center_points = {}
        self.id_count = 0

    def update(self, objects_rect):
        objects_bbs_ids = []
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2
            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = math.hypot(cx - pt[0], cy - pt[1])
                if dist < 150: 
                    self.center_points[id] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, id])
                    same_object_detected = True
                    break
            if not same_object_detected:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.id_count])
                self.id_count += 1
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center
        self.center_points = new_center_points.copy()
        return objects_bbs_ids

class YoloDetector:
    def __init__(self):
        rospy.init_node('yolo_detector_node', anonymous=True)
        rospy.loginfo("正在加载 YOLOv5n (Nano) 模型...")
        try:
            self.model = torch.hub.load('ultralytics/yolov5:v6.1', 'yolov5n', pretrained=True)
        except Exception:
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)
        self.model.classes = [0] 
        self.model.conf = 0.4    
        self.model.iou = 0.45    
        self.tracker = EuclideanDistTracker()
        self.color_map = {} 
        self.prev_frame_time = 0
        self.sub = rospy.Subscriber("/repaired_image", Image, self.image_callback)
        self.box_pub = rospy.Publisher('/yolo_box', Int32MultiArray, queue_size=1)
        self.debug_pub = rospy.Publisher('/patrol/debug_image', Image, queue_size=1)
        rospy.loginfo("YOLOv5n 启动成功！准备检测...")

    def get_color(self, id):
        if id not in self.color_map:
            random.seed(id)
            self.color_map[id] = (random.randint(50, 255), random.randint(50, 255), random.randint(50, 255))
        return self.color_map[id]

    def image_callback(self, msg):
        try:
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            cv_img = img_data.reshape((msg.height, msg.width, 3))
            results = self.model(cv_img[..., ::-1]) 
            df = results.pandas().xyxy[0]
            detections = []
            for _, row in df.iterrows():
                x1, y1, x2, y2 = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                w = x2 - x1
                h = y2 - y1
                detections.append([x1, y1, w, h])
            boxes_ids = self.tracker.update(detections)
            
            box_data = [-1, len(boxes_ids)] 
            debug_img = cv_img.copy()
            for box_id in boxes_ids:
                x, y, w, h, id = box_id
                x2 = x + w
                y2 = y + h
                box_data.extend([x, y, x2, y2, id])
                color = self.get_color(id)
                cv2.rectangle(debug_img, (x, y), (x2, y2), color, 2)
                label = f"ID:{id}"
                t_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)[0]
                cv2.rectangle(debug_img, (x, y - t_size[1] - 5), (x + t_size[0], y), color, -1)
                cv2.putText(debug_img, label, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            
            self.new_frame_time = time.time()
            fps = 1 / (self.new_frame_time - self.prev_frame_time) if (self.new_frame_time - self.prev_frame_time) > 0 else 0
            self.prev_frame_time = self.new_frame_time
            now = datetime.datetime.now()
            time_str = now.strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(debug_img, f"FPS: {int(fps)}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(debug_img, time_str, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            
            self.box_pub.publish(Int32MultiArray(data=box_data))
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