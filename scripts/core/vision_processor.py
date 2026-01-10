#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import random
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

class VisionProcessor:
    def __init__(self):
        self.latest_image = None
        self.person_detected = False
        self.detected_objects = [] 
        self.debug_pub = rospy.Publisher('/patrol/debug_image', Image, queue_size=1)
        self.bridge = CvBridge() 
        self.last_box_time = rospy.Time(0) # 记录最后一次收到box的时间
        
        # 颜色池：为不同ID分配固定颜色
        self.color_map = {}

    def get_color(self, track_id):
        if track_id not in self.color_map:
            # 为新ID随机生成一个颜色 (BGR)
            self.color_map[track_id] = (
                random.randint(50, 255), 
                random.randint(50, 255), 
                random.randint(50, 255)
            )
        return self.color_map[track_id]

    def image_callback(self, msg):
        try:
            # 1. 手动解码 (Tobytes -> Numpy)
            np_arr = np.fromstring(msg.data, np.uint8)
            self.latest_image = np_arr.reshape((msg.height, msg.width, 3))
            
            # 2. 只要有图，就根据当前的检测结果画框
            # 这一步必须放在 image_callback 里，保证画面刷新时框也刷新
            self.publish_debug_image()
                
        except Exception as e:
            pass

    def box_callback(self, msg):
        """
        修复版解析逻辑
        数据结构: [target_id, count, x1, y1, x2, y2, ...]
        """
        # 更新时间戳
        self.last_box_time = rospy.Time.now()
        data = msg.data
        
        # === 关键修复 1: 每次收到新数据，立刻清空旧列表 ===
        # 这能解决“人走了框还在”的问题
        self.detected_objects = [] 
        
        # 基础长度校验
        if len(data) < 2: 
            self.person_detected = False
            return
        
        target_idx = data[0] # 锁定目标的索引 (不是ID)
        num_people = data[1] # 总人数
        
        # === 关键修复 2: 严格校验数据长度，消除“长线”BUG ===
        # 理论长度应该是: 2 (头) + num_people * 4 (坐标)
        expected_len = 2 + num_people * 4
        
        if len(data) != expected_len:
            # 如果长度对不上，说明数据包损坏或串位，直接丢弃，不画
            # rospy.logwarn("脏数据丢弃: 预期长度 %d, 实际 %d", expected_len, len(data))
            return

        coords = data[2:]    # 坐标数据
        
        if num_people > 0:
            self.person_detected = True
            
            for i in range(num_people):
                base = i * 4
                x1 = coords[base]
                y1 = coords[base+1]
                x2 = coords[base+2]
                y2 = coords[base+3]
                
                # === 关键修复 3: 坐标合法性清洗 ===
                # 厂家算法有时会输出 0,0,0,0 或者负数，导致画出怪线
                if x1 == 0 and y1 == 0 and x2 == 0 and y2 == 0:
                    continue
                if abs(x1-x2) > 2000 or abs(y1-y2) > 2000: # 过滤飞出天际的框
                    continue

                obj = {
                    'id': i, # 暂时用索引当ID
                    'is_locked': (i == target_idx),
                    'x1': x1, 'y1': y1, 
                    'x2': x2, 'y2': y2
                }
                self.detected_objects.append(obj)
        else:
            self.person_detected = False

    def publish_debug_image(self):
        if self.latest_image is None: return
        
        # === 新增：超时检查 ===
        # 如果当前时间 - 上次收到box的时间 > 0.1秒
        # 说明 box 数据断流了（也就是没人了），强制清空框
        if (rospy.Time.now() - self.last_box_time).to_sec() > 0.1:
            self.detected_objects = []
            self.person_detected = False
        # ====================

        # 在当前帧的拷贝上画图
        debug_img = self.latest_image.copy()
        
        # 如果当前没有检测到人，detected_objects 是空的，画面上自然就没有框
        # 这就解决了“鬼影”问题
        
        for obj in self.detected_objects:
            x1, y1, x2, y2 = obj['x1'], obj['y1'], obj['x2'], obj['y2']
            
            # === 关键修复 4: 不同目标不同颜色 ===
            color = self.get_color(obj['id'])
            if obj['is_locked']: 
                color = (0, 0, 255) # 锁定的强制红色
            
            # 画矩形
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
            
            # 写标签
            label = "ID: {}".format(obj['id'])
            # 加上背景色让字看清楚
            cv2.rectangle(debug_img, (x1, y1-20), (x1+60, y1), color, -1)
            cv2.putText(debug_img, label, (x1+5, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        try:
            # 这里的 bridge 可能会报错，如果报错请用下面的手动打包
            ros_msg = self.bridge.cv2_to_imgmsg(debug_img, "bgr8")
            self.debug_pub.publish(ros_msg)
        except Exception:
            # 备用手动打包
            pass


    def save_evidence(self):
        """保存带框的证据图片"""
        if self.latest_image is None:
            return 

        try:
            home_path = os.path.expanduser("~") 
            save_dir = os.path.join(home_path, "patrol_evidences")
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)

            timestamp = rospy.Time.now().to_nsec()
            
            # 构建文件名：如果有多个入侵者，列出他们的ID
            # 例如: evidence_Intruder1_Intruder2_123456789.jpg
            names = []
            for idx, obj in enumerate(self.detected_objects):
                tid = obj['id']
                names.append("Intruder{}".format(tid if tid != -1 else idx + 1))
            
            name_str = "_".join(names)
            if not name_str: name_str = "Unknown"
            
            filename = os.path.join(save_dir, "evidence_{}_{}.jpg".format(name_str, timestamp))
            
            # 我们保存那个画了框的 debug 图片，更有说服力！
            # 为了省事，这里重画一遍（或者你也可以把 debug_img 存成成员变量）
            evidence_img = self.latest_image.copy()
            for idx, obj in enumerate(self.detected_objects):
                x, y, w, h = obj['x'], obj['y'], obj['w'], obj['h']
                track_id = obj['id']
                label_text = "Intruder_{}".format(track_id if track_id != -1 else idx + 1)
                cv2.rectangle(evidence_img, (x, y), (x+w, y+h), (0, 0, 255), 2)
                cv2.putText(evidence_img, label_text, (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            cv2.imwrite(filename, evidence_img)
            rospy.loginfo("【SUCCESS】已保存入侵者证据: %s", filename)
                
        except Exception as e:
            rospy.logerr("保存失败: %s", e)