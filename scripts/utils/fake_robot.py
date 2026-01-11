#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
import math

class FakeRobot:
    def __init__(self):
        rospy.init_node('fake_robot_simulator')
        
        # 初始位置
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        
        self.vx = 0.0
        self.vth = 0.0
        
        self.last_time = rospy.Time.now()
        self.br = tf.TransformBroadcaster()
        
        # 订阅
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.set_pose)
        rospy.Subscriber('/cmd_vel', Twist, self.vel_cb)
        
        # 发布里程计话题
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    def set_pose(self, msg):
        # 响应 Rviz 的 2D Pose Estimate
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # 四元数转欧拉角
        q = msg.pose.pose.orientation
        (r, p, y) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.th = y
        rospy.loginfo("重置机器人位置: ({:.2f}, {:.2f})".format(self.x, self.y))

    def vel_cb(self, msg):
        self.vx = msg.linear.x
        self.vth = msg.angular.z

    def update(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()
        self.last_time = current_time
        
        # 1. 计算运动 (积分)
        delta_x = (self.vx * math.cos(self.th)) * dt
        delta_y = (self.vx * math.sin(self.th)) * dt
        delta_th = self.vth * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th
        
        # 构建四元数
        quat = tf.transformations.quaternion_from_euler(0, 0, self.th)
        
        # 2. 发布 TF: map -> odom
        # 在纯仿真中，我们假设定位是完美的，所以 map 和 odom 重合
        self.br.sendTransform((0, 0, 0), (0, 0, 0, 1),
                             current_time, "odom", "map")
                             
        # 3. 发布 TF: odom -> base_link (机器人移动)
        self.br.sendTransform((self.x, self.y, 0), quat,
                             current_time, "base_link", "odom")
                             
        # 4. 发布 TF: base_link -> rslidar (雷达安装位置)
        # 假装雷达在头顶 0.2米处
        self.br.sendTransform((0.2, 0, 0.1), (0, 0, 0, 1),
                             current_time, "rslidar", "base_link")

        # 5. 发布 /odom 话题 (为了让 move_base 收到速度反馈)
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.vth
        self.odom_pub.publish(odom)

if __name__ == '__main__':
    sim = FakeRobot()
    rate = rospy.Rate(20) # 20Hz
    while not rospy.is_shutdown():
        sim.update()
        rate.sleep()