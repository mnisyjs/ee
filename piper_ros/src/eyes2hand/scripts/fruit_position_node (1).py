#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32, Float32
from cv_bridge import CvBridge
import numpy as np

class FruitPositionNode:
    def __init__(self):
        rospy.init_node('fruit_position_node', anonymous=True)

        self.bridge = CvBridge()
        self.latest_depth = None
        self.camera_fx = 615.0  # 需要根据相机参数修改
        self.camera_fy = 615.0
        self.camera_cx = 320.0
        self.camera_cy = 240.0

        self.pose_pub = rospy.Publisher('/apple_pose', PoseStamped, queue_size=10)
        self.id_pub = rospy.Publisher('/apple_id', Int32, queue_size=10)
        self.conf_pub = rospy.Publisher('/apple_confidence', Float32, queue_size=10)

        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/camera/detections',  # 订阅 YOLO 检测
                         rospy.msg.AnyMsg, 
                         self.detection_callback)

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def detection_callback(self, msg):
        if self.latest_depth is None:
            return

        detections = json.loads(msg._buff.decode())  # YOLO输出JSON
        for idx, det in enumerate(detections):
            x1, y1, x2, y2 = det['bbox']
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            depth = self.latest_depth[cy, cx] / 1000.0  # 转成米
            X = (cx - self.camera_cx) * depth / self.camera_fx
            Y = (cy - self.camera_cy) * depth / self.camera_fy
            Z = depth

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "camera_link"
            pose_msg.pose.position.x = X
            pose_msg.pose.position.y = Y
            pose_msg.pose.position.z = Z

            self.pose_pub.publish(pose_msg)
            self.id_pub.publish(idx)
            self.conf_pub.publish(det['confidence'])

if __name__ == '__main__':
    try:
        FruitPositionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
