#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped

class FruitPositionNode:
    def __init__(self):
        rospy.init_node('fruit_position_node', anonymous=True)

        # 创建一个发布器，发布目标位置信息
        self.pose_pub = rospy.Publisher('/camera/targets', PoseStamped, queue_size=10)

        # 设置一个固定的果实位置
        self.fixed_x = 1.0  # 固定位置X
        self.fixed_y = 0.5  # 固定位置Y
        self.fixed_z = 0.3  # 固定位置Z

        # 定时器以定期发布固定位置
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_target)

    def publish_target(self, event):
        # 创建 PoseStamped 消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "camera_link"  # 可以根据需要调整坐标系

        # 设置固定的三维位置
        pose_msg.pose.position.x = self.fixed_x
        pose_msg.pose.position.y = self.fixed_y
        pose_msg.pose.position.z = self.fixed_z

        # 发布消息
        self.pose_pub.publish(pose_msg)

if __name__ == '__main__':
    try:
        FruitPositionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
