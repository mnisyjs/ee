#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from arm_control_node.msg import CameraTargets  # 你的自定义消息类型路径
from std_msgs.msg import Header

class FruitPositionNode:
    def __init__(self):
        rospy.init_node('fruit_position_node', anonymous=True)

        # 创建一个发布器，发布目标位置信息
        self.pose_pub = rospy.Publisher('/camera/targets', CameraTargets, queue_size=10)

        # 设置固定的果实位置
        self.red_fruit_x = 0.5  # 红色果实 X 位置
        self.red_fruit_y = 0.5  # 红色果实 Y 位置
        self.red_fruit_z = 0.5  # 红色果实 Z 位置

        self.green_fruit_x = 1.0  # 绿色果实 X 位置
        self.green_fruit_y = 0.5  # 绿色果实 Y 位置
        self.green_fruit_z = 0.3  # 绿色果实 Z 位置

        self.red_apple_id = 1   # 红色果实 ID
        self.green_apple_id = 2  # 绿色果实 ID

        self.red_confidence = 0.95  # 红色果实置信度
        self.green_confidence = 0.92  # 绿色果实置信度

        self.confidence = 0.93  # 总体置信度

        # 定时器定期发布固定位置
        self.timer = rospy.Timer(rospy.Duration(1), self.publish_target)

    def publish_target(self, event):
        # 创建 CameraTargets 消息
        target_msg = CameraTargets()

        # 设置消息头
        target_msg.header = Header()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.header.frame_id = "camera_link"  # 选择适当的坐标系

        # 设置红色果实目标
        red_target = PoseStamped()
        red_target.header.stamp = rospy.Time.now()
        red_target.header.frame_id = "camera_link"
        red_target.pose.position.x = self.red_fruit_x
        red_target.pose.position.y = self.red_fruit_y
        red_target.pose.position.z = self.red_fruit_z

        # 设置绿色果实目标
        green_target = PoseStamped()
        green_target.header.stamp = rospy.Time.now()
        green_target.header.frame_id = "camera_link"
        green_target.pose.position.x = self.green_fruit_x
        green_target.pose.position.y = self.green_fruit_y
        green_target.pose.position.z = self.green_fruit_z

        # 设置消息字段
        target_msg.red_apple_pose = red_target  # 红色果实位置
        target_msg.green_apple_pose = green_target  # 绿色果实位置

        # 设置目标类型（例如 "both" 表示同时存在红色和绿色果实）
        target_msg.target_type = "both"

        # 设置果实 ID
        target_msg.red_apple_id = self.red_apple_id
        target_msg.green_apple_id = self.green_apple_id

        # 设置果实置信度
        target_msg.red_confidence = self.red_confidence
        target_msg.green_confidence = self.green_confidence

        # 设置整体置信度
        target_msg.confidence = self.confidence

        # 发布消息
        self.pose_pub.publish(target_msg)

if __name__ == '__main__':
    try:
        FruitPositionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
