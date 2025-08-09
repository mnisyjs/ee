#!/usr/bin/env python

import rospy
import math
import random
import sys
import os

# 添加消息路径
sys.path.append(os.path.join(os.path.dirname(__file__), '..', '..', '..'))

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
try:
    from arm_control_node.msg import CameraTargets
except ImportError:
    # 如果无法导入新消息，回退到旧的方式
    rospy.logwarn("Could not import CameraTargets message, using PoseStamped instead")
    CameraTargets = None

class CameraSimulator:
    def __init__(self):
        rospy.init_node('camera_simulator')
        
        # 发布目标点位（苹果+果树）
        if CameraTargets is not None:
            self.target_pub = rospy.Publisher('/camera/targets', CameraTargets, queue_size=10)
            self.use_new_msg = True
            rospy.loginfo("Using new CameraTargets message format")
        else:
            self.target_pub = rospy.Publisher('/camera/target_pose', PoseStamped, queue_size=10)
            self.use_new_msg = False
            rospy.loginfo("Using legacy PoseStamped message format")
        
        # 模拟参数
        self.target_count = 0
        self.max_targets = 5  # 最大目标数量
        
        # 苹果和果树位置列表（示例坐标：苹果位置, 果树位置）
        self.target_data = [
            {"apple": (2.0, 1.5, 0.8), "tree": (1.8, 1.3, 0.0)},   # 目标1
            {"apple": (3.0, 2.0, 0.9), "tree": (2.7, 1.8, 0.0)},   # 目标2
            {"apple": (1.5, 3.0, 0.7), "tree": (1.2, 2.8, 0.0)},   # 目标3
            {"apple": (4.0, 1.0, 0.8), "tree": (3.8, 0.7, 0.0)},   # 目标4
            {"apple": (2.5, 2.5, 0.9), "tree": (2.2, 2.3, 0.0)},   # 目标5
        ]
        
        # 定时器，每10秒发布一个目标
        self.timer = rospy.Timer(rospy.Duration(10.0), self.publish_target)
        
        rospy.loginfo("Camera simulator started. Will publish targets every 10 seconds.")
    
    def publish_target(self, event):
        if self.target_count >= self.max_targets:
            rospy.loginfo("All targets published. Stopping simulator.")
            self.timer.shutdown()
            return
        
        # 获取苹果和果树数据
        data = self.target_data[self.target_count]
        apple_pos = data["apple"]
        tree_pos = data["tree"]
        
        if self.use_new_msg and CameraTargets is not None:
            # 使用新的CameraTargets消息
            target_msg = CameraTargets()
            target_msg.header = Header()
            target_msg.header.frame_id = "map"
            target_msg.header.stamp = rospy.Time.now()
            
            # 苹果位置
            target_msg.apple_pose = PoseStamped()
            target_msg.apple_pose.header = target_msg.header
            
            # 添加一些随机噪声，模拟真实相机检测
            apple_x = apple_pos[0] + random.uniform(-0.1, 0.1)
            apple_y = apple_pos[1] + random.uniform(-0.1, 0.1)
            apple_z = apple_pos[2] + random.uniform(-0.05, 0.05)
            
            target_msg.apple_pose.pose.position.x = apple_x
            target_msg.apple_pose.pose.position.y = apple_y
            target_msg.apple_pose.pose.position.z = apple_z
            target_msg.apple_pose.pose.orientation.w = 1.0
            
            # 果树位置
            target_msg.tree_pose = PoseStamped()
            target_msg.tree_pose.header = target_msg.header
            
            tree_x = tree_pos[0] + random.uniform(-0.05, 0.05)
            tree_y = tree_pos[1] + random.uniform(-0.05, 0.05)
            tree_z = tree_pos[2]
            
            target_msg.tree_pose.pose.position.x = tree_x
            target_msg.tree_pose.pose.position.y = tree_y
            target_msg.tree_pose.pose.position.z = tree_z
            target_msg.tree_pose.pose.orientation.w = 1.0
            
            # 其他信息
            target_msg.target_type = "apple"
            target_msg.apple_id = self.target_count + 1
            target_msg.tree_id = self.target_count + 1
            target_msg.confidence = random.uniform(0.8, 0.95)
            
            # 发布目标
            self.target_pub.publish(target_msg)
            
            rospy.loginfo("Published targets %d: apple(%.2f,%.2f,%.2f), tree(%.2f,%.2f,%.2f)", 
                         self.target_count + 1, apple_x, apple_y, apple_z, tree_x, tree_y, tree_z)
        else:
            # 回退到旧的PoseStamped消息（只发布苹果位置）
            target = PoseStamped()
            target.header = Header()
            target.header.frame_id = "map"
            target.header.stamp = rospy.Time.now()
            
            apple_x = apple_pos[0] + random.uniform(-0.1, 0.1)
            apple_y = apple_pos[1] + random.uniform(-0.1, 0.1)
            apple_z = apple_pos[2] + random.uniform(-0.05, 0.05)
            
            target.pose.position.x = apple_x
            target.pose.position.y = apple_y
            target.pose.position.z = apple_z
            target.pose.orientation.w = 1.0
            
            # 发布目标
            self.target_pub.publish(target)
            
            rospy.loginfo("Published target %d: x=%.2f, y=%.2f, z=%.2f", 
                         self.target_count + 1, apple_x, apple_y, apple_z)
        
        self.target_count += 1
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        simulator = CameraSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass 