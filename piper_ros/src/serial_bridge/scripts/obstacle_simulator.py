#!/usr/bin/env python

import rospy
import random
from std_msgs.msg import Bool

class ObstacleSimulator:
    def __init__(self):
        rospy.init_node('obstacle_simulator')
        
        # 发布障碍检测结果
        self.obstacle_pub = rospy.Publisher('/camera_obstacle', Bool, queue_size=10)
        
        # 模拟参数
        self.obstacle_probability = 0.1  # 10%的概率检测到障碍
        
        # 定时器，每秒检查一次
        self.timer = rospy.Timer(rospy.Duration(1.0), self.check_obstacle)
        
        rospy.loginfo("Obstacle simulator started.")
    
    def check_obstacle(self, event):
        # 模拟障碍检测
        has_obstacle = random.random() < self.obstacle_probability
        
        # 发布障碍检测结果
        obstacle_msg = Bool()
        obstacle_msg.data = has_obstacle
        self.obstacle_pub.publish(obstacle_msg)
        
        if has_obstacle:
            rospy.logwarn("Obstacle detected!")
        else:
            rospy.logdebug("No obstacle detected.")
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        simulator = ObstacleSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass 