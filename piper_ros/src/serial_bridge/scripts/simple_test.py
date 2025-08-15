#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简化版串口通信测试脚本
用于测试ROS和STM32之间的串口通信是否正常工作（无PID版本）
"""

import rospy
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class SimpleSerialTest:
    def __init__(self):
        rospy.init_node('simple_serial_test', anonymous=True)
        
        # 发布者
        self.pub_target_pose = rospy.Publisher('/chassis/target_pose', PoseStamped, queue_size=10)
        self.pub_emergency_stop = rospy.Publisher('/chassis/emergency_stop', Bool, queue_size=10)
        
        # 等待发布者初始化
        rospy.sleep(1.0)
        
        rospy.loginfo("Simple Serial Test initialized")
        
    def send_simple_movement(self, x, y, yaw=0.0):
        """发送简单的运动指令"""
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "map"
        
        target_pose.pose.position.x = x
        target_pose.pose.position.y = y
        target_pose.pose.position.z = 0.0
        
        # 设置方向（yaw角转四元数）
        import math
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        
        # 发布目标位置
        self.pub_target_pose.publish(target_pose)
        rospy.loginfo(f"Sent movement command: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
        
    def test_emergency_stop(self):
        """测试紧急停止"""
        rospy.loginfo("Testing emergency stop...")
        
        # 激活紧急停止
        emergency_msg = Bool()
        emergency_msg.data = True
        self.pub_emergency_stop.publish(emergency_msg)
        
        rospy.sleep(2.0)
        
        # 取消紧急停止
        emergency_msg.data = False
        self.pub_emergency_stop.publish(emergency_msg)
        
        rospy.loginfo("Emergency stop test completed")
        
    def run_simple_test(self):
        """运行简单测试"""
        rospy.loginfo("Starting simple serial communication test...")
        
        # 等待ROS系统稳定
        rospy.sleep(2.0)
        
        try:
            # 测试1：向前移动
            rospy.loginfo("Test 1: Move forward")
            self.send_simple_movement(1.0, 0.0, 0.0)
            rospy.sleep(3.0)
            
            # 测试2：向右移动
            rospy.loginfo("Test 2: Move right")
            self.send_simple_movement(1.0, 1.0, 0.0)
            rospy.sleep(3.0)
            
            # 测试3：旋转
            rospy.loginfo("Test 3: Rotate")
            self.send_simple_movement(1.0, 1.0, 1.57)  # 90度
            rospy.sleep(3.0)
            
            # 测试4：回到原点
            rospy.loginfo("Test 4: Return to origin")
            self.send_simple_movement(0.0, 0.0, 0.0)
            rospy.sleep(3.0)
            
            # 测试5：紧急停止
            rospy.loginfo("Test 5: Emergency stop")
            self.test_emergency_stop()
            
            rospy.loginfo("All tests completed successfully!")
            
        except KeyboardInterrupt:
            rospy.loginfo("Test interrupted by user")
        except Exception as e:
            rospy.logerr(f"Test error: {e}")
            
    def run_continuous_test(self):
        """运行连续测试"""
        rospy.loginfo("Starting continuous test (press Ctrl+C to stop)...")
        
        rate = rospy.Rate(0.5)  # 每2秒发送一次指令
        test_count = 0
        
        try:
            while not rospy.is_shutdown():
                test_count += 1
                
                # 发送圆形运动指令
                import math
                t = time.time()
                x = 0.5 * math.cos(t * 0.5)
                y = 0.5 * math.sin(t * 0.5)
                yaw = t * 0.25
                
                self.send_simple_movement(x, y, yaw)
                
                # 每10次测试执行一次紧急停止测试
                if test_count % 10 == 0:
                    self.test_emergency_stop()
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Continuous test stopped by user")
        except Exception as e:
            rospy.logerr(f"Continuous test error: {e}")

if __name__ == '__main__':
    try:
        test = SimpleSerialTest()
        
        # 选择测试模式
        import sys
        if len(sys.argv) > 1 and sys.argv[1] == "continuous":
            test.run_continuous_test()
        else:
            test.run_simple_test()
            
    except rospy.ROSInterruptException:
        pass
