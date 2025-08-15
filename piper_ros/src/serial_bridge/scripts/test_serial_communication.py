#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口通信测试脚本
用于测试ROS和STM32之间的串口通信是否正常工作
"""

import rospy
import serial
import struct
import time
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

class SerialCommunicationTest:
    def __init__(self):
        rospy.init_node('serial_communication_test', anonymous=True)
        
        # 串口参数
        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.serial_conn = None
        
        # 发布者
        self.pub_target_pose = rospy.Publisher('/chassis/target_pose', PoseStamped, queue_size=10)
        self.pub_emergency_stop = rospy.Publisher('/chassis/emergency_stop', Bool, queue_size=10)
        
        # 测试参数
        self.test_interval = rospy.get_param('~test_interval', 5.0)  # 测试间隔
        
        rospy.loginfo(f"Serial Communication Test initialized on port {self.port}")
        
    def connect_serial(self):
        """连接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0
            )
            rospy.loginfo(f"Successfully connected to {self.port}")
            return True
        except Exception as e:
            rospy.logerr(f"Failed to connect to {self.port}: {e}")
            return False
    
    def disconnect_serial(self):
        """断开串口连接"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            rospy.loginfo("Serial connection closed")
    
    def send_test_command(self):
        """发送测试运动指令"""
        # 创建测试目标位置
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "map"
        
        # 设置目标位置（简单的圆形运动）
        import math
        t = time.time()
        target_pose.pose.position.x = 0.5 * math.cos(t * 0.5)
        target_pose.pose.position.y = 0.5 * math.sin(t * 0.5)
        target_pose.pose.position.z = 0.0
        
        # 设置目标方向
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = math.sin(t * 0.25)
        target_pose.pose.orientation.w = math.cos(t * 0.25)
        
        # 发布目标位置
        self.pub_target_pose.publish(target_pose)
        rospy.loginfo(f"Sent test target pose: x={target_pose.pose.position.x:.2f}, y={target_pose.pose.position.y:.2f}")
    
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
    
    def read_stm32_data(self):
        """读取STM32发送的数据"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        
        try:
            # 尝试读取STM32发送的数据
            if self.serial_conn.in_waiting > 0:
                data = self.serial_conn.read(self.serial_conn.in_waiting)
                if data:
                    rospy.loginfo(f"Received from STM32: {data.hex()}")
                    
                    # 检查是否是STM32格式数据
                    if data.startswith(b'STM:'):
                        rospy.loginfo("Detected STM32 data format")
                        # 这里可以添加数据解析逻辑
                    
        except Exception as e:
            rospy.logerr(f"Error reading serial data: {e}")
    
    def run_test(self):
        """运行测试"""
        rospy.loginfo("Starting serial communication test...")
        
        # 连接串口
        if not self.connect_serial():
            rospy.logerr("Failed to connect serial port, exiting...")
            return
        
        rate = rospy.Rate(1.0 / self.test_interval)
        test_count = 0
        
        try:
            while not rospy.is_shutdown():
                test_count += 1
                rospy.loginfo(f"Test iteration {test_count}")
                
                # 发送测试指令
                self.send_test_command()
                
                # 读取STM32数据
                self.read_stm32_data()
                
                # 每10次测试执行一次紧急停止测试
                if test_count % 10 == 0:
                    self.test_emergency_stop()
                
                rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Test interrupted by user")
        except Exception as e:
            rospy.logerr(f"Test error: {e}")
        finally:
            self.disconnect_serial()
            rospy.loginfo("Test completed")

if __name__ == '__main__':
    try:
        test = SerialCommunicationTest()
        test.run_test()
    except rospy.ROSInterruptException:
        pass
