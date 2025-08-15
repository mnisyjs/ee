#!/usr/bin/env python3
import rospy
import serial
import struct
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SerialBridge:
    def __init__(self):
        port = rospy.get_param('~port', '/dev/ttyUSB0')
        baud = rospy.get_param('~baud', 115200)
        self.ser = serial.Serial(port, baud, timeout=0.1)
        rospy.init_node('serial_bridge')
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.pub = rospy.Publisher('/stm32_feedback', String, queue_size=10)
        rospy.loginfo(f"SerialBridge started on {port}@{baud}")

    def cmd_vel_callback(self, msg):
        # 限制速度范围在 [-1.0, 1.0]
        vx = max(min(msg.linear.x, 1.0), -1.0)
        vy = max(min(msg.linear.y, 1.0), -1.0)
        w = max(min(msg.angular.z, 1.0), -1.0)
        
        # 转换为STM32协议要求的字节格式
        vx_byte = int(128 - vx * 128)
        vy_byte = int(128 - vy * 128)
        w_byte = int(128 - w * 128)
        
        # 构建8字节数据包
        packet = bytes([
            0x3F, 0x21, 0x01,  # 固定包头
            vx_byte,           # vx
            vy_byte,           # vy
            w_byte,            # w
            0x00,              # 未使用字节
            0x21               # 固定包尾
        ])
        
        # 发送二进制数据
        self.ser.write(packet)
        rospy.loginfo(f"Sent: {packet.hex(' ')} | vx={vx:.2f}, vy={vy:.2f}, w={w:.2f}")

    def spin(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.ser.in_waiting:
                try:
                    raw = self.ser.read(self.ser.in_waiting)
                    self.pub.publish(raw.hex())
                except Exception as e:
                    rospy.logerr(f"Serial read error: {str(e)}")
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SerialBridge()
        node.spin()
    except rospy.ROSInterruptException:
        pass