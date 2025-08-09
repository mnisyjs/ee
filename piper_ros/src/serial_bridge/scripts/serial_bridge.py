#!/usr/bin/env python3
import rospy
import serial
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class SerialBridge:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
        rospy.init_node('serial_bridge')
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.pub = rospy.Publisher('/stm32_feedback', String, queue_size=10)

    def cmd_vel_callback(self, msg):
        # 假设通信协议为: "v:线速度,w:角速度\n"
        data = "v:{:.2f},w:{:.2f}\n".format(msg.linear.x, msg.angular.z)
        self.ser.write(data.encode())

    def spin(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.ser.in_waiting:
                stm32_msg = self.ser.readline().decode().strip()
                self.pub.publish(stm32_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SerialBridge()
        node.spin()
    except rospy.ROSInterruptException:
        pass
