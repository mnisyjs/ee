#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from eyes2hand.msg import HandEyeIK
import numpy as np
import moveit_commander
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import rospkg
import os
# URDF解析依赖
from urdf_parser_py.urdf import URDF

class HandEyeTransformNode(object):
    def __init__(self):
        rospy.init_node("handeye_transform_node", anonymous=True)

        # 1. 加载URDF文件，初始化逆解求解器
        rospack = rospkg.RosPack()
        urdf_pkg_path = rospack.get.path('arm')
        urdf_path = os.path.join(urdf_pkg_path, 'urdf/arm.urdf')
        self.robot_urdf = URDF.from_xml_file(urdf_path)
        rospy.loginfo("Loaded URDF from: {}".format(urdf_path))

        # 初始化MoveIt!（如果使用）
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("arm") #movegroup名称

        # 2. 订阅相机目标位姿
        camera_topic = rospy.get_param('~camera_pose_topic', '/camera/target_pose')
        rospy.Subscriber(camera_topic, PoseStamped, self.camera_callback)

        # 3. 发布手眼逆解结果
        self.ik_pub = rospy.Publisher('/handeye/ik_result', HandEyeIK, queue_size=10)

        rospy.loginfo("Hand-eye transform node is ready.")

    def camera_callback(self, msg):
        rospy.loginfo("Received camera pose.")

        # 4. 手眼变换：相机坐标系 -> 末端执行器坐标系
        # 这里假设手眼外参已知，存储为4x4矩阵，或通过参数导入
        # 例如：T_cam2ee = np.array([...]).reshape(4,4)
        T_cam2ee = self.get_handeye_matrix()

        # 相机目标位姿
        T_base2obj = self.pose_to_matrix(msg.pose)

        # 末端目标位姿 = 基座到目标 * 目标到末端
        T_base2ee = np.dot(T_base2obj, np.linalg.inv(T_cam2ee))

        # 5. 逆解关节角度
        target_pose = self.matrix_to_pose(T_base2ee)
        joint_angles = self.solve_ik(target_pose)

        # 6. 发布消息
        ik_msg = HandEyeIK()
        ik_msg.joint_angles = joint_angles if joint_angles is not None else []
        ik_msg.matrix = T_base2ee.flatten().tolist()
        ik_msg.reference_frame = msg.header.frame_id

        self.ik_pub.publish(ik_msg)
        rospy.loginfo("Published IK result.")

    def get_handeye_matrix(self):
        # 示例：从参数服务器读取或硬编码手眼标定矩阵
        # 实际开发请替换为你的标定结果（4x4）
        # 例：T_cam2ee = np.eye(4)
        return np.eye(4)  # TODO: 替换为实际手眼矩阵

    def pose_to_matrix(self, pose):
        # 将geometry_msgs/Pose转换为4x4变换矩阵
        import tf
        trans = tf.transformations.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
        rot = tf.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return np.dot(trans, rot)

    def matrix_to_pose(self, mat):
        # 将4x4变换矩阵转回geometry_msgs/Pose
        import tf
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = tf.transformations.translation_from_matrix(mat)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = tf.transformations.quaternion_from_matrix(mat)
        return pose

    def solve_ik(self, pose):
        # 使用MoveIt!进行逆解，返回关节角度列表
        try:
            self.arm_group.set_pose_target(pose)
            plan = self.arm_group.plan()
            if plan and plan.joint_trajectory.points:
                return plan.joint_trajectory.points[-1].positions
                rospy.logwarn("IK failed: No valid joint solution found for given pose")
        except Exception as e:
            rospy.logerr("IK Exception: {}".format(repr(e)))
            return None

if __name__ == '__main__':
    try:
        node = HandEyeTransformNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

