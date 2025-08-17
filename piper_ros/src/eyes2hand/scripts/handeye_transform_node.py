#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from arm_control_node.msg import CameraTargets
from eyes2hand.msg import HandEyeIK
import numpy as np
import moveit_commander
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
import rospkg
import os
from urdf_parser_py.urdf import URDF

class HandEyeTransformNode(object):
    def __init__(self):
        rospy.init_node("handeye_transform_node", anonymous=True)

        # 1. 加载URDF文件，初始化逆解求解器
        rospack = rospkg.RosPack()
        urdf_pkg_path = rospack.get_path('armmu')
        urdf_path = os.path.join(urdf_pkg_path, 'urdf/armmu.urdf')
        self.robot_urdf = URDF.from_xml_file(urdf_path)
        rospy.loginfo("Loaded URDF from: {}".format(urdf_path))

        # 初始化MoveIt!
        moveit_commander.roscpp_initialize([])
        self.arm_group = moveit_commander.MoveGroupCommander("piper")

        # 2. 订阅相机目标位姿
        camera_topic = rospy.get_param('~camera_pose_topic', '/camera/targets')
        rospy.Subscriber(camera_topic, CameraTargets, self.camera_callback)

        # 3. 发布手眼逆解结果
        self.ik_pub = rospy.Publisher('/handeye/ik_result', HandEyeIK, queue_size=10)

        # 手眼标定矩阵可参数化
        self.T_cam2ee = rospy.get_param('~handeye_matrix', [1,0,0,-0.15,0,1,0,0,0,0,1,-0.05,0,0,0,1])
        self.T_cam2ee = np.array(self.T_cam2ee).reshape((4,4))

        self.confidence_threshold = rospy.get_param('~red_confidence_threshold', 0.7)
        rospy.loginfo("Hand-eye transform node is ready.")

    def camera_callback(self, msg):
        header = msg.header
        rospy.loginfo("Received CameraTargets msg: type=%s, red_conf=%.2f, green_conf=%.2f",
                      msg.target_type, msg.red_confidence, msg.green_confidence)

        # 只处理红色果实
        if msg.target_type not in ["red_apple", "both"]:
            rospy.loginfo("Target type is %s, not a red apple. Skipping.", msg.target_type)
            return

        if msg.red_confidence < self.confidence_threshold:
            rospy.loginfo("Red apple confidence %.2f below threshold %.2f. Skipping.",
                          msg.red_confidence, self.confidence_threshold)
            return

        # 目标点
        target_pose = msg.red_apple_pose.pose
        rospy.loginfo("Processing red apple target: xyz=(%.3f, %.3f, %.3f)",
                      target_pose.position.x, target_pose.position.y, target_pose.position.z)

        # 手眼变换：相机坐标系 -> 末端执行器坐标系
        T_cam2ee = self.T_cam2ee
        T_base2obj = self.pose_to_matrix(target_pose)
        T_base2ee = np.dot(T_base2obj, np.linalg.inv(T_cam2ee))

        # 逆解关节角度
        target_pose_ee = self.matrix_to_pose(T_base2ee)
        joint_angles = self.solve_ik(target_pose_ee)

        # 发布消息
        ik_msg = HandEyeIK()
        if joint_angles is not None and len(joint_angles) >= 6:
            ik_msg.joint_angles = [float(a) for a in joint_angles[:6]]
            rospy.loginfo("IK solution: %s", ["%.3f" % a for a in ik_msg.joint_angles])
        else:
            ik_msg.joint_angles = []
            rospy.logwarn("No valid IK solution found for red apple pose!")

        ik_msg.matrix = T_base2ee.flatten().tolist()
        ik_msg.reference_frame = header.frame_id if header.frame_id else "base_link"

        self.ik_pub.publish(ik_msg)
        rospy.loginfo("Published IK result for red apple.")

    def pose_to_matrix(self, pose):
        import tf
        trans = tf.transformations.translation_matrix([pose.position.x, pose.position.y, pose.position.z])
        rot = tf.transformations.quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        return np.dot(trans, rot)

    def matrix_to_pose(self, mat):
        import tf
        from geometry_msgs.msg import Pose
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = tf.transformations.translation_from_matrix(mat)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = tf.transformations.quaternion_from_matrix(mat)
        return pose

    def solve_ik(self, pose):
        try:
            self.arm_group.set_pose_target(pose)
            plan = self.arm_group.plan()
            if plan and plan.joint_trajectory.points:
                return plan.joint_trajectory.points[-1].positions
            else:
                rospy.logwarn("IK failed: No trajectory points.")
                return None
        except Exception as e:
            rospy.logerr("IK Exception: {}".format(repr(e)))
            return None

if __name__ == '__main__':
    try:
        node = HandEyeTransformNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass