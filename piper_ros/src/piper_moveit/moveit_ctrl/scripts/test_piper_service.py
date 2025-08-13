#!/usr/bin/env python

import rospy
import sys
from moveit_ctrl.srv import JointMoveitCtrl

def test_piper_service():
    """测试Piper MoveIt控制服务"""
    rospy.init_node('test_piper_service')
    
    # 等待服务可用
    rospy.loginfo("Waiting for /joint_moveit_ctrl_piper service...")
    try:
        rospy.wait_for_service('/joint_moveit_ctrl_piper', timeout=10.0)
        rospy.loginfo("Service is available!")
    except rospy.ROSException:
        rospy.logerr("Service /joint_moveit_ctrl_piper not available!")
        return False
    
    # 创建服务代理
    try:
        piper_service = rospy.ServiceProxy('/joint_moveit_ctrl_piper', JointMoveitCtrl)
        rospy.loginfo("Service proxy created successfully!")
    except Exception as e:
        rospy.logerr(f"Failed to create service proxy: {e}")
        return False
    
    # 测试服务调用
    try:
        # 设置测试关节角度（安全位置）
        joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6个关节
        gripper = 0.0  # 夹爪位置
        max_velocity = 0.1  # 低速
        max_acceleration = 0.1  # 低加速度
        
        rospy.loginfo("Testing service call with safe joint angles...")
        response = piper_service(joint_states, gripper, max_velocity, max_acceleration)
        
        if response.status:
            rospy.loginfo("Service call successful!")
            return True
        else:
            rospy.logerr(f"Service call failed with error code: {response.error_code}")
            return False
            
    except Exception as e:
        rospy.logerr(f"Exception during service call: {e}")
        return False

if __name__ == '__main__':
    try:
        success = test_piper_service()
        if success:
            rospy.loginfo("Test completed successfully!")
            sys.exit(0)
        else:
            rospy.logerr("Test failed!")
            sys.exit(1)
    except rospy.ROSInterruptException:
        rospy.loginfo("Test interrupted!")
        sys.exit(1)
