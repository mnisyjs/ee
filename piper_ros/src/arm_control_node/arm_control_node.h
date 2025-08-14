#pragma once

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <eyes2hand/HandEyeIK.h>
#include <moveit_ctrl/JointMoveitCtrl.h>
#include <piper_msgs/Gripper.h>
#include <sensor_msgs/JointState.h>
#include <arm_control_node/CameraTargets.h>

class ArmControlNode {
public:
    ArmControlNode(ros::NodeHandle& nh);

    void cameraTargetCallback(const arm_control_node::CameraTargets::ConstPtr& msg);
    void handeyeCallback(const eyes2hand::HandEyeIK::ConstPtr& msg);
    void teachBasketCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void visionCallback(const std_msgs::Bool::ConstPtr& msg);
    void chassisArrivalCallback(const std_msgs::Bool::ConstPtr& msg);
    void chassisStatusCallback(const std_msgs::String::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void run();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_camera_target_;    // 订阅相机目标点位
    ros::Subscriber sub_handeye_;          // 订阅手眼变换结果
    ros::Subscriber sub_teach_basket_;     // 订阅果篮示教角度
    ros::Subscriber sub_vision_;           // 订阅视觉障碍检测
    ros::Subscriber sub_chassis_arrival_;  // 订阅小车到位状态
    ros::Subscriber sub_chassis_status_;   // 订阅小车状态
    ros::Subscriber sub_odom_;             // 订阅里程计
    
    ros::Publisher pub_status_;            // 发布任务状态
    ros::Publisher pub_chassis_target_;    // 发布小车目标位置
    ros::Publisher pub_emergency_stop_;    // 发布紧急停止指令
    
    ros::ServiceClient piper_client_;      // piper规划组服务客户端
    ros::ServiceClient gripper_client_;    // 夹爪控制服务客户端
    
    std::vector<double> basket_joint_angles_;
    std::vector<double> safe_joint_angles_;
    arm_control_node::CameraTargets current_camera_targets_;  // 当前相机目标（苹果+果树）
    geometry_msgs::PoseStamped current_chassis_pose_;  // 当前小车位置
    geometry_msgs::PoseStamped storage_pose_;          // 果子存储区位姿
    geometry_msgs::PoseStamped return_pose_;           // 倒果前的位置（用于返回）
    
    // 控制参数
    double position_tolerance_;      // 位置容差
    double orientation_tolerance_;   // 角度容差
    double optimal_distance_;        // 最佳操作距离
    int basket_count_;               // 已放果次数统计
    int max_basket_count_;           // 达到阈值后去倒果
    
    // 状态变量
    bool has_obstacle;
    bool waiting_for_chassis_;       // 是否等待小车到位
    bool chassis_arrived_;           // 小车是否已到位

    // 状态机变量
    enum State {
        IDLE, EVALUATE_POSITION, MOVE_CHASSIS, WAIT_FOR_CHASSIS, 
        EXECUTE_ARM_OPERATION, MOVE_TO_FRUIT, CUT_FRUIT, RETRACT, 
        MOVE_TO_BASKET, PLACE_FRUIT, RECOVER, EMERGENCY_STOP,
        MOVE_TO_STORAGE, WAIT_FOR_STORAGE, DUMP_FRUITS, RETURN_TO_WORKSITE, WAIT_FOR_RETURN
    };
    State current_state_;

    void moveToJointAngles(const std::vector<double>& joint_angles);
    static std::string stateToString(State state);
    void retrieveToSavePoint(const std::vector<double>& joint_angles);
    void operateGripper(const std::string& action);
    void checkAndPlanPath(const std::vector<double>& from, const std::vector<double>& to);
    void publishStatus(const std::string& status);
    bool detectObstacle();
    
    // 新增函数
    bool evaluatePosition(const arm_control_node::CameraTargets::ConstPtr& targets);
    void planChassisMovement(const arm_control_node::CameraTargets::ConstPtr& targets);
    void executeArmOperation();
    bool checkTreeCollisionRisk(const geometry_msgs::PoseStamped& apple_pos, 
                               const geometry_msgs::PoseStamped& tree_pos, 
                               const geometry_msgs::PoseStamped& chassis_pos);

    // 倒果相关
    void startDumpIfNeeded();
    void publishChassisTarget(const geometry_msgs::PoseStamped& pose);
    void performDumpFruits();
};