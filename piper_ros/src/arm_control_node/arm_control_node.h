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
#include <vector>
#include <queue>
#include <map>

// 红果信息结构体
struct RedAppleInfo {
    geometry_msgs::PoseStamped pose;
    int id;
    float confidence;
    bool is_safe;
    ros::Time detection_time;
    
    RedAppleInfo() : id(0), confidence(0.0), is_safe(false) {}
};

class ArmControlNode {
public:
    ArmControlNode(ros::NodeHandle& nh);

    void cameraTargetCallback(const arm_control_node::CameraTargets::ConstPtr& msg);
    void handeyeCallback(const eyes2hand::HandEyeIK::ConstPtr& msg);
    void chassisArrivalCallback(const std_msgs::Bool::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    void run();

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_camera_target_;    // 订阅相机目标点位
    ros::Subscriber sub_handeye_;          // 订阅手眼变换结果
    ros::Subscriber sub_chassis_arrival_;  // 订阅小车到位状态
    ros::Subscriber sub_odom_;             // 订阅里程计
    
    ros::Publisher pub_status_;            // 发布任务状态
    ros::Publisher pub_chassis_target_;    // 发布小车目标位置

    
    ros::ServiceClient piper_client_;      // piper规划组服务客户端
    ros::ServiceClient gripper_client_;    // 夹爪控制服务客户端
    
    std::vector<double> place_fruit_joint_angles_;     // 放果位置角度
    std::vector<double> grab_basket_joint_angles_;     // 抓取果篮位置角度
    std::vector<double> dump_fruit_joint_angles_;      // 倾倒果子位置角度
    std::vector<double> basket_return_angles_;   // 果篮定位角度
    
    arm_control_node::CameraTargets current_camera_targets_;  // 当前相机目标（红果+绿果）
    eyes2hand::HandEyeIK last_handeye_ik_;
    geometry_msgs::PoseStamped current_chassis_pose_;  // 当前小车位置
    geometry_msgs::PoseStamped storage_pose_;          // 果子存储区位姿
    geometry_msgs::PoseStamped return_pose_;           // 倒果前的位置（用于返回）
    
    // 果树位置信息（用于安全避障）
    std::vector<geometry_msgs::PoseStamped> tree_positions_;
    
    // 新增：红果队列管理
    std::queue<RedAppleInfo> red_apple_queue_;        // 待采摘的红果队列
    std::map<int, RedAppleInfo> detected_red_apples_; // 已检测到的红果记录
    int next_red_apple_id_;                           // 下一个红果ID
    
    // 新增：剪刀末端执行器变换矩阵
    // 变换矩阵格式：[R, t; 0, 1] 其中 R=eye(3), t=(0.01, 0.02, 0.11)
    double scissor_transform_[4][4];                  // 4x4变换矩阵
    double scissor_position_tolerance_[3];            // 位置容差范围 [x_tol, y_tol, z_tol]
    
    // 控制参数
    double position_tolerance_;      // 位置容差
    double orientation_tolerance_;   // 角度容差
    double optimal_distance_;        // 最佳操作距离
    int basket_count_;               // 已放果次数统计
    int max_basket_count_;           // 达到阈值后去倒果
    
    // 状态变量
    bool waiting_for_chassis_;       // 是否等待小车到位
    bool chassis_arrived_;           // 小车是否已到位

    // 状态机变量
    enum State {
        IDLE, EVALUATE_POSITION, MOVE_CHASSIS, WAIT_FOR_CHASSIS, 
        EXECUTE_ARM_OPERATION, MOVE_TO_FRUIT, CUT_FRUIT, RETRACT, 
        MOVE_TO_BASKET, PLACE_FRUIT, RECOVER,
        MOVE_TO_STORAGE, WAIT_FOR_STORAGE, DUMP_FRUITS, RETURN_TO_WORKSITE, WAIT_FOR_RETURN
    };
    State current_state_;

    void moveToJointAngles(const std::vector<double>& joint_angles);
    static std::string stateToString(State state);
    void operateGripper(const std::string& action);
    void checkAndPlanPath(const std::vector<double>& from, const std::vector<double>& to);
    void publishStatus(const std::string& status);
    
    // 核心评估和规划方法
    bool evaluatePositionForRedApple(const arm_control_node::CameraTargets::ConstPtr& targets);
    void planChassisMovementForRedApple(const arm_control_node::CameraTargets::ConstPtr& targets);
    double calculateSafetyScore(const arm_control_node::CameraTargets::ConstPtr& targets,
        const geometry_msgs::PoseStamped& chassis_pos);

    // 安全检查方法（简化版，只检查果树）
    bool checkSafetyForRedApple(const arm_control_node::CameraTargets::ConstPtr& targets, 
                                const geometry_msgs::PoseStamped& chassis_pos);
    
    // 新增：红果管理方法
    void addRedAppleToQueue(const arm_control_node::CameraTargets::ConstPtr& targets);
    void processRedAppleQueue();
    bool isRedAppleSafe(const RedAppleInfo& red_apple);
    void applyScissorTransform(geometry_msgs::PoseStamped& target_pose);
    
    // 果树位置管理
    void loadTreePositions();
    
    // 机械臂操作
    void executeArmOperation();
    
    // 示教角度管理
    void loadTeachAngles();
    
    // 倒果相关
    void startDumpIfNeeded();
    void publishChassisTarget(const geometry_msgs::PoseStamped& pose);
    void performDumpFruits();
    void teachBasketCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
    
    // 新增：初始化剪刀变换矩阵
    void initializeScissorTransform();
};