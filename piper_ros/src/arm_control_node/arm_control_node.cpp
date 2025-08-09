#include "arm_control_node.h"
#include <moveit_ctrl/JointMoveitCtrl.h>
#include <piper_msgs/Gripper.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

ArmControlNode::ArmControlNode(ros::NodeHandle& nh) : nh_(nh), current_state_(IDLE), has_obstacle(false), 
                                                     waiting_for_chassis_(false), chassis_arrived_(false) {
    // 订阅相机目标点位（苹果+果树）
    sub_camera_target_ = nh_.subscribe("/camera/targets", 10, &ArmControlNode::cameraTargetCallback, this);
    
    // 订阅手眼变换节点消息（用于精确的机械臂控制）
    sub_handeye_ = nh_.subscribe("/handeye/ik_result", 10, &ArmControlNode::handeyeCallback, this);

    // 订阅果篮示教角度（可选，如用配置文件则不用订阅）
    sub_teach_basket_ = nh_.subscribe("/basket_teach_joints", 1, &ArmControlNode::teachBasketCallback, this);

    // 发布当前任务状态
    pub_status_ = nh_.advertise<std_msgs::String>("/arm_control/status", 10);
    
    // 订阅障碍检测节点消息
    sub_vision_ = nh_.subscribe("/camera_obstacle", 1, &ArmControlNode::visionCallback, this);
    
    // 订阅小车到位状态
    sub_chassis_arrival_ = nh_.subscribe("/chassis/arrival_status", 10, &ArmControlNode::chassisArrivalCallback, this);
    sub_chassis_status_ = nh_.subscribe("/chassis/status", 10, &ArmControlNode::chassisStatusCallback, this);
    
    // 订阅里程计获取小车当前位置
    sub_odom_ = nh_.subscribe("/odom", 10, &ArmControlNode::odomCallback, this);

    // 发布小车运动指令
    pub_chassis_target_ = nh_.advertise<geometry_msgs::PoseStamped>("/chassis/target_pose", 10);
    pub_emergency_stop_ = nh_.advertise<std_msgs::Bool>("/chassis/emergency_stop", 10);

    // 初始化服务客户端 - 使用piper规划组
    piper_client_ = nh_.serviceClient<moveit_ctrl::JointMoveitCtrl>("/joint_moveit_ctrl_piper");
    gripper_client_ = nh_.serviceClient<piper_msgs::Gripper>("/joint_moveit_ctrl_gripper");
    
    // 等待服务可用
    if (!piper_client_.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Piper moveit control service not available!");
    } else {
        ROS_INFO("Piper moveit control service is ready.");
    }
    
    if (!gripper_client_.waitForExistence(ros::Duration(5.0))) {
        ROS_ERROR("Gripper control service not available!");
    } else {
        ROS_INFO("Gripper control service is ready.");
    }

    // 获取参数
    nh_.param<double>("position_tolerance", position_tolerance_, 0.5);  // 位置容差
    nh_.param<double>("orientation_tolerance", orientation_tolerance_, 0.3); // 角度容差
    nh_.param<double>("optimal_distance", optimal_distance_, 1.0); // 最佳操作距离

    ROS_INFO("ArmControlNode initialized with camera-based control.");
}

void ArmControlNode::cameraTargetCallback(const arm_control_node::CameraTargets::ConstPtr& msg) {
    ROS_INFO("Received camera targets: apple(%.2f,%.2f,%.2f), tree(%.2f,%.2f,%.2f), type=%s", 
             msg->apple_pose.pose.position.x, msg->apple_pose.pose.position.y, msg->apple_pose.pose.position.z,
             msg->tree_pose.pose.position.x, msg->tree_pose.pose.position.y, msg->tree_pose.pose.position.z,
             msg->target_type.c_str());
    
    current_camera_targets_ = *msg;
    current_state_ = EVALUATE_POSITION;
    
    // 评估当前位置是否适合操作
    if (evaluatePosition(msg)) {
        ROS_INFO("Current position is suitable for operation.");
        current_state_ = EXECUTE_ARM_OPERATION;
        executeArmOperation();
    } else {
        ROS_INFO("Current position is not suitable. Planning chassis movement.");
        current_state_ = MOVE_CHASSIS;
        planChassisMovement(msg);
    }
}

void ArmControlNode::handeyeCallback(const your_msgs_pkg::HandEyeIK::ConstPtr& msg) {
    ROS_INFO("Received HandEyeIK target for precise control.");
    
    // 只有在小车到位后才执行精确的机械臂控制
    if (chassis_arrived_) {
        current_state_ = MOVE_TO_FRUIT;
        moveToJointAngles(msg->joint_angles);

        current_state_ = CUT_FRUIT;
        operateGripper("cut");

        current_state_ = RETRACT;
        retrieveToSavePoint(msg->joint_angles);

        current_state_ = MOVE_TO_BASKET;
        moveToJointAngles(basket_joint_angles_);    
        
        current_state_ = PLACE_FRUIT;
        operateGripper("release");

        current_state_ = RETRACT;
        moveToJointAngles(safe_joint_angles_);

        current_state_ = IDLE;
        chassis_arrived_ = false; // 重置状态
        publishStatus("task_complete");
    } else {
        ROS_WARN("Chassis not in position yet, ignoring HandEyeIK command.");
    }
}

void ArmControlNode::chassisArrivalCallback(const std_msgs::Bool::ConstPtr& msg) {
    chassis_arrived_ = msg->data;
    if (chassis_arrived_) {
        ROS_INFO("Chassis arrived at target position!");
        waiting_for_chassis_ = false;
        
        // 如果正在等待小车到位，现在可以执行机械臂操作
        if (current_state_ == WAIT_FOR_CHASSIS) {
            current_state_ = EXECUTE_ARM_OPERATION;
            executeArmOperation();
        }
    }
}

void ArmControlNode::chassisStatusCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("Chassis status: %s", msg->data.c_str());
    
    if (msg->data == "emergency_stop") {
        ROS_ERROR("Chassis emergency stop detected!");
        current_state_ = EMERGENCY_STOP;
        publishStatus("emergency_stop");
    }
}

void ArmControlNode::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 更新当前小车位置
    current_chassis_pose_.pose = msg->pose.pose;
    current_chassis_pose_.header = msg->header;
    
    // 可以添加调试信息（可选）
    ROS_DEBUG("Chassis position updated: x=%.2f, y=%.2f, yaw=%.2f", 
             current_chassis_pose_.pose.position.x,
             current_chassis_pose_.pose.position.y,
             tf::getYaw(current_chassis_pose_.pose.orientation));
}

bool ArmControlNode::evaluatePosition(const arm_control_node::CameraTargets::ConstPtr& targets) {
    
    // 获取当前小车位置（x, y坐标）
    double chassis_x = current_chassis_pose_.pose.position.x;
    double chassis_y = current_chassis_pose_.pose.position.y;
    double chassis_yaw = tf::getYaw(current_chassis_pose_.pose.orientation);
    
    // 获取目标苹果位置
    double apple_x = targets->apple_pose.pose.position.x;
    double apple_y = targets->apple_pose.pose.position.y;
    
    // 计算小车与苹果的距离
    double distance_to_apple = std::sqrt(
        std::pow(apple_x - chassis_x, 2) + 
        std::pow(apple_y - chassis_y, 2)
    );
    
    ROS_INFO("Position evaluation: chassis(%.2f,%.2f), apple(%.2f,%.2f), distance=%.2f, optimal=%.2f", 
             chassis_x, chassis_y, apple_x, apple_y, distance_to_apple, optimal_distance_);
    
    // 检查距离是否在合适范围内
    if (std::abs(distance_to_apple - optimal_distance_) < position_tolerance_) {
        // 计算从小车到苹果的角度
        double angle_to_apple = std::atan2(apple_y - chassis_y, apple_x - chassis_x);
        
        // 计算角度差（小车朝向与苹果方向的差异）
        double angle_diff = std::abs(chassis_yaw - angle_to_apple);
        // 处理角度跨越问题（-π到π）
        if (angle_diff > M_PI) {
            angle_diff = 2 * M_PI - angle_diff;
        }
        
        ROS_INFO("Angle evaluation: chassis_yaw=%.2f, apple_angle=%.2f, diff=%.2f, tolerance=%.2f", 
                 chassis_yaw, angle_to_apple, angle_diff, orientation_tolerance_);
        
        // 检查角度是否合适
        if (angle_diff < orientation_tolerance_) {
            // 检查是否存在与果树碰撞的风险
            if (!checkTreeCollisionRisk(targets->apple_pose, targets->tree_pose, current_chassis_pose_)) {
                ROS_INFO("Position is suitable for operation!");
                return true;
            } else {
                ROS_INFO("Tree collision risk detected, position not suitable.");
            }
        } else {
            ROS_INFO("Angle not suitable, chassis needs to rotate.");
        }
    } else {
        ROS_INFO("Distance not optimal, chassis needs to move.");
    }
    
    return false;
}

void ArmControlNode::planChassisMovement(const arm_control_node::CameraTargets::ConstPtr& targets) {
    /**
     * planChassisMovement方法的改进逻辑：
     * 1. 计算从苹果位置出发的多个候选位置
     * 2. 检查每个候选位置是否会导致与果树碰撞
     * 3. 选择距离果树最远且满足操作要求的位置
     * 4. 如果所有位置都有碰撞风险，放弃当前苹果
     */
    
    geometry_msgs::PoseStamped apple_pos = targets->apple_pose;
    geometry_msgs::PoseStamped tree_pos = targets->tree_pose;
    
    double apple_x = apple_pos.pose.position.x;
    double apple_y = apple_pos.pose.position.y;
    
    // 生成多个候选位置（围绕苹果的圆周）
    std::vector<geometry_msgs::PoseStamped> candidate_positions;
    int num_candidates = 8; // 8个方向
    
    for (int i = 0; i < num_candidates; ++i) {
        double angle = (2.0 * M_PI * i) / num_candidates;
        
        geometry_msgs::PoseStamped chassis_target;
        chassis_target.header.frame_id = apple_pos.header.frame_id;
        chassis_target.header.stamp = ros::Time::now();
        
        // 计算候选位置（在最佳距离处）
        chassis_target.pose.position.x = apple_x - optimal_distance_ * std::cos(angle);
        chassis_target.pose.position.y = apple_y - optimal_distance_ * std::sin(angle);
        chassis_target.pose.position.z = 0.0;
        
        // 设置朝向（面向苹果）
        tf::Quaternion q;
        q.setRPY(0, 0, angle);
        chassis_target.pose.orientation.x = q.x();
        chassis_target.pose.orientation.y = q.y();
        chassis_target.pose.orientation.z = q.z();
        chassis_target.pose.orientation.w = q.w();
        
        candidate_positions.push_back(chassis_target);
    }
    
    // 选择最佳位置（距离果树最远且无碰撞风险的位置）
    geometry_msgs::PoseStamped best_position;
    double max_tree_distance = -1.0;
    bool found_safe_position = false;
    
    for (const auto& candidate : candidate_positions) {
        // 检查是否与果树有碰撞风险
        if (!checkTreeCollisionRisk(apple_pos, tree_pos, candidate)) {
            // 计算候选位置与果树的距离
            double tree_distance = std::sqrt(
                std::pow(candidate.pose.position.x - tree_pos.pose.position.x, 2) +
                std::pow(candidate.pose.position.y - tree_pos.pose.position.y, 2)
            );
            
            if (tree_distance > max_tree_distance) {
                max_tree_distance = tree_distance;
                best_position = candidate;
                found_safe_position = true;
            }
        }
    }
    
    if (found_safe_position) {
        ROS_INFO("Found safe chassis position: x=%.2f, y=%.2f, distance_to_tree=%.2f", 
                 best_position.pose.position.x, best_position.pose.position.y, max_tree_distance);
        
        // 发布小车运动指令
        pub_chassis_target_.publish(best_position);
        
        current_state_ = WAIT_FOR_CHASSIS;
        waiting_for_chassis_ = true;
        publishStatus("waiting_for_chassis");
    } else {
        ROS_WARN("No safe position found for apple at (%.2f, %.2f). Skipping this apple due to tree collision risk.", 
                 apple_x, apple_y);
        
        current_state_ = IDLE;
        publishStatus("apple_skipped_collision_risk");
    }
}

bool ArmControlNode::checkTreeCollisionRisk(const geometry_msgs::PoseStamped& apple_pos, 
                                           const geometry_msgs::PoseStamped& tree_pos, 
                                           const geometry_msgs::PoseStamped& chassis_pos) {
    /**
     * 检查从指定小车位置操作苹果时是否有与果树碰撞的风险
     * 逻辑：如果小车距离果树的距离小于小车距离苹果的距离，则认为有碰撞风险
     */
    
    double chassis_x = chassis_pos.pose.position.x;
    double chassis_y = chassis_pos.pose.position.y;
    
    double apple_x = apple_pos.pose.position.x;
    double apple_y = apple_pos.pose.position.y;
    
    double tree_x = tree_pos.pose.position.x;
    double tree_y = tree_pos.pose.position.y;
    
    // 计算小车到苹果的距离
    double distance_to_apple = std::sqrt(
        std::pow(apple_x - chassis_x, 2) + 
        std::pow(apple_y - chassis_y, 2)
    );
    
    // 计算小车到果树的距离
    double distance_to_tree = std::sqrt(
        std::pow(tree_x - chassis_x, 2) + 
        std::pow(tree_y - chassis_y, 2)
    );
    
    // 设置安全余量（机械臂操作时需要的额外空间）
    double safety_margin = 0.5; // 50cm 安全余量
    
    ROS_DEBUG("Collision check: chassis(%.2f,%.2f), apple_dist=%.2f, tree_dist=%.2f, margin=%.2f", 
              chassis_x, chassis_y, distance_to_apple, distance_to_tree, safety_margin);
    
    // 如果小车距离果树太近，或者机械臂操作路径可能经过果树附近，则认为有碰撞风险
    if (distance_to_tree < (distance_to_apple + safety_margin)) {
        ROS_DEBUG("Tree collision risk detected: tree too close to chassis or in arm path");
        return true;
    }
    
    // 检查苹果、小车、果树三点的几何关系
    // 如果果树在小车到苹果的直线路径附近，也认为有风险
    double cross_product = std::abs(
        (tree_x - chassis_x) * (apple_y - chassis_y) - 
        (tree_y - chassis_y) * (apple_x - chassis_x)
    );
    double line_distance = cross_product / distance_to_apple;
    
    if (line_distance < safety_margin && 
        ((tree_x - chassis_x) * (apple_x - chassis_x) + (tree_y - chassis_y) * (apple_y - chassis_y)) > 0) {
        ROS_DEBUG("Tree collision risk detected: tree is near the arm operation path");
        return true;
    }
    
    return false;
}

void ArmControlNode::executeArmOperation() {
    ROS_INFO("Executing arm operation for target at: x=%.2f, y=%.2f, z=%.2f", 
             current_camera_targets_.apple_pose.pose.position.x, 
             current_camera_targets_.apple_pose.pose.position.y, 
             current_camera_targets_.apple_pose.pose.position.z);
    
    // 这里可以调用手眼标定或其他算法来计算精确的关节角度
    // 暂时使用简单的示例角度
    std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 示例角度
    
    current_state_ = MOVE_TO_FRUIT;
    moveToJointAngles(joint_angles);

    current_state_ = CUT_FRUIT;
    operateGripper("cut");

    current_state_ = RETRACT;
    retrieveToSavePoint(joint_angles);

    // 移动到果篮位置（如果有的话）
    if (!basket_joint_angles_.empty()) {
        current_state_ = MOVE_TO_BASKET;
        moveToJointAngles(basket_joint_angles_);    
        
        current_state_ = PLACE_FRUIT;
        operateGripper("release");
    }

    current_state_ = RETRACT;
    moveToJointAngles(safe_joint_angles_);

    current_state_ = IDLE;
    publishStatus("operation_complete");
}

void ArmControlNode::moveToJointAngles(const std::vector<double>& joint_angles) {
    moveit_ctrl::JointMoveitCtrl srv;
    
    // 设置关节角度（前6个关节）
    for (int i = 0; i < 6 && i < joint_angles.size(); ++i) {
        srv.request.joint_states[i] = joint_angles[i];
    }
    
    // 设置夹爪状态（保持当前状态）
    srv.request.gripper = 0.0; // 可以根据需要调整
    
    // 设置速度和加速度限制
    srv.request.max_velocity = 0.5;
    srv.request.max_acceleration = 0.5;
    
    if (piper_client_.call(srv)) {
        if (srv.response.status) {
            ROS_INFO("Piper movement executed successfully.");
        } else {
            ROS_ERROR("Piper movement failed with error code: %ld", srv.response.error_code);
            publishStatus(stateToString(current_state_) + "_motion_failed");
        }
    } else {
        ROS_ERROR("Failed to call piper moveit control service!");
        publishStatus(stateToString(current_state_) + "_service_failed");
    }
}

void ArmControlNode::operateGripper(const std::string& action) {
    piper_msgs::Gripper srv;
    
    if (action == "cut") {
        srv.request.gripper = 0.0; // 夹爪闭合
    } else if (action == "release") {
        srv.request.gripper = 0.035; // 夹爪张开
    } else {
        ROS_WARN("Unknown gripper action: %s", action.c_str());
        return;
    }
    
    if (gripper_client_.call(srv)) {
        if (srv.response.status) {
            ROS_INFO("Gripper control success");
        } else {
            ROS_ERROR("Gripper control failed");
        }
    } else {
        ROS_ERROR("Failed to call gripper control service!");
    }
}

void ArmControlNode::teachBasketCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    basket_joint_angles_ = msg->data;
    ROS_INFO("Received basket joint angles for placement.");
}

void ArmControlNode::publishStatus(const std::string& status) {
    std_msgs::String msg;
    msg.data = status;
    pub_status_.publish(msg);
}

std::string ArmControlNode::stateToString(State state) {
    switch (state) {
        case IDLE: return "IDLE";
        case EVALUATE_POSITION: return "EVALUATE_POSITION";
        case MOVE_CHASSIS: return "MOVE_CHASSIS";
        case WAIT_FOR_CHASSIS: return "WAIT_FOR_CHASSIS";
        case EXECUTE_ARM_OPERATION: return "EXECUTE_ARM_OPERATION";
        case MOVE_TO_FRUIT: return "MOVE_TO_FRUIT";
        case CUT_FRUIT: return "CUT_FRUIT";
        case RETRACT: return "RETRACT";
        case MOVE_TO_BASKET: return "MOVE_TO_BASKET";
        case PLACE_FRUIT: return "PLACE_FRUIT";
        case RECOVER: return "RECOVER";
        case EMERGENCY_STOP: return "EMERGENCY_STOP";
        default: return "UNKNOWN";
    }
}

// 检查路径是否安全（简化版本，直接尝试移动到目标位置）
void ArmControlNode::checkAndPlanPath(const std::vector<double>& from, const std::vector<double>& to) {
    // 由于使用服务接口，我们简化路径检查
    // 直接尝试移动到目标位置，如果失败则认为有碰撞
    moveit_ctrl::JointMoveitCtrl srv;
    
    // 设置目标关节角度
    for (int i = 0; i < 6 && i < to.size(); ++i) {
        srv.request.joint_states[i] = to[i];
    }
    
    srv.request.gripper = 0.0; // 保持当前夹爪状态
    srv.request.max_velocity = 0.3; // 降低速度进行路径检查
    srv.request.max_acceleration = 0.3;
    
    if (piper_client_.call(srv)) {
        if (srv.response.status) {
            ROS_INFO("Path planning from [from] to [to] succeeded: path is collision-free.");
        } else {
            ROS_WARN("Path planning from [from] to [to] failed: collision or infeasible path.");
            publishStatus("collision_detected");
        }
    } else {
        ROS_WARN("Failed to check path planning from [from] to [to].");
        publishStatus("path_check_failed");
    }
}

// 假设 joint_angles 是当前机械臂关节状态
// safe_joint_angles 是撤回的目标关节角度
void ArmControlNode::retrieveToSavePoint(const std::vector<double>& joint_angles) {
    // 模拟视觉检测是否有障碍（未来可用视觉节点回调结果）
    bool has_obstacle = detectObstacle(); // 这里用一个函数模拟，后面可用消息回调替换
    
    safe_joint_angles_ = joint_angles;

    if (!has_obstacle) {
        // 直接撤回到安全点
        current_state_ = RETRACT;
        moveToJointAngles(safe_joint_angles_);
    } else {
        ROS_WARN("Obstacle detected! Executing avoidance maneuver...");
        // 假设joint6是末端旋转关节，向下转动30度（约0.52弧度）
        if (safe_joint_angles_.size() >= 6) {
            safe_joint_angles_[5] += 0.52; // 修改joint6角度，模拟避障
        }
        current_state_ = RETRACT;
        moveToJointAngles(safe_joint_angles_);
        ROS_INFO("Moved to avoidance point. Checking if obstacle cleared...");
        // 此处可再次检测视觉画面是否清除障碍
        // 若仍有障碍，可循环调整或报警
    }
}

// 模拟障碍检测函数（后续替换为视觉节点消息回调结果）
bool ArmControlNode::detectObstacle() {
    // TODO: 替换为相机/视觉节点实际检测结果
    // 这里先用参数或随机模拟
    // 例如：return nh_.getParam("/vision/has_obstacle", false);
    return has_obstacle; // 使用从视觉回调中获取的状态
}

// 障碍检测回调函数
void ArmControlNode::visionCallback(const std_msgs::Bool::ConstPtr& msg) {
    has_obstacle = msg->data;
    ROS_INFO("Vision callback: obstacle detected = %s", has_obstacle ? "true" : "false");
}

void ArmControlNode::run() {
    ros::spin();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_control_node");
    ros::NodeHandle nh("~");
    ArmControlNode node(nh);
    node.run();
    return 0;
}