#include "arm_control_node.h"
#include <moveit_ctrl/JointMoveitCtrl.h>
#include <piper_msgs/Gripper.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

ArmControlNode::ArmControlNode(ros::NodeHandle& nh) : nh_(nh), current_state_(IDLE), 
                                                     waiting_for_chassis_(false), chassis_arrived_(false) {
    // 订阅相机目标点位（红色和绿色果实）
    sub_camera_target_ = nh_.subscribe("/camera/targets", 10, &ArmControlNode::cameraTargetCallback, this);
    
    // 订阅手眼变换节点消息（用于精确的机械臂控制）
    sub_handeye_ = nh_.subscribe("/handeye/ik_result", 10, &ArmControlNode::handeyeCallback, this);

    // 发布当前任务状态
    pub_status_ = nh_.advertise<std_msgs::String>("/arm_control/status", 10);
    
    // 订阅小车到位状态
    sub_chassis_arrival_ = nh_.subscribe("/chassis/arrival_status", 10, &ArmControlNode::chassisArrivalCallback, this);
    
    // 订阅里程计获取小车当前位置
    sub_odom_ = nh_.subscribe("/odom", 10, &ArmControlNode::odomCallback, this);

    // 发布小车运动指令
    pub_chassis_target_ = nh_.advertise<geometry_msgs::PoseStamped>("/chassis/target_pose", 10);

    // 初始化服务客户端 - 使用piper规划组
    piper_client_ = nh_.serviceClient<moveit_ctrl::JointMoveitCtrl>("/joint_moveit_ctrl_piper");
    gripper_client_ = nh_.serviceClient<piper_msgs::Gripper>("/joint_moveit_ctrl_gripper");
    
    // 循环等待服务可用，避免5秒超时问题
    ROS_INFO("Waiting for MoveIt services to become available...");
    while (!piper_client_.waitForExistence(ros::Duration(1.0)) && ros::ok()) {
        ROS_WARN("Piper moveit control service not available, retrying...");
        ros::Duration(1.0).sleep();
    }
    if (ros::ok()) {
        ROS_INFO("Piper moveit control service is ready.");
    }
    
    while (!gripper_client_.waitForExistence(ros::Duration(1.0)) && ros::ok()) {
        ROS_WARN("Gripper control service not available, retrying...");
        ros::Duration(1.0).sleep();
    }
    if (ros::ok()) {
        ROS_INFO("Gripper control service is ready.");
    }

    // 获取参数
    nh_.param<double>("position_tolerance", position_tolerance_, 0.5);  // 位置容差
    nh_.param<double>("orientation_tolerance", orientation_tolerance_, 0.3); // 角度容差
    nh_.param<double>("optimal_distance", optimal_distance_, 1.0); // 最佳操作距离
    // 新增：果实计数与存储区位姿
    nh_.param<int>("max_basket_count", max_basket_count_, 8);
    basket_count_ = 0;
    storage_pose_.header.frame_id = "map";
    nh_.param<double>("storage_pose_x", storage_pose_.pose.position.x, 0.0);
    nh_.param<double>("storage_pose_y", storage_pose_.pose.position.y, 0.0);
    nh_.param<double>("storage_pose_yaw", storage_pose_.pose.orientation.z, 0.0); // 用 yaw 生成四元数
    {
        double yaw = storage_pose_.pose.orientation.z;
        tf::Quaternion q;
        q.setRPY(0, 0, yaw);
        storage_pose_.pose.orientation.x = q.x();
        storage_pose_.pose.orientation.y = q.y();
        storage_pose_.pose.orientation.z = q.z();
        storage_pose_.pose.orientation.w = q.w();
    }

    // 从配置文件读取果树位置（用于安全避障）
    loadTreePositions();

    // 从配置文件读取示教角度
    loadTeachAngles();

    ROS_INFO("ArmControlNode initialized with red apple collection and green apple avoidance.");
}

void ArmControlNode::cameraTargetCallback(const arm_control_node::CameraTargets::ConstPtr& msg) {
    ROS_INFO("Received camera targets: red_apple(%.2f,%.2f,%.2f), green_apple(%.2f,%.2f,%.2f), type=%s", 
             msg->red_apple_pose.pose.position.x, msg->red_apple_pose.pose.position.y, msg->red_apple_pose.pose.position.z,
             msg->green_apple_pose.pose.position.x, msg->green_apple_pose.pose.position.y, msg->green_apple_pose.pose.position.z,
             msg->target_type.c_str());
    
    current_camera_targets_ = *msg;
    
    // 检查是否有红色果实需要采集
    if (msg->target_type == "red_apple" || msg->target_type == "both") {
        if (msg->red_confidence > 0.7) { // 置信度阈值
            current_state_ = EVALUATE_POSITION;
            
            // 评估当前位置是否适合操作红色果实
            if (evaluatePositionForRedApple(msg)) {
                ROS_INFO("Current position is suitable for red apple collection.");
                current_state_ = EXECUTE_ARM_OPERATION;
                executeArmOperation();
            } else {
                ROS_INFO("Current position is not suitable. Planning chassis movement.");
                current_state_ = MOVE_CHASSIS;
                planChassisMovementForRedApple(msg);
            }
        } else {
            ROS_WARN("Red apple confidence too low: %.2f", msg->red_confidence);
        }
    } else {
        ROS_INFO("No red apple detected, staying in IDLE state.");
        current_state_ = IDLE;
    }
}

void ArmControlNode::handeyeCallback(const eyes2hand::HandEyeIK::ConstPtr& msg) {
    ROS_INFO("Received HandEyeIK target for precise control.");
    
    // 只有在小车到位后才执行精确的机械臂控制
    if (chassis_arrived_) {
        current_state_ = MOVE_TO_FRUIT;
        moveToJointAngles(msg->joint_angles);

        current_state_ = CUT_FRUIT;
        operateGripper("cut");

        current_state_ = MOVE_TO_BASKET;
        moveToJointAngles(place_fruit_joint_angles_);    
        
        current_state_ = PLACE_FRUIT;
        operateGripper("release");

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

bool ArmControlNode::evaluatePositionForRedApple(const arm_control_node::CameraTargets::ConstPtr& targets) {
    /**
     * 评估当前位置是否适合采集红色果实
     * 综合检查距离、角度、安全避障等条件
     */
    
    // 获取当前小车位置（x, y坐标）
    double chassis_x = current_chassis_pose_.pose.position.x;
    double chassis_y = current_chassis_pose_.pose.position.y;
    double chassis_yaw = tf::getYaw(current_chassis_pose_.pose.orientation);
    
    // 获取目标红色果实位置
    double red_apple_x = targets->red_apple_pose.pose.position.x;
    double red_apple_y = targets->red_apple_pose.pose.position.y;
    
    // 计算小车与红色果实的距离
    double distance_to_red_apple = std::sqrt(
        std::pow(red_apple_x - chassis_x, 2) + 
        std::pow(red_apple_y - chassis_y, 2)
    );
    
    ROS_INFO("Position evaluation for red apple: chassis(%.2f,%.2f), red_apple(%.2f,%.2f), distance=%.2f, optimal=%.2f", 
             chassis_x, chassis_y, red_apple_x, red_apple_y, distance_to_red_apple, optimal_distance_);
    
    // 检查距离是否在合适范围内
    if (std::abs(distance_to_red_apple - optimal_distance_) < position_tolerance_) {
        // 计算从小车到红色果实的角度
        double angle_to_red_apple = std::atan2(red_apple_y - chassis_y, red_apple_x - chassis_x);
        
        // 计算角度差（小车朝向与红色果实方向的差异）
        double angle_diff = std::abs(chassis_yaw - angle_to_red_apple);
        // 处理角度跨越问题（-π到π）
        if (angle_diff > M_PI) {
            angle_diff = 2 * M_PI - angle_diff;
        }
        
        ROS_INFO("Angle evaluation: chassis_yaw=%.2f, red_apple_angle=%.2f, diff=%.2f, tolerance=%.2f", 
                 chassis_yaw, angle_to_red_apple, angle_diff, orientation_tolerance_);
        
        // 检查角度是否合适
        if (angle_diff < orientation_tolerance_) {
            // 综合安全检查（绿色果实避障 + 果树避障）
            if (checkSafetyForRedApple(targets, current_chassis_pose_)) {
                ROS_INFO("Position is suitable for red apple collection!");
                return true;
            } else {
                ROS_INFO("Safety check failed, position not suitable.");
            }
        } else {
            ROS_INFO("Angle not suitable, chassis needs to rotate.");
        }
    } else {
        ROS_INFO("Distance not optimal, chassis needs to move.");
    }
    
    return false;
}

bool ArmControlNode::checkSafetyForRedApple(const arm_control_node::CameraTargets::ConstPtr& targets, 
                                            const geometry_msgs::PoseStamped& chassis_pos) {
    /**
     * 综合安全检查：绿色果实避障 + 果树避障
     * 合并了原来的checkGreenAppleCollisionRisk和checkTreeCollisionRisk功能
     */
    
    double chassis_x = chassis_pos.pose.position.x;
    double chassis_y = chassis_pos.pose.position.y;
    
    double red_apple_x = targets->red_apple_pose.pose.position.x;
    double red_apple_y = targets->red_apple_pose.pose.position.y;
    
    // 计算小车到红色果实的距离
    double distance_to_red_apple = std::sqrt(
        std::pow(red_apple_x - chassis_x, 2) + 
        std::pow(red_apple_y - chassis_y, 2)
    );
    
    // 1. 检查绿色果实避障
    if (targets->target_type != "red_apple" && targets->green_apple_pose.pose.position.x != 0.0) {
        double green_apple_x = targets->green_apple_pose.pose.position.x;
        double green_apple_y = targets->green_apple_pose.pose.position.y;
        
        double distance_to_green = std::sqrt(
            std::pow(green_apple_x - chassis_x, 2) + 
            std::pow(green_apple_y - chassis_y, 2)
        );
        
        // 绿色果实安全余量（80cm，确保不接触）
        double green_safety_margin = 0.8;
        
        // 如果绿色果实距离太近，认为有碰撞风险
        if (distance_to_green < (distance_to_red_apple + green_safety_margin)) {
            ROS_WARN("Green apple collision risk detected: green apple too close to operation area");
            return false;
        }
        
        // 检查绿色果实是否在机械臂操作路径上
        double cross_product = std::abs(
            (green_apple_x - chassis_x) * (red_apple_y - chassis_y) - 
            (green_apple_y - chassis_y) * (red_apple_x - chassis_x)
        );
        double line_distance = cross_product / distance_to_red_apple;
        
        if (line_distance < green_safety_margin && 
            ((green_apple_x - chassis_x) * (red_apple_x - chassis_x) + 
             (green_apple_y - chassis_y) * (red_apple_y - chassis_y)) > 0) {
            ROS_WARN("Green apple collision risk detected: green apple is near the arm operation path");
            return false;
        }
        
        // 检查绿色果实是否在红色果实的采集范围内
        double distance_between_apples = std::sqrt(
            std::pow(red_apple_x - green_apple_x, 2) + 
            std::pow(red_apple_y - green_apple_y, 2)
        );
        
        if (distance_between_apples < green_safety_margin) {
            ROS_WARN("Green apple collision risk detected: green apple too close to red apple");
            return false;
        }
    }
    
    // 2. 检查果树避障
    double tree_safety_margin = 0.5; // 50cm 安全余量
    for (const auto& tree : tree_positions_) {
        double tree_x = tree.pose.position.x;
        double tree_y = tree.pose.position.y;
        
        double distance_to_tree = std::sqrt(
            std::pow(tree_x - chassis_x, 2) + 
            std::pow(tree_y - chassis_y, 2)
        );
        
        // 如果小车距离果树太近，认为有碰撞风险
        if (distance_to_tree < (distance_to_red_apple + tree_safety_margin)) {
            ROS_DEBUG("Tree collision risk detected: tree too close to chassis or in arm path");
            return false;
        }
        
        // 检查果树是否在机械臂操作路径上
        double cross_product = std::abs(
            (tree_x - chassis_x) * (red_apple_y - chassis_y) - 
            (tree_y - chassis_y) * (red_apple_x - chassis_x)
        );
        double line_distance = cross_product / distance_to_red_apple;
        
        if (line_distance < tree_safety_margin && 
            ((tree_x - chassis_x) * (red_apple_x - chassis_x) + 
             (tree_y - chassis_y) * (red_apple_y - chassis_y)) > 0) {
            ROS_DEBUG("Tree collision risk detected: tree is near the arm operation path");
            return false;
        }
    }
    
    return true; // 所有安全检查通过
}

void ArmControlNode::planChassisMovementForRedApple(const arm_control_node::CameraTargets::ConstPtr& targets) {
    /**
     * 为红色果实采集规划小车移动
     * 选择最安全的位置，避免与绿色果实和果树碰撞
     */
    
    geometry_msgs::PoseStamped red_apple_pos = targets->red_apple_pose;
    
    double red_apple_x = red_apple_pos.pose.position.x;
    double red_apple_y = red_apple_pos.pose.position.y;
    
    // 生成多个候选位置（围绕红色果实的圆周）
    std::vector<geometry_msgs::PoseStamped> candidate_positions;
    int num_candidates = 8; // 8个方向
    
    for (int i = 0; i < num_candidates; ++i) {
        double angle = (2.0 * M_PI * i) / num_candidates;
        
        geometry_msgs::PoseStamped chassis_target;
        chassis_target.header.frame_id = red_apple_pos.header.frame_id;
        chassis_target.header.stamp = ros::Time::now();
        
        // 计算候选位置（在最佳距离处）
        chassis_target.pose.position.x = red_apple_x - optimal_distance_ * std::cos(angle);
        chassis_target.pose.position.y = red_apple_y - optimal_distance_ * std::sin(angle);
        chassis_target.pose.position.z = 0.0;
        
        // 设置朝向（面向红色果实）
        tf::Quaternion q;
        q.setRPY(0, 0, angle);
        chassis_target.pose.orientation.x = q.x();
        chassis_target.pose.orientation.y = q.y();
        chassis_target.pose.orientation.z = q.z();
        chassis_target.pose.orientation.w = q.w();
        
        candidate_positions.push_back(chassis_target);
    }
    
    // 选择最佳位置（通过安全检查且最安全的位置）
    geometry_msgs::PoseStamped best_position;
    double max_safety_score = -1.0;
    bool found_safe_position = false;
    
    for (const auto& candidate : candidate_positions) {
        // 使用综合安全检查方法
        if (checkSafetyForRedApple(targets, candidate)) {
            // 计算安全评分（综合考虑距离绿色果实和果树的远近）
            double safety_score = calculateSafetyScore(targets, candidate);
            
            if (safety_score > max_safety_score) {
                max_safety_score = safety_score;
                best_position = candidate;
                found_safe_position = true;
            }
        }
    }
    
    if (found_safe_position) {
        ROS_INFO("Found safe chassis position: x=%.2f, y=%.2f, safety_score=%.2f", 
                 best_position.pose.position.x, best_position.pose.position.y, max_safety_score);
        
        // 发布小车运动指令
        pub_chassis_target_.publish(best_position);
        
        current_state_ = WAIT_FOR_CHASSIS;
        waiting_for_chassis_ = true;
        publishStatus("waiting_for_chassis");
    } else {
        ROS_WARN("No safe position found for red apple at (%.2f, %.2f). Skipping this apple due to collision risk.", 
                 red_apple_x, red_apple_y);
        
        current_state_ = IDLE;
        publishStatus("red_apple_skipped_collision_risk");
    }
}

double ArmControlNode::calculateSafetyScore(const arm_control_node::CameraTargets::ConstPtr& targets, 
                                           const geometry_msgs::PoseStamped& chassis_pos) {
    /**
     * 计算指定小车位置的安全评分
     * 评分越高表示位置越安全
     */
    double safety_score = 0.0;
    
    double chassis_x = chassis_pos.pose.position.x;
    double chassis_y = chassis_pos.pose.position.y;
    
    // 计算与绿色果实的安全距离评分
    if (targets->target_type != "red_apple" && targets->green_apple_pose.pose.position.x != 0.0) {
        double green_apple_x = targets->green_apple_pose.pose.position.x;
        double green_apple_y = targets->green_apple_pose.pose.position.y;
        
        double distance_to_green = std::sqrt(
            std::pow(green_apple_x - chassis_x, 2) + 
            std::pow(green_apple_y - chassis_y, 2)
        );
        
        // 距离越远，安全评分越高
        safety_score += std::min(distance_to_green / 5.0, 1.0); // 归一化到0-1
    }
    
    // 计算与果树的安全距离评分
    for (const auto& tree : tree_positions_) {
        double tree_x = tree.pose.position.x;
        double tree_y = tree.pose.position.y;
        
        double distance_to_tree = std::sqrt(
            std::pow(tree_x - chassis_x, 2) + 
            std::pow(tree_y - chassis_y, 2)
        );
        
        // 距离越远，安全评分越高
        safety_score += std::min(distance_to_tree / 3.0, 1.0); // 归一化到0-1
    }
    
    return safety_score;
}

void ArmControlNode::executeArmOperation() {
    ROS_INFO("Executing arm operation for red apple at: x=%.2f, y=%.2f, z=%.2f", 
             current_camera_targets_.red_apple_pose.pose.position.x, 
             current_camera_targets_.red_apple_pose.pose.position.y, 
             current_camera_targets_.red_apple_pose.pose.position.z);
    
    // 这里可以调用手眼标定或其他算法来计算精确的关节角度
    // 暂时使用简单的示例角度
    std::vector<double> joint_angles = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // 示例角度
    
    current_state_ = MOVE_TO_FRUIT;
    moveToJointAngles(joint_angles);

    current_state_ = CUT_FRUIT;
    operateGripper("cut");

        // 移动到果篮位置（如果有的话）
        if (!place_fruit_joint_angles_.empty()) {
            current_state_ = MOVE_TO_BASKET;
            moveToJointAngles(place_fruit_joint_angles_);    
            
            current_state_ = PLACE_FRUIT;
            operateGripper("release");
            // 放果完成后计数 +1，并在需要时启动倒果流程
            basket_count_ += 1;
            publishStatus("basket_count_" + std::to_string(basket_count_));
            startDumpIfNeeded();
        }

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
        srv.request.gripper_angle = 0.0; // 夹爪闭合
        srv.request.gripper_effort = 0.0; // 夹爪力度
        srv.request.gripper_code = 0; // 夹爪代码
        srv.request.set_zero = 0; // 不设置零点
    } else if (action == "release") {
        srv.request.gripper_angle = 0.035; // 夹爪张开
        srv.request.gripper_effort = 0.0; // 夹爪力度
        srv.request.gripper_code = 0; // 夹爪代码
        srv.request.set_zero = 0; // 不设置零点
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

        case MOVE_TO_STORAGE: return "MOVE_TO_STORAGE";
        case WAIT_FOR_STORAGE: return "WAIT_FOR_STORAGE";
        case DUMP_FRUITS: return "DUMP_FRUITS";
        case RETURN_TO_WORKSITE: return "RETURN_TO_WORKSITE";
        case WAIT_FOR_RETURN: return "WAIT_FOR_RETURN";
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

void ArmControlNode::run() {
    ros::spin();
}

// 新增：如达到阈值，触发倒果流程
void ArmControlNode::startDumpIfNeeded() {
    if (basket_count_ < max_basket_count_) {
        return;
    }
    // 记录返回点为当前底盘位姿
    return_pose_ = current_chassis_pose_;
    // 发布去往存储区的目标
    publishStatus("dump_triggered");
    publishChassisTarget(storage_pose_);
    current_state_ = MOVE_TO_STORAGE;
    waiting_for_chassis_ = true;
}

void ArmControlNode::publishChassisTarget(const geometry_msgs::PoseStamped& pose) {
    geometry_msgs::PoseStamped target = pose;
    target.header.stamp = ros::Time::now();
    pub_chassis_target_.publish(target);
}

// 在到达回调里衔接倒果状态机
void ArmControlNode::chassisArrivalCallback(const std_msgs::Bool::ConstPtr& msg) {
    chassis_arrived_ = msg->data;
    if (chassis_arrived_) {
        ROS_INFO("Chassis arrived at target position!");
        waiting_for_chassis_ = false;

        if (current_state_ == WAIT_FOR_CHASSIS) {
            current_state_ = EXECUTE_ARM_OPERATION;
            executeArmOperation();
        } else if (current_state_ == MOVE_TO_STORAGE) {
            current_state_ = WAIT_FOR_STORAGE;
            // 简化：到位即执行倒果
            performDumpFruits();
        } else if (current_state_ == RETURN_TO_WORKSITE) {
            current_state_ = WAIT_FOR_RETURN;
            // 回到作业点，恢复为 IDLE，继续摄像头回调驱动任务
            publishStatus("returned_to_worksite");
            current_state_ = IDLE;
        }
    }
}

void ArmControlNode::performDumpFruits() {
    publishStatus("dump_fruits_start");
    // TODO 这里可加入机械臂/执行器动作以倒果
    // 简化：仅发布状态并清零计数
    basket_count_ = 0;
    publishStatus("dump_fruits_done");
    // 返回原作业位姿
    publishChassisTarget(return_pose_);
    current_state_ = RETURN_TO_WORKSITE;
    waiting_for_chassis_ = true;
}

void ArmControlNode::loadTreePositions() {
    // 从配置文件读取果树位置
    XmlRpc::XmlRpcValue tree_poses;
    if (nh_.getParam("tree_positions", tree_poses)) {
        if (tree_poses.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for (int i = 0; i < tree_poses.size(); ++i) {
                if (tree_poses[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    geometry_msgs::PoseStamped tree_pose;
                    tree_pose.header.frame_id = "map"; // 假设所有果树都在地图坐标系下
                    tree_pose.header.stamp = ros::Time::now();
                    
                    if (tree_poses[i]["x"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        tree_pose.pose.position.x = static_cast<double>(tree_poses[i]["x"]);
                    }
                    if (tree_poses[i]["y"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        tree_pose.pose.position.y = static_cast<double>(tree_poses[i]["y"]);
                    }
                    if (tree_poses[i]["z"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        tree_pose.pose.position.z = static_cast<double>(tree_poses[i]["z"]);
                    }
                    
                    if (tree_poses[i]["yaw"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                        double yaw = static_cast<double>(tree_poses[i]["yaw"]);
                        tf::Quaternion q;
                        q.setRPY(0, 0, yaw);
                        tree_pose.pose.orientation.x = q.x();
                        tree_pose.pose.orientation.y = q.y();
                        tree_pose.pose.orientation.z = q.z();
                        tree_pose.pose.orientation.w = q.w();
                    }
                    tree_positions_.push_back(tree_pose);
                    ROS_INFO("Loaded tree position %d: x=%.2f, y=%.2f, yaw=%.2f", i, 
                             tree_pose.pose.position.x, tree_pose.pose.position.y, tf::getYaw(tree_pose.pose.orientation));
                }
            }
        } else {
            ROS_WARN("tree_positions parameter is not an array.");
        }
    } else {
        ROS_WARN("tree_positions parameter not found in the parameter server.");
    }
}

void ArmControlNode::loadTeachAngles() {
    // 从配置文件读取示教角度
    std::vector<double> temp_angles;
    
    // 读取放果位置角度
    if (nh_.getParam("place_fruit_position/joint_angles", temp_angles)) {
        place_fruit_joint_angles_ = temp_angles;
        ROS_INFO("Loaded place fruit position angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                 place_fruit_joint_angles_[0], place_fruit_joint_angles_[1], place_fruit_joint_angles_[2],
                 place_fruit_joint_angles_[3], place_fruit_joint_angles_[4], place_fruit_joint_angles_[5]);
    } else {
        // 设置默认放果角度
        place_fruit_joint_angles_ = {0.0, -0.5, 0.0, -1.0, 0.0, 0.0};
        ROS_WARN("Failed to load place fruit position angles, using defaults");
    }
    
    // 读取抓取果篮位置角度
    if (nh_.getParam("grab_basket_position/joint_angles", temp_angles)) {
        grab_basket_joint_angles_ = temp_angles;
        ROS_INFO("Loaded grab basket position angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                 grab_basket_joint_angles_[0], grab_basket_joint_angles_[1], grab_basket_joint_angles_[2],
                 grab_basket_joint_angles_[3], grab_basket_joint_angles_[4], grab_basket_joint_angles_[5]);
    } else {
        // 设置默认抓取果篮角度
        grab_basket_joint_angles_ = {0.0, -0.8, 0.0, -1.2, 0.0, 0.0};
        ROS_WARN("Failed to load grab basket position angles, using defaults");
    }
    
    // 读取倾倒果子位置角度
    if (nh_.getParam("dump_fruit_position/joint_angles", temp_angles)) {
        dump_fruit_joint_angles_ = temp_angles;
        ROS_INFO("Loaded dump fruit position angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                 dump_fruit_joint_angles_[0], dump_fruit_joint_angles_[1], dump_fruit_joint_angles_[2],
                 dump_fruit_joint_angles_[3], dump_fruit_joint_angles_[4], dump_fruit_joint_angles_[5]);
    } else {
        // 设置默认倾倒角度
        dump_fruit_joint_angles_ = {0.0, -1.0, 0.0, -0.8, 0.0, 0.0};
        ROS_WARN("Failed to load dump fruit position angles, using defaults");
    }
    
    // 读取果篮粗定位角度
    if (nh_.getParam("basket_return_position/rough_position/joint_angles", temp_angles)) {
        basket_return_rough_angles_ = temp_angles;
        ROS_INFO("Loaded basket return rough position angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                 basket_return_rough_angles_[0], basket_return_rough_angles_[1], basket_return_rough_angles_[2],
                 basket_return_rough_angles_[3], basket_return_rough_angles_[4], basket_return_rough_angles_[5]);
    } else {
        // 设置默认粗定位角度
        basket_return_rough_angles_ = {0.0, -0.6, 0.0, -1.1, 0.0, 0.0};
        ROS_WARN("Failed to load basket return rough position angles, using defaults");
    }
    
    // 读取果篮精确放回角度
    if (nh_.getParam("basket_return_position/precise_position/joint_angles", temp_angles)) {
        basket_return_precise_angles_ = temp_angles;
        ROS_INFO("Loaded basket return precise position angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                 basket_return_precise_angles_[0], basket_return_precise_angles_[1], basket_return_precise_angles_[2],
                 basket_return_precise_angles_[3], basket_return_precise_angles_[4], basket_return_precise_angles_[5]);
    } else {
        // 设置默认精确放回角度
        basket_return_precise_angles_ = {0.0, -0.55, 0.0, -1.05, 0.0, 0.0};
        ROS_WARN("Failed to load basket return precise position angles, using defaults");
    }
    
    ROS_INFO("All teach angles loaded successfully.");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_control_node");
    ros::NodeHandle nh("~");
    ArmControlNode node(nh);
    node.run();
    return 0;
}