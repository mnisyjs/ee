#include "arm_control_node.h"
#include <moveit_ctrl/JointMoveitCtrl.h>
#include <piper_msgs/Gripper.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>

ArmControlNode::ArmControlNode(ros::NodeHandle& nh) : nh_(nh), current_state_(IDLE), 
                                                     waiting_for_chassis_(false), chassis_arrived_(false),
                                                     next_red_apple_id_(1) {
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
    
    // 初始化剪刀变换矩阵
    initializeScissorTransform();

    ROS_INFO("ArmControlNode initialized with red apple collection and tree collision avoidance.");
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
            // 将红果添加到队列中
            addRedAppleToQueue(msg);
            
            // 如果当前没有任务在执行，开始处理红果队列
            if (current_state_ == IDLE) {
                processRedAppleQueue();
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
    last_handeye_ik_ = *msg;
    // 只有在小车到位后才执行精确的机械臂控制
    if (chassis_arrived_) {
        // 应用剪刀变换到目标位姿
        if (!last_handeye_ik_.matrix.empty() && last_handeye_ik_.matrix.size() >= 16) {
            // 如果有变换矩阵，应用剪刀变换
            geometry_msgs::PoseStamped target_pose;
            target_pose.header.frame_id = last_handeye_ik_.reference_frame;
            target_pose.pose.position.x = last_handeye_ik_.matrix[3];   // x
            target_pose.pose.position.y = last_handeye_ik_.matrix[7];   // y
            target_pose.pose.position.z = last_handeye_ik_.matrix[11];  // z
            
            applyScissorTransform(target_pose);
            ROS_INFO("Applied scissor transform to target pose: (%.3f, %.3f, %.3f)", 
                     target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);
        }
        
        current_state_ = MOVE_TO_FRUIT;
        moveToJointAngles(last_handeye_ik_.joint_angles);

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
     * 简化版安全检查：只检查果树避障
     * 删除了绿色果实的安全距离检测
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
    
    // 删除绿色果实安全检查，只保留果树避障检查
    
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
    
    return true; // 果树安全检查通过
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
     * 只考虑与果树的安全距离
     */
    double safety_score = 0.0;
    
    double chassis_x = chassis_pos.pose.position.x;
    double chassis_y = chassis_pos.pose.position.y;
    
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
    
    // 等待手眼变换节点返回逆解结果
    ros::Time start_time = ros::Time::now();
    double timeout = 1.0; // 等待1秒    
    bool received_ik = false;
    std::vector<double> joint_angles;

    while (ros::ok() && (ros::Time::now() - start_time).toSec() < timeout) {
        if (!last_handeye_ik_.joint_angles.empty()) {
            joint_angles = last_handeye_ik_.joint_angles;
            received_ik = true;
            break;
        }
        ros::Duration(0.05).sleep();
        ros::spinOnce(); // 保证能收到回调
    }
    
    current_state_ = MOVE_TO_FRUIT;
    moveToJointAngles(joint_angles);

    current_state_ = CUT_FRUIT;
    operateGripper("cut");
    ros::Duration(1.0).sleep();

    // 移动到放果位置（如果有的话）
    if (!place_fruit_joint_angles_.empty()) {
        current_state_ = MOVE_TO_BASKET;
        moveToJointAngles(place_fruit_joint_angles_);    
        
        current_state_ = PLACE_FRUIT;
        operateGripper("release");
        ros::Duration(1.0).sleep();
        // 放果完成后计数 +1，并在需要时启动倒果流程
        basket_count_ += 1;
        publishStatus("basket_count_" + std::to_string(basket_count_));
        startDumpIfNeeded();
    }

    current_state_ = IDLE;
    publishStatus("operation_complete");

    // 清空已用的IK结果，保证下次采集时再次等待
    last_handeye_ik_.joint_angles.clear();
    last_handeye_ik_.matrix.clear();
    last_handeye_ik_.reference_frame.clear();
    
    // 从队列中移除已处理的红果
    if (!red_apple_queue_.empty()) {
        red_apple_queue_.pop();
        ROS_INFO("Removed processed red apple from queue. Remaining: %zu", red_apple_queue_.size());
        
        // 如果队列中还有红果，继续处理
        if (!red_apple_queue_.empty()) {
            processRedAppleQueue();
        }
    }
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

// 如达到阈值，触发倒果流程
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

    // 步骤1：机械臂移动到抓取果篮位置，夹爪张开
    moveToJointAngles(grab_basket_joint_angles_);
    operateGripper("release");  // 张开夹爪
    ros::Duration(2.0).sleep(); // 留一点时间，防止动作没执行完

    // 步骤2：夹爪收紧，抓取果篮
    operateGripper("cut");      // 收紧夹爪
    ros::Duration(1.0).sleep();

    // 步骤3：机械臂移动到倾倒果子位置，等待5秒
    moveToJointAngles(dump_fruit_joint_angles_);
    ros::Duration(5.0).sleep(); // 倒果操作

    // 步骤4：机械臂移动到果篮复原位置
    moveToJointAngles(basket_return_angles_);
    ros::Duration(2.0).sleep();

    // 步骤5：发布倒果完成指令
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
    } 
    
    // 读取抓取果篮位置角度
    if (nh_.getParam("grab_basket_position/joint_angles", temp_angles)) {
        grab_basket_joint_angles_ = temp_angles;
        ROS_INFO("Loaded grab basket position angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                 grab_basket_joint_angles_[0], grab_basket_joint_angles_[1], grab_basket_joint_angles_[2],
                 grab_basket_joint_angles_[3], grab_basket_joint_angles_[4], grab_basket_joint_angles_[5]);
    } 
    
    // 读取倾倒果子位置角度
    if (nh_.getParam("dump_fruit_position/joint_angles", temp_angles)) {
        dump_fruit_joint_angles_ = temp_angles;
        ROS_INFO("Loaded dump fruit position angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                 dump_fruit_joint_angles_[0], dump_fruit_joint_angles_[1], dump_fruit_joint_angles_[2],
                 dump_fruit_joint_angles_[3], dump_fruit_joint_angles_[4], dump_fruit_joint_angles_[5]);
    } 
    
    // 读取果篮定位角度
    if (nh_.getParam("basket_return_position/joint_angles", temp_angles)) {
        basket_return_angles_ = temp_angles;
        ROS_INFO("Loaded basket return position angles: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]", 
                 basket_return_angles_[0], basket_return_angles_[1], basket_return_angles_[2],
                 basket_return_angles_[3], basket_return_angles_[4], basket_return_angles_[5]);
    } 
    
    
    ROS_INFO("All teach angles loaded successfully.");
}

void ArmControlNode::initializeScissorTransform() {
    // 初始化剪刀变换矩阵
    // 变换矩阵格式：[R, t; 0, 1] 其中 R=eye(3), t=(0.01, 0.02, 0.11)
    
    // 设置旋转矩阵为单位矩阵
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            scissor_transform_[i][j] = (i == j) ? 1.0 : 0.0;
        }
    }
    
    // 设置平移向量 (0.01, 0.02, 0.11)
    scissor_transform_[0][3] = 0.01;  // x方向偏移
    scissor_transform_[1][3] = 0.02;  // y方向偏移  
    scissor_transform_[2][3] = 0.11;  // z方向偏移
    
    // 设置最后一行 [0, 0, 0, 1]
    scissor_transform_[3][0] = 0.0;
    scissor_transform_[3][1] = 0.0;
    scissor_transform_[3][2] = 0.0;
    scissor_transform_[3][3] = 1.0;
    
    // 设置位置容差范围（可配置）
    nh_.param<double>("scissor_x_tolerance", scissor_position_tolerance_[0], 0.02);  // x方向容差
    nh_.param<double>("scissor_y_tolerance", scissor_position_tolerance_[1], 0.03);  // y方向容差
    nh_.param<double>("scissor_z_tolerance", scissor_position_tolerance_[2], 0.05);  // z方向容差
    
    ROS_INFO("Scissor transform initialized: translation=(%.3f, %.3f, %.3f), tolerance=(%.3f, %.3f, %.3f)", 
             scissor_transform_[0][3], scissor_transform_[1][3], scissor_transform_[2][3],
             scissor_position_tolerance_[0], scissor_position_tolerance_[1], scissor_position_tolerance_[2]);
}


void ArmControlNode::run() {
    ros::spin();
}

// 新增：红果队列管理方法实现
void ArmControlNode::addRedAppleToQueue(const arm_control_node::CameraTargets::ConstPtr& targets) {
    // 检查是否已经检测过这个红果（通过位置判断，避免重复）
    bool is_duplicate = false;
    for (const auto& pair : detected_red_apples_) {
        const RedAppleInfo& existing = pair.second;
        double distance = std::sqrt(
            std::pow(existing.pose.pose.position.x - targets->red_apple_pose.pose.position.x, 2) +
            std::pow(existing.pose.pose.position.y - targets->red_apple_pose.pose.position.y, 2) +
            std::pow(existing.pose.pose.position.z - targets->red_apple_pose.pose.position.z, 2)
        );
        
        // 如果距离小于阈值，认为是同一个红果
        if (distance < 0.05) { // 5cm阈值
            is_duplicate = true;
            ROS_INFO("Red apple at (%.2f, %.2f, %.2f) already detected, skipping duplicate", 
                     targets->red_apple_pose.pose.position.x,
                     targets->red_apple_pose.pose.position.y,
                     targets->red_apple_pose.pose.position.z);
            break;
        }
    }
    
    if (!is_duplicate) {
        RedAppleInfo new_red_apple;
        new_red_apple.pose = targets->red_apple_pose;
        new_red_apple.id = next_red_apple_id_++;
        new_red_apple.confidence = targets->red_confidence;
        new_red_apple.detection_time = ros::Time::now();
        
        // 检查红果是否安全
        new_red_apple.is_safe = isRedAppleSafe(new_red_apple);
        
        // 添加到检测记录
        detected_red_apples_[new_red_apple.id] = new_red_apple;
        
        // 如果安全，添加到采摘队列
        if (new_red_apple.is_safe) {
            red_apple_queue_.push(new_red_apple);
            ROS_INFO("Added safe red apple %d to queue at (%.2f, %.2f, %.2f), confidence=%.2f", 
                     new_red_apple.id,
                     new_red_apple.pose.pose.position.x,
                     new_red_apple.pose.pose.position.y,
                     new_red_apple.pose.pose.position.z,
                     new_red_apple.confidence);
        } else {
            ROS_WARN("Red apple %d at (%.2f, %.2f, %.2f) is not safe, skipping", 
                     new_red_apple.id,
                     new_red_apple.pose.pose.position.x,
                     new_red_apple.pose.pose.position.y,
                     new_red_apple.pose.pose.position.z);
        }
    }
}

void ArmControlNode::processRedAppleQueue() {
    if (red_apple_queue_.empty()) {
        ROS_INFO("Red apple queue is empty, staying in IDLE state");
        current_state_ = IDLE;
        return;
    }
    
    // 从队列中取出下一个红果（从后往前采摘）
    RedAppleInfo& next_red_apple = red_apple_queue_.front();
    
    ROS_INFO("Processing red apple %d at (%.2f, %.2f, %.2f)", 
             next_red_apple.id,
             next_red_apple.pose.pose.position.x,
             next_red_apple.pose.pose.position.y,
             next_red_apple.pose.pose.position.z);
    
    // 评估当前位置是否适合操作这个红果
    current_state_ = EVALUATE_POSITION;
    
    // 创建临时的CameraTargets消息用于评估
    arm_control_node::CameraTargets temp_targets;
    temp_targets.red_apple_pose = next_red_apple.pose;
    temp_targets.red_confidence = next_red_apple.confidence;
    temp_targets.target_type = "red_apple";
    
    if (evaluatePositionForRedApple(&temp_targets)) {
        ROS_INFO("Current position is suitable for red apple %d collection.", next_red_apple.id);
        current_state_ = EXECUTE_ARM_OPERATION;
        executeArmOperation();
    } else {
        ROS_INFO("Current position is not suitable for red apple %d. Planning chassis movement.", next_red_apple.id);
        current_state_ = MOVE_CHASSIS;
        planChassisMovementForRedApple(&temp_targets);
    }
}

bool ArmControlNode::isRedAppleSafe(const RedAppleInfo& red_apple) {
    // 简化版安全检查：只检查与果树的碰撞风险
    // 如果小车到红果距离大于小车到果树距离，则认为有碰撞风险
    
    double red_apple_x = red_apple.pose.pose.position.x;
    double red_apple_y = red_apple.pose.pose.position.y;
    
    // 检查与果树的碰撞风险
    double tree_safety_margin = 0.3; // 30cm 安全余量
    for (const auto& tree : tree_positions_) {
        double tree_x = tree.pose.position.x;
        double tree_y = tree.pose.position.y;
        
        // 计算果树到红果的距离
        double tree_to_red_distance = std::sqrt(
            std::pow(tree_x - red_apple_x, 2) + 
            std::pow(tree_y - red_apple_y, 2)
        );
        
        // 如果果树距离红果太近，认为有碰撞风险
        if (tree_to_red_distance < tree_safety_margin) {
            ROS_WARN("Red apple %d is too close to tree at (%.2f, %.2f), distance=%.2f, safety_margin=%.2f", 
                     red_apple.id, tree_x, tree_y, tree_to_red_distance, tree_safety_margin);
            return false;
        }
    }
    
    return true;
}

void ArmControlNode::applyScissorTransform(geometry_msgs::PoseStamped& target_pose) {
    // 应用剪刀变换矩阵到目标位姿
    // target_pose是在相机坐标系下的，需要转换到剪刀坐标系下
    
    double x = target_pose.pose.position.x;
    double y = target_pose.pose.position.y;
    double z = target_pose.pose.position.z;
    
    // 应用变换矩阵：新位置 = R * 原位置 + t
    double new_x = scissor_transform_[0][0] * x + scissor_transform_[0][1] * y + scissor_transform_[0][2] * z + scissor_transform_[0][3];
    double new_y = scissor_transform_[1][0] * x + scissor_transform_[1][1] * y + scissor_transform_[1][2] * z + scissor_transform_[1][3];
    double new_z = scissor_transform_[2][0] * x + scissor_transform_[2][1] * y + scissor_transform_[2][2] * z + scissor_transform_[2][3];
    
    target_pose.pose.position.x = new_x;
    target_pose.pose.position.y = new_y;
    target_pose.pose.position.z = new_z;
    
    ROS_DEBUG("Applied scissor transform: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)", 
              x, y, z, new_x, new_y, new_z);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_control_node");
    ros::NodeHandle nh("~");
    ArmControlNode node(nh);
    node.run();
    return 0;
}