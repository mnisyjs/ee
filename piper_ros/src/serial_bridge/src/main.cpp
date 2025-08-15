#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include "serial_bridge/uart.hpp"
#include "serial_bridge/uart_Thread.hpp"
#include "serial_bridge/Queue_T.hpp"

// 小车运动控制节点
class ChassisControlNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_target_pose_;      // 订阅目标位置
    ros::Subscriber sub_emergency_stop_;   // 订阅紧急停止
    ros::Subscriber sub_odom_;             // 订阅里程计信息
    ros::Publisher pub_arrival_status_;    // 发布到位状态
    ros::Publisher pub_chassis_status_;    // 发布小车状态
    
    Uart_Thread uart_;
    
    // 状态变量
    bool is_moving_;
    bool is_emergency_stop_;
    bool has_odom_data_;                   // 是否收到过里程计数据
    geometry_msgs::PoseStamped current_target_;
    nav_msgs::Odometry current_odom_;      // 当前里程计信息
    
    // 容差参数
    double position_tolerance_;            // 位置容差
    double orientation_tolerance_;         // 角度容差
    
public:
    ChassisControlNode() : 
        is_moving_(false), 
        is_emergency_stop_(false),
        has_odom_data_(false),
        position_tolerance_(0.1), 
        orientation_tolerance_(0.1)
    {
        // 获取参数
        std::string com_name;
        nh_.param<std::string>("com_name", com_name, "/dev/ttyUSB0");
        nh_.param<double>("position_tolerance", position_tolerance_, 0.1);
        nh_.param<double>("orientation_tolerance", orientation_tolerance_, 0.1);
        
        // 初始化串口
        uart_.InitSerialPort(com_name);
        uart_.Enable_Thread_Read_Uart();
        
        // 订阅话题
        sub_target_pose_ = nh_.subscribe("/chassis/target_pose", 10, 
                                        &ChassisControlNode::targetPoseCallback, this);
        sub_emergency_stop_ = nh_.subscribe("/chassis/emergency_stop", 10, 
                                           &ChassisControlNode::emergencyStopCallback, this);
        sub_odom_ = nh_.subscribe("/odom", 10, 
                                  &ChassisControlNode::odomCallback, this);
        
        // 发布话题
        pub_arrival_status_ = nh_.advertise<std_msgs::Bool>("/chassis/arrival_status", 10);
        pub_chassis_status_ = nh_.advertise<std_msgs::String>("/chassis/status", 10);
        
        ROS_INFO("Chassis Control Node initialized with serial port: %s", com_name.c_str());
        publishStatus("ready");
    }
    
    ~ChassisControlNode() {
        uart_.Disable_Thread_Read_Uart();
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_odom_ = *msg;
        has_odom_data_ = true;
    }
    
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        if (is_emergency_stop_) {
            ROS_WARN("Emergency stop active, ignoring target pose");
            return;
        }
        
        ROS_INFO("Received target pose: x=%.2f, y=%.2f, yaw=%.2f", 
                 msg->pose.position.x, msg->pose.position.y, 
                 tf::getYaw(msg->pose.orientation));
        
        current_target_ = *msg;
        is_moving_ = true;
        publishStatus("moving");
        
        // 发送运动指令到串口
        sendMovementCommand(msg);
    }
    
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
        is_emergency_stop_ = msg->data;
        if (is_emergency_stop_) {
            ROS_WARN("Emergency stop activated!");
            stopMovement();
            publishStatus("emergency_stop");
        } else {
            ROS_INFO("Emergency stop deactivated");
            publishStatus("ready");
        }
    }
    
    void sendMovementCommand(const geometry_msgs::PoseStamped::ConstPtr& target) {
        // 计算目标位置和角度
        double target_x = target->pose.position.x;
        double target_y = target->pose.position.y;
        double target_yaw = tf::getYaw(target->pose.orientation);
        
        // 如果有当前位置信息，计算相对运动速度
        double vx = target_x;
        double vy = target_y;
        double vyaw = target_yaw;
        
        if (has_odom_data_) {
            // 获取当前位置和朝向
            double current_x = current_odom_.pose.pose.position.x;
            double current_y = current_odom_.pose.pose.position.y;
            double current_yaw = tf::getYaw(current_odom_.pose.pose.orientation);
            
            // 计算相对位置和角度
            vx = target_x - current_x;
            vy = target_y - current_y;
            vyaw = angles::shortest_angular_distance(current_yaw, target_yaw);
            
            ROS_DEBUG("Relative motion: vx=%.3f, vy=%.3f, vyaw=%.3f", vx, vy, vyaw);
        }
        
        // 限制速度范围在[-1.0, 1.0]之间
        vx = std::max(-1.0, std::min(1.0, vx));
        vy = std::max(-1.0, std::min(1.0, vy));
        vyaw = std::max(-1.0, std::min(1.0, vyaw));
    
        // 发送运动指令到串口
        uart_.Mission_Send(Uart_Thread_Space::Lidar_Position, &uart_, 
                          static_cast<float>(vx), 
                          static_cast<float>(vy), 
                          static_cast<float>(vyaw));
        
        ROS_INFO("Sent movement command: vx=%.2f, vy=%.2f, vyaw=%.2f", vx, vy, vyaw);
    }
 
    void stopMovement() {
        // 发送停止指令
        uart_.Mission_Send(Uart_Thread_Space::Lidar_Position, &uart_, 0.0f, 0.0f, 0.0f);
        is_moving_ = false;
        ROS_INFO("Movement stopped");
    }
    
    void checkArrivalStatus() {
        if (is_moving_ && has_odom_data_) {
            // 获取当前位置和朝向
            double current_x = current_odom_.pose.pose.position.x;
            double current_y = current_odom_.pose.pose.position.y;
            double current_yaw = tf::getYaw(current_odom_.pose.pose.orientation);
            
            // 获取目标位置和朝向
            double target_x = current_target_.pose.position.x;
            double target_y = current_target_.pose.position.y;
            double target_yaw = tf::getYaw(current_target_.pose.orientation);
            
            // 计算位置和角度偏差
            double dx = target_x - current_x;
            double dy = target_y - current_y;
            double dyaw = angles::shortest_angular_distance(current_yaw, target_yaw);
            
            double distance = sqrt(dx*dx + dy*dy);
            
            // 检查是否到达目标位置
            if (distance < position_tolerance_ && fabs(dyaw) < orientation_tolerance_) {
                is_moving_ = false;
                
                std_msgs::Bool arrival_msg;
                arrival_msg.data = true;
                pub_arrival_status_.publish(arrival_msg);
                publishStatus("arrived");
                
                ROS_INFO("Target position reached! Distance=%.3f, Angle diff=%.3f rad", 
                         distance, dyaw);
            }
        }
    }
    
    void publishStatus(const std::string& status) {
        std_msgs::String status_msg;
        status_msg.data = status;
        pub_chassis_status_.publish(status_msg);
    }
    
    void run() {
        ros::Rate rate(10); // 10Hz
        
        while (ros::ok()) {
            checkArrivalStatus();
            ros::spinOnce();
            rate.sleep();
        }
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "chassis_control_node");
    
    ChassisControlNode node;
    node.run();
    
    return 0;
}