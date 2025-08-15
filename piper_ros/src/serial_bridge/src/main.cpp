#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include "serial_bridge/pid_controller.h"
#include "serial_bridge/uart.hpp"
#include "serial_bridge/uart_Thread.hpp"
#include "serial_bridge/Queue_T.hpp"

// 小车运动控制节点
class ChassisControlNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_target_pose_;      // 订阅目标位置
    ros::Subscriber sub_emergency_stop_;   // 订阅紧急停止
    ros::Publisher pub_arrival_status_;    // 发布到位状态
    ros::Publisher pub_chassis_status_;    // 发布小车状态
    
    Uart_Thread uart_;
    
    // PID 控制器对象（用于精确位置控制）
    PIDController pid_x;
    PIDController pid_y;
    PIDController pid_yaw;
    
    // 状态变量
    bool is_moving_;
    bool is_emergency_stop_;
    geometry_msgs::PoseStamped current_target_;
    double position_tolerance_;  // 位置容差
    double orientation_tolerance_; // 角度容差
    
public:
    ChassisControlNode() : is_moving_(false), is_emergency_stop_(false), position_tolerance_(0.1), orientation_tolerance_(0.1), pid_x(1.0, 0.0, 0.05, -0.5, 0.5), pid_y(1.0, 0.0, 0.05, -0.5, 0.5), pid_yaw(2.0, 0.0, 0.1, -1.0, 1.0)
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
        sub_target_pose_ = nh_.subscribe("/chassis/target_pose", 10, &ChassisControlNode::targetPoseCallback, this);
        sub_emergency_stop_ = nh_.subscribe("/chassis/emergency_stop", 10, &ChassisControlNode::emergencyStopCallback, this);
        
        // 发布话题
        pub_arrival_status_ = nh_.advertise<std_msgs::Bool>("/chassis/arrival_status", 10);
        pub_chassis_status_ = nh_.advertise<std_msgs::String>("/chassis/status", 10);
        
        ROS_INFO("Chassis Control Node initialized with serial port: %s", com_name.c_str());
        publishStatus("ready");
    }
    
    ~ChassisControlNode() {
        uart_.Disable_Thread_Read_Uart();
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

        // 这里可以添加当前位置获取逻辑（如果有里程计的话）
        // 暂时使用简单的运动指令   
        
        // 限制速度范围在[-1.0, 1.0]之间
        target_x = std::max(-1.0, std::min(1.0, target_x));
        target_y = std::max(-1.0, std::min(1.0, target_y));
        target_yaw = std::max(-1.0, std::min(1.0, target_yaw));
    
        // 发送运动指令到串口
        uart_.Mission_Send(Uart_Thread_Space::Lidar_Position, &uart_, 
                          static_cast<float>(target_x), 
                          static_cast<float>(target_y), 
                          static_cast<float>(target_yaw));
        
        ROS_INFO("Sent movement command: vx=%.2f, vy=%.2f, vyaw=%.2f", target_x, target_y, target_yaw);
    }
 
    void stopMovement() {
        // 发送停止指令
        uart_.Mission_Send(Uart_Thread_Space::Lidar_Position, &uart_, 0.0f, 0.0f, 0.0f);
        is_moving_ = false;
        ROS_INFO("Movement stopped");
    }
    
    void checkArrivalStatus() {
        // 这里应该检查实际位置是否到达目标位置
        // 暂时模拟到达状态
        if (is_moving_) {
            // 模拟一段时间后到达
            static int check_count = 0;
            check_count++;
            if (check_count > 50) { // 大约5秒后到达
                is_moving_ = false;
                check_count = 0;
                
                std_msgs::Bool arrival_msg;
                arrival_msg.data = true;
                pub_arrival_status_.publish(arrival_msg);
                publishStatus("arrived");
                
                ROS_INFO("Target position reached!");
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

