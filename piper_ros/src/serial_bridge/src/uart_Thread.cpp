#include "serial_bridge/uart_Thread.hpp"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <cstring>

/**
 * @brief 读串口线程函数
 */
void Uart_Thread::Thread_Read_Uart()
{
    while (1)
    {
        /*是否退出线程*/
        if (this->flag_thread_read_uart == false)
        {
            break;
        }

        /*使用 select 函数堵塞，监听串口文件描述符的可读事件*/
        Select();

        /*堵塞读取串口*/
        ssize_t read_length = ReadBuffer();

#if enable_show_read
        printf("read length %ld\n", read_length);
#endif

        /*读取到串口后将数据送入队列*/
        PushreadBuffToQueue(read_length);

        /*尝试解析ASCII odom数据（优先处理）*/
        if (read_length > 0) {
            // 创建临时缓冲区来存储ASCII数据
            char ascii_buffer[256];
            memset(ascii_buffer, 0, sizeof(ascii_buffer));
            
            // 将读取的数据转换为ASCII字符串
            for (int i = 0; i < read_length && i < sizeof(ascii_buffer) - 1; i++) {
                ascii_buffer[i] = readBuff[i];
            }
            
            // 尝试解析odom数据
            if (parseOdomData(ascii_buffer)) {
                // 成功解析odom，继续处理二进制数据
                ROS_DEBUG("Successfully parsed odom data");
            } else if (strstr(ascii_buffer, "STM:") != nullptr) {
                // 检测到STM格式数据，跳过二进制处理
                ROS_DEBUG("Detected STM format data, skipping binary processing");
                continue;
            }
        }

        /*从队列从获取正确的数据*/
        uint8_t aligned_data[uart_length] = {0};
        if (GetAlignedFromQueue(aligned_data) != -1)
        {
            /*从队列中获取正确的数据成功*/
            if (aligned_data[2] == 0x01)
            {
                /*任务1*/
                uint32_t X = 0;
                memcpy(&X, &aligned_data[3], 4);
#if enable_show_read
                printf("X:%d\n", X);
                printf("Receive Mission 1:\n");
                ShowReadBuff();
#endif
            }
            else if (aligned_data[2] == 0x02)
            {
                /*任务2*/
                uint32_t X = 0;
                memcpy(&X, &aligned_data[4], 4);
#if enable_show_read
                printf("X:%d\n", X);
                printf("Receive Mission 2:\n");
                ShowReadBuff();
#endif
            }
        }
        else
        {
            /*从队列中获取正确的数据失败*/

            /*打印错误数据*/
#if enable_show_read
            COUT_RED_START;
            ShowReadBuff();
            COUT_COLOR_END;
#endif
        }
    }
}

/**
 * @brief 写串口线程函数
 */
void Uart_Thread::Thread_Write_Uart()
{
    uint32_t X = 0;
    while (1)
    {
        /*是否退出线程*/
        if (this->flag_thread_write_uart == false)
        {
            break;
        }

        /*清空写串口缓冲区*/
        ClearWriteBuff();

        /*为写串口缓冲区赋值*/
        writeBuff[2] = 0x01;

        X++;
        memcpy(&writeBuff[3], &X, 4);

        /*给写入串口进行上锁保护，并写入串口*/
        std::lock_guard<std::mutex> res_lock_write_uart(mutex_write_uart);
        WriteBuffer();

#if enable_show_write
        ShowWriteBuff();
#endif

        // 休眠100ms
        usleep(100000);
    }
}

/**
 * @brief 开启读串口线程
 */
void Uart_Thread::Enable_Thread_Read_Uart()
{
    flag_thread_read_uart = true;

    thread_read_uart = std::thread(&Uart_Thread::Thread_Read_Uart, this);
}

/**
 * @brief 开启写串口线程
 */
void Uart_Thread::Enable_Thread_Write_Uart()
{
    flag_thread_write_uart = true;

    thread_write_uart = std::thread(&Uart_Thread::Thread_Write_Uart, this);
}

/**
 * @brief 关闭读串口线程
 */
void Uart_Thread::Disable_Thread_Read_Uart()
{
    flag_thread_read_uart = false;
}

/**
 * @brief 关闭写串口线程
 */
void Uart_Thread::Disable_Thread_Write_Uart()
{
    flag_thread_write_uart = false;
}

/**
 * @brief 任务1赋值串口模板函数
 *
 * @param uart_ptr 赋值的串口
 * @param X 任务1执行完毕后的数据
 */
void Uart_Thread_Space::Lidar_Position(Uart_Thread *uart_ptr, float X,float Y, float Yaw)

{
    printf("Mission1 Send!\n");

    /*为写串口缓冲区赋值*/
    uart_ptr->writeBuff[2] = 0x01;
    memcpy(&uart_ptr->writeBuff[3], &X, 4);
    memcpy(&uart_ptr->writeBuff[7], &Y, 4);
    memcpy(&uart_ptr->writeBuff[11], &Yaw, 4); 
}

/**
 * @brief 任务2赋值串口模板函数
 *
 * @param uart_ptr 赋值的串口
 * @param X 任务2执行完毕后的数据
 * @param Y 任务2执行完毕后的数据
 */
void Uart_Thread_Space::Mission2_Assignment(Uart_Thread *uart_ptr, uint16_t X, float Y)
{
    printf("Mission2 Send!\n");

    /*为写串口缓冲区赋值*/
    uart_ptr->writeBuff[2] = 0x02;
    memcpy(&uart_ptr->writeBuff[3], &X, 2);
    memcpy(&uart_ptr->writeBuff[5], &Y, 4);
}

/**
 * @brief 解析ASCII odom数据
 * @param ascii_data ASCII字符串数据
 * @return true if successfully parsed odom data
 */

bool Uart_Thread::parseOdomData(const char* ascii_data)
{
    // 检查是否包含odom数据
    if (strstr(ascii_data, "odom:") == nullptr) {
        return false;
    }
    
    // 解析格式: "odom:%.3f;%.3f;%.3f;%.3f;%.3f;\r\n"
    // 对应: pos_x, pos_y, ang_rad, v_linear, v_angular
    float pos_x, pos_y, ang_rad, v_linear, v_angular;
    
    if (sscanf(ascii_data, "odom:%f;%f;%f;%f;%f;", &pos_x, &pos_y, &ang_rad, &v_linear, &v_angular) == 5) {
        // 成功解析，发布odom消息
        static ros::NodeHandle nh;
        static ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
        static tf::TransformBroadcaster odom_broadcaster;
        
        // 创建odom消息
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = ros::Time::now();
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // 设置位置
        odom_msg.pose.pose.position.x = pos_x;
        odom_msg.pose.pose.position.y = pos_y;
        odom_msg.pose.pose.position.z = 0.0;
        
        // 设置方向（yaw角转四元数）
        tf::Quaternion q;
        q.setRPY(0, 0, ang_rad);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        
        // 设置速度
        odom_msg.twist.twist.linear.x = v_linear;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.linear.z = 0.0;
        odom_msg.twist.twist.angular.x = 0.0;
        odom_msg.twist.twist.angular.y = 0.0;
        odom_msg.twist.twist.angular.z = v_angular;
        
        // 发布odom消息
        odom_pub.publish(odom_msg);
        
        // 发布tf变换
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = pos_x;
        odom_trans.transform.translation.y = pos_y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation.x = q.x();
        odom_trans.transform.rotation.y = q.y();
        odom_trans.transform.rotation.z = q.z();
        odom_trans.transform.rotation.w = q.w();
        odom_broadcaster.sendTransform(odom_trans);
        
        ROS_INFO("Published odom: x=%.3f, y=%.3f, yaw=%.3f, v=%.3f, w=%.3f", 
                 pos_x, pos_y, ang_rad, v_linear, v_angular);
        
        return true;
    }
    
    return false;
}

// 添加新的STM数据解析函数
bool Uart_Thread::parseSTMData(const uint8_t* data, size_t length)
{
    // 检查STM数据头
    if (length < 16 || memcmp(data, "STM:", 4) != 0) {
        return false;
    }
    
    // 解析STM数据格式（根据你的实际数据格式调整）
    // 假设格式: STM:pos_x(4字节)pos_y(4字节)ang_rad(4字节)v_linear(4字节)
    float pos_x, pos_y, ang_rad, v_linear;
    
    memcpy(&pos_x, data + 4, 4);
    memcpy(&pos_y, data + 8, 4);
    memcpy(&ang_rad, data + 12, 4);
    memcpy(&v_linear, data + 16, 4);
    
    // 发布odom消息（复用现有逻辑）
    static ros::NodeHandle nh;
    static ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    static tf::TransformBroadcaster odom_broadcaster;
    
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    odom_msg.pose.pose.position.x = pos_x;
    odom_msg.pose.pose.position.y = pos_y;
    odom_msg.pose.pose.position.z = 0.0;
    
    tf::Quaternion q;
    q.setRPY(0, 0, ang_rad);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    odom_msg.twist.twist.linear.x = v_linear;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = 0.0;
    
    odom_pub.publish(odom_msg);
    
    // 发布tf变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = pos_x;
    odom_trans.transform.translation.y = pos_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();
    odom_broadcaster.sendTransform(odom_trans);
    
    ROS_INFO("Published STM odom: x=%.3f, y=%.3f, yaw=%.3f, v=%.3f", 
             pos_x, pos_y, ang_rad, v_linear);
    
    return true;
}
