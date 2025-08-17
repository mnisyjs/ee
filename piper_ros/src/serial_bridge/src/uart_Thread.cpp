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
        /* 从队列中获取正确的数据 */
        uint8_t aligned_data[uart_length] = {0};
        if (GetAlignedFromQueue(aligned_data) != -1)
        {
            /* 检查STM32发送的ASCII格式数据帧 */
            if (aligned_data[0] == 'S' &&
                aligned_data[1] == 'T' &&
                aligned_data[2] == 'M' &&
                aligned_data[3] == ':' &&
                aligned_data[104] == '>' &&
                aligned_data[105] == 'R' &&
                aligned_data[106] == 'O' &&
                aligned_data[107] == 'S' &&
                aligned_data[108] == '\r' &&
                aligned_data[109] == '\n')
            {
                /* 解析STM32发送的100字节数据 */
                // 假设数据结构为：odom_px, odom_py, odom_ang, odom_vx, odom_vy, odom_womoga, uwb_px, uwb_py, spe_m1-4, pos_m1-4
                float odom_px, odom_py, odom_ang, odom_vx, odom_vy, odom_womoga;
                float uwb_px, uwb_py;
                float spe_m1, spe_m2, spe_m3, spe_m4;
                float pos_m1, pos_m2, pos_m3, pos_m4;
                
                // 解析里程计数据 (前24字节)
                memcpy(&odom_px, &aligned_data[4], 4);
                memcpy(&odom_py, &aligned_data[8], 4);
                memcpy(&odom_ang, &aligned_data[12], 4);
                memcpy(&odom_vx, &aligned_data[16], 4);
                memcpy(&odom_vy, &aligned_data[20], 4);
                memcpy(&odom_womoga, &aligned_data[24], 4);
                
                // 解析UWB数据 (8字节)
                memcpy(&uwb_px, &aligned_data[28], 4);
                memcpy(&uwb_py, &aligned_data[32], 4);
                
                // 解析电机速度数据 (16字节)
                memcpy(&spe_m1, &aligned_data[36], 4);
                memcpy(&spe_m2, &aligned_data[40], 4);
                memcpy(&spe_m3, &aligned_data[44], 4);
                memcpy(&spe_m4, &aligned_data[48], 4);
                
                // 解析电机位置数据 (16字节)
                memcpy(&pos_m1, &aligned_data[52], 4);
                memcpy(&pos_m2, &aligned_data[56], 4);
                memcpy(&pos_m3, &aligned_data[60], 4);
                memcpy(&pos_m4, &aligned_data[64], 4);
                
                ROS_INFO("Received STM32 data: odom(%.3f,%.3f,%.3f) vel(%.3f,%.3f,%.3f) uwb(%.3f,%.3f)",
                         odom_px, odom_py, odom_ang, odom_vx, odom_vy, odom_womoga, uwb_px, uwb_py);
                
                // 发布里程计消息
                publishOdomMessage(odom_px, odom_py, odom_ang, odom_vx, odom_vy, odom_womoga);
            }
            else
            {
                ROS_WARN("Received invalid STM32 frame");
            }
        }

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

        /*尝试解析ASCII odom数据（兼容旧格式）*/
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
            }
        }

        /*从队列从获取正确的数据*/
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
 void Uart_Thread_Space::Lidar_Position(Uart_Thread *uart_ptr, float X, float Y, float Yaw)
 {
     // 将浮点数速度转换为STM32期望的单字节格式
     uint8_t vx_byte = static_cast<uint8_t>(128 - X * 128);
     uint8_t vy_byte = static_cast<uint8_t>(128 - Y * 128);
     uint8_t w_byte  = static_cast<uint8_t>(128 - Yaw * 128);
 
     printf("Mission1 Send: vx=%.2f->%d, vy=%.2f->%d, w=%.2f->%d\n", 
            X, vx_byte, Y, vy_byte, Yaw, w_byte);
 
     /* 设置包头 */
     uart_ptr->writeBuff[0] = 0x3F; // STM32期望的包头1
     uart_ptr->writeBuff[1] = 0x21; // STM32期望的包头2
     uart_ptr->writeBuff[2] = 0x01; // 任务ID
     
     /* 速度数据 */
     uart_ptr->writeBuff[3] = vx_byte; // X方向速度
     uart_ptr->writeBuff[4] = vy_byte; // Y方向速度
     uart_ptr->writeBuff[5] = w_byte;  // 角速度
     
     /* 未使用字节 */
     uart_ptr->writeBuff[6] = 0x00;    // 保留位
     
     /* 包尾 */
     uart_ptr->writeBuff[7] = 0x21;    // STM32期望的包尾
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

/**
 * @brief 发布里程计消息
 * @param px X位置
 * @param py Y位置
 * @param ang 角度
 * @param vx X速度
 * @param vy Y速度
 * @param w 角速度
 */
void Uart_Thread::publishOdomMessage(float px, float py, float ang, float vx, float vy, float w)
{
    static ros::NodeHandle nh;
    static ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    static tf::TransformBroadcaster odom_broadcaster;
    
    // 创建odom消息
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";
    
    // 设置位置
    odom_msg.pose.pose.position.x = px;
    odom_msg.pose.pose.position.y = py;
    odom_msg.pose.pose.position.z = 0.0;
    
    // 设置方向（yaw角转四元数）
    tf::Quaternion q;
    q.setRPY(0, 0, ang);
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();
    
    // 设置速度
    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = w;
    
    // 发布odom消息
    odom_pub.publish(odom_msg);
    
    // 发布tf变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = px;
    odom_trans.transform.translation.y = py;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation.x = q.x();
    odom_trans.transform.rotation.y = q.y();
    odom_trans.transform.rotation.z = q.z();
    odom_trans.transform.rotation.w = q.w();
    odom_broadcaster.sendTransform(odom_trans);
    
    ROS_DEBUG("Published STM32 odom: x=%.3f, y=%.3f, yaw=%.3f, vx=%.3f, vy=%.3f, w=%.3f", 
             px, py, ang, vx, vy, w);
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
