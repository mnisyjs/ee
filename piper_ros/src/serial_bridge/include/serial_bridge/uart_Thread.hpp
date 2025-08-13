#ifndef __UART_THREAD_HPP
#define __UART_THREAD_HPP

#include "uart.hpp"

#include <thread> /*多线程*/
#include <mutex>  /*互斥锁*/

class Uart_Thread : public Uart
{
private:
/*是否显示原始读串口帧数据*/
#define enable_show_read true
/*是否显示原始写串口帧数据*/
#define enable_show_write true

    /*读串口线程*/
    std::thread thread_read_uart;
    /*写串口线程*/
    std::thread thread_write_uart;

    /*写串口线程锁*/
    std::mutex mutex_write_uart;

    /*是否开启读串口线程*/
    volatile bool flag_thread_read_uart = false;
    /*是否开启写串口线程*/
    volatile bool flag_thread_write_uart = false;

    /**
     * @brief 读串口线程函数
     */
    void Thread_Read_Uart();

    /**
     * @brief 写串口线程函数
     */
    void Thread_Write_Uart();

public:
    /**
     * @brief 开启读串口线程
     */
    void Enable_Thread_Read_Uart();

    /**
     * @brief 开启写串口线程
     */
    void Enable_Thread_Write_Uart();

    /**
     * @brief 关闭读串口线程
     */
    void Disable_Thread_Read_Uart();

    /**
     * @brief 关闭写串口线程
     */
    void Disable_Thread_Write_Uart();

    /**
     * @brief 解析ASCII odom数据
     * @param ascii_data ASCII字符串数据
     * @return true if successfully parsed odom data
     */
    bool parseOdomData(const char* ascii_data);

    /**
     * @brief 解析STM二进制数据
     * @param data 二进制数据
     * @param length 数据长度
     * @return true if successfully parsed STM data
     */
    bool parseSTMData(const uint8_t* data, size_t length);
 
    /**
     * @brief 任务发送串口模板函数
     * @note 模板函数不能在.h或.hpp中定义，再在.cpp中实现
     *
     * @tparam Args 函数指针的参数
     * @param Assignment_Func 为writeBuff赋值的函数指针
     * @param uart_ptr 任务发送串口
     * @param args Assignment_Func的参数
     */
    template <typename... Args>
    void Mission_Send(void (*Assignment_Func)(Uart_Thread *, Args...), Uart_Thread *uart_ptr, Args... args)
    {
        /*给写入串口进行上锁保护*/
        std::lock_guard<std::mutex> res_lock_write_uart(mutex_write_uart);

        /*清空写串口缓冲区*/
        ClearWriteBuff();

        /*为写串口缓冲区赋值*/
        Assignment_Func(uart_ptr, args...);

        /*写入串口*/
        WriteBuffer();

#if enable_show_write
        printf("Mission Send:");
        ShowWriteBuff();
#endif
    }
};

/**
 * @brief 存放一些任务发送串口线要用到的函数
 *
 */
namespace Uart_Thread_Space
{
    /**
     * 对于Mission_Send函数所接收到的函数指针Assignment_Func，有以下要求：
     * 1. 第一个形参必须为Uart_Thread *uart_ptr
     * 2. 后续形参不指定数量和类型
     * 具体用法可以参考以下几个函数
     */

    /**
     * @brief 任务1赋值串口模板函数
     *
     * @param uart_ptr 赋值的串口
     * @param X 任务1执行完毕后的数据
     */
    void Lidar_Position(Uart_Thread *uart_ptr, float X, float Y, float Z);

    /**
     * @brief 任务2赋值串口模板函数
     *
     * @param uart_ptr 赋值的串口
     * @param X 任务2执行完毕后的数据
     * @param Y 任务2执行完毕后的数据
     */
    void Mission2_Assignment(Uart_Thread *uart_ptr, uint16_t X, float Y);
};

#endif
