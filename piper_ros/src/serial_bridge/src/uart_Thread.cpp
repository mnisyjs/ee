#include "serial_bridge/uart_Thread.hpp"

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
            COUT_RED_START;
            ShowReadBuff();
            COUT_COLOR_END;
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
