#include "serial_bridge/uart.hpp"

/**
 * 串口初始化，如果初始化失败，则会调用std::exit(1)退出程序
 * @param dev 打开串口名称
 * @return -1表示打开串口失败，其他表示打开成功
 */
int Uart::InitSerialPort(std::string dev)
{
    COUT_BLUE_START;
    printf("SerialPort Connecting ..\n");
    COUT_COLOR_END;

    OpenPort(dev);

    int speed = 115200; // 波特率
    int flow_ctrl = 0;  // 流控制
    int databits = 8;   // 数据位
    int stopbits = 1;   // 停止位
    SetUp(speed, flow_ctrl, databits, stopbits);

    if (fd == -1)
    {
        /*打开设备失败*/
        COUT_RED_START;
        std::cerr << "Error!! Open Port " << dev << " Failed!" << std::endl;
        std::cerr << "Closing the process!" << std::endl;
        COUT_COLOR_END;

        // 退出程序
        std::exit(1);
    }
    else
    {
        /*打开设备成功*/
        COUT_GREEN_START;
        std::cout << "Open Port " << dev << " Success!" << std::endl;
        COUT_COLOR_END;
    }

    return fd;
}

/**
 * @brief 读串口
 * 从串口中读取数据，并将读取到的数据存储在readBuff数组中
 */
int Uart::ReadBuffer()
{
    memset(readBuff, 0, sizeof(readBuff));
    return read(fd, &readBuff, uart_length);
}

/**
 * @brief 写串口
 */
int Uart::WriteBuffer()
{
    return write(fd, &writeBuff, uart_length);
}

/**
 * @brief 打印读到的串口
 */
void Uart::ShowReadBuff()
{
    printf("readBuff: ");
    for (size_t i = 0; i < uart_length; i++)
        printf("%x ", readBuff[i]);
    printf("\n");
}

/**
 * @brief 打印写的串口
 */
void Uart::ShowWriteBuff()
{
    printf("writeBuff: ");
    for (size_t i = 0; i < uart_length; i++)
        printf("%x ", writeBuff[i]);
    printf("\n");
}

/**
 * @brief 清空writeBuff并加上头尾帧
 */
void Uart::ClearWriteBuff()
{
    /*清空*/
    memset(writeBuff, 0, sizeof(writeBuff));

    /*头帧*/
    writeBuff[0] = '?';
    writeBuff[1] = '!';

    /*尾帧*/
    writeBuff[uart_length - 1] = '!';
}

/**
 * @brief 使用 select 函数堵塞,监听串口文件描述符的可读事件
 */
void Uart::Select()
{
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    select(fd + 1, &rfds, NULL, NULL, NULL);
}

/**
 * @brief 将收到的串口帧加入队列
 * 如果read_length为0，表示读取到的数据长度为整个readBuff数组的长度，即读取到了一批完整的数据帧。此时，将整个readBuff数组中的元素依次存入队列，以便后续处理。如果read_length不为0，表示读取到的数据长度为read_length，即读取到了部分数据帧。此时，将readBuff数组中前read_length个元素依次存入队列，以便后续处理
 */
void Uart::PushreadBuffToQueue(ssize_t read_length)
{
    if (read_length == 0)
    {
        for (size_t i = 0; i < uart_length; i++)
            readBuff_queue.push(readBuff[i]);
    }
    else
    {
        for (size_t i = 0; i < read_length; i++)
            readBuff_queue.push(readBuff[i]);
    }
}

/**
 * @brief 从队列中提取对齐好的数据
 * @param *pData 提取到的数据的数组指针
 * @return -1表示提取失败，其他表示提取成功
 */
int8_t Uart::GetAlignedFromQueue(uint8_t *pData)
{
    /*判断队列长度与数据帧长度*/
    while (uart_length <= readBuff_queue.size())
    {
        if (readBuff_queue[0] == '?' &&
            readBuff_queue[1] == '!' &&
            readBuff_queue[uart_length - 1] == '!')
        {
            /*队列中存在合法数据，赋值并返回*/
            for (uint8_t i = 0; i < uart_length; i++)
            {
                pData[i] = readBuff_queue.pop();
            }

            return 1;
        }
        else
        {
            /*队首的数据不合法，出队*/
            readBuff_queue.pop();
        }
    }

    return -1;
}

/**
 * @brief 打开串口并返回串口设备文件描述
 * @param dev 打开串口名称
 */
void Uart::OpenPort(std::string dev)
{
    const char *_dev = new char[32];
    _dev = dev.c_str();

    fd = open(_dev, O_RDWR | O_NDELAY); // 读写打开、使I/O变成非搁置模式
    if (fd == -1)
    {
        COUT_RED_START;
        printf("connect fail!\n");
        perror("Can't Open Serial Port");
        COUT_COLOR_END;
        return;
    }

    // 判断串口的状态是否为阻塞状态
    // fcntl()针对(文件)描述符提供控制.参数fd是被参数cmd操作，这里的参数指“设置文件状态标记“
    if (fcntl(fd, F_SETFL, 0) == -1)
    { //<0
        COUT_RED_START;
        perror("fcntl failed!");
        COUT_COLOR_END;
        return;
    }
    else
    {
        COUT_GREEN_START;
        printf("serial is available.\n");
        COUT_COLOR_END;
    }

    int DTR_flag;
    DTR_flag = TIOCM_DTR;           // TIOCM_DTR表示终端设备已经准备好
    ioctl(fd, TIOCMBIS, &DTR_flag); // Set RTS pin
    return;
}

/**
 * @brief 设置串口
 * @param speed 波特率
 * @param flow_ctrl 数据流控制方式
 * @param databits 数据位
 * @param stopbits 停止位
 */
void Uart::SetUp(int speed, int flow_ctrl, int databits, int stopbits)
{
    struct termios options;
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options，
    该函数还可以测试配置是否正确，该串口是否可用等。
    若调用成功，函数返回值为0，若调用失败，函数返回值为1。
    */
    if (tcgetattr(fd, &options))
    {
        perror("SetupSerial error 1 ");
        return;
    }

    /*波特率*/
    int speed_arr[] = {B9600, B19200, B115200, B1000000};
    int name_arr[] = {9600, 19200, 115200, 1000000};

    // 设置串口输入波特率和输出波特率
    for (size_t i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
    {
        if (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    // 修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    // 修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    // 设置数据流控制
    switch (flow_ctrl)
    {
    case 0:                          // 不使用流控制
        options.c_cflag &= ~CRTSCTS; // 默认
        break;
    case 1: // 使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2: // 使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }

    // 使用数据位掩码清空数据位设置
    options.c_cflag &= ~CSIZE;
    // 设置数据位
    switch (databits)
    {
    case 5:
        options.c_cflag |= CS5;
        break;
    case 6:
        options.c_cflag |= CS6;
        break;
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8; // 默认
        break;
    default:
        fprintf(stderr, "Unsupported data size\n");
        return;
    }

    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;
        break; // 默认
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr, "Unsupported stop bits\n");
        return;
    }

    // 修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

    // 设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1;  /* 读取字符的最少个数为1 */
    // 如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd, TCIFLUSH);

    // 激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        perror("serial parameter set error!");
        return;
    }
    return;
}
