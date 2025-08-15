# 串口通信修复说明

## 问题描述

原始的ROS侧串口通信代码存在以下问题：

1. **数据帧长度不匹配**：ROS代码期望8字节数据，但STM32发送110字节
2. **协议格式不一致**：ROS代码试图解析二进制格式，但STM32发送ASCII格式
3. **数据解析逻辑错误**：无法正确解析STM32发送的数据结构

## 修复内容

### 1. 数据帧长度修正
- 将 `uart_length` 从8字节修改为110字节
- 匹配STM32发送的数据格式：`STM:` + 100字节数据 + `>ROS\r\n`

### 2. 协议格式统一
- 修正了数据帧头尾检测逻辑
- 支持STM32的ASCII格式数据：`STM:...>ROS\r\n`
- 保持ROS发送的二进制格式：`0x3F 0x21 0x01 [数据] 0x00 0x21`

### 3. 数据解析优化
- 添加了STM32数据结构的完整解析
- 支持里程计、UWB、电机速度/位置等数据
- 自动发布ROS里程计消息和TF变换

### 4. 新增功能
- `publishOdomMessage()` 函数：发布里程计消息
- 测试脚本：验证串口通信功能
- 错误处理和日志记录

## 数据格式说明

### STM32发送格式 (110字节)
```
STM: [100字节数据] >ROS\r\n
```

100字节数据结构：
- 字节 0-23: 里程计数据 (6个float，24字节)
  - odom_px, odom_py, odom_ang, odom_vx, odom_vy, odom_womoga
- 字节 24-31: UWB数据 (2个float，8字节)
  - uwb_px, uwb_py
- 字节 32-47: 电机速度 (4个float，16字节)
  - spe_m1, spe_m2, spe_m3, spe_m4
- 字节 48-63: 电机位置 (4个float，16字节)
  - pos_m1, pos_m2, pos_m3, pos_m4
- 字节 64-99: 保留字节 (36字节)

### ROS发送格式 (8字节)
```
0x3F 0x21 0x01 [vx] [vy] [w] 0x00 0x21
```

## 使用方法

### 1. 编译
```bash
cd ~/catkin_ws
catkin_make
```

### 2. 运行主节点
```bash
roslaunch serial_bridge camera_arm_control.launch
```

### 3. 运行测试脚本
```bash
rosrun serial_bridge test_serial_communication.py _port:=/dev/ttyUSB0
```

### 4. 发送测试指令
```bash
# 发布目标位置
rostopic pub /chassis/target_pose geometry_msgs/PoseStamped "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: 'map'
pose:
  position: {x: 1.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}" -r 1

# 紧急停止
rostopic pub /chassis/emergency_stop std_msgs/Bool "data: true"
```

## 参数配置

### 串口参数
- `com_name`: 串口设备名 (默认: `/dev/ttyUSB0`)
- `baudrate`: 波特率 (默认: 115200)
- `position_tolerance`: 位置容差 (默认: 0.1m)
- `orientation_tolerance`: 角度容差 (默认: 0.1rad)

### 测试参数
- `port`: 测试串口设备名
- `baudrate`: 测试波特率
- `test_interval`: 测试间隔 (默认: 5秒)

## 话题说明

### 订阅话题
- `/chassis/target_pose`: 目标位置 (geometry_msgs/PoseStamped)
- `/chassis/emergency_stop`: 紧急停止 (std_msgs/Bool)

### 发布话题
- `/chassis/arrival_status`: 到达状态 (std_msgs/Bool)
- `/chassis/status`: 小车状态 (std_msgs/String)
- `/odom`: 里程计信息 (nav_msgs/Odometry)

### TF变换
- `odom` → `base_link`: 里程计变换

## 故障排除

### 1. 串口连接失败
- 检查串口设备是否存在
- 确认串口权限设置
- 验证波特率配置

### 2. 数据接收异常
- 检查数据帧格式是否正确
- 确认STM32发送的数据长度
- 查看ROS日志输出

### 3. 运动控制异常
- 检查速度范围限制
- 确认PID参数设置
- 验证紧急停止功能

## 注意事项

1. **数据同步**：确保ROS和STM32的时钟同步
2. **错误处理**：添加适当的错误处理和恢复机制
3. **性能优化**：根据实际需求调整数据更新频率
4. **安全考虑**：确保紧急停止功能正常工作

## 版本历史

- v1.0: 初始版本，修复基本通信问题
- v1.1: 添加测试脚本和错误处理
- v1.2: 优化数据解析和发布逻辑
