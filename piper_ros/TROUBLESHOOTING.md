# 故障排除指南

## 问题1：串口缓冲区数据持续输出

### 症状
```
readBuff: 53 54 4d 3a 0 0 82 42 0 0 0 0 0 0 0 0
readBuff: 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0
...
```

### 原因
1. **数据格式不匹配**：串口接收到的数据格式与期望的二进制帧格式不匹配
2. **帧同步失败**：无法找到正确的帧头（`? !`）和帧尾（`!`）
3. **STM数据干扰**：数据以 `STM:` 开头，可能是传感器数据

### 解决方案
1. **启用调试输出**：在 `uart_Thread.hpp` 中设置 `#define enable_show_read true`
2. **改进数据解析**：在 `uart_Thread.cpp` 中添加对STM格式数据的检测和跳过
3. **检查串口配置**：确保波特率、数据位、停止位等配置正确

### 修复后的代码
```cpp
// 在 uart_Thread.cpp 中
if (strstr(ascii_buffer, "STM:") != nullptr) {
    // 检测到STM格式数据，跳过二进制处理
    ROS_DEBUG("Detected STM format data, skipping binary processing");
    continue;
}
```

## 问题2：Piper MoveIt 服务不可用

### 症状
```
[WARN] [1755003512.199057164]: Piper moveit control service not available, retrying...
```

### 原因
1. **服务节点未启动**：`joint_moveit_ctrl_server.py` 没有运行
2. **MoveIt 配置问题**：MoveIt 配置未正确加载
3. **依赖服务缺失**：ROS master 或其他必要服务未启动

### 解决方案

#### 步骤1：启动 MoveIt 控制服务
```bash
# 方法1：直接启动服务节点
rosrun moveit_ctrl joint_moveit_ctrl_server.py

# 方法2：使用launch文件
roslaunch moveit_ctrl moveit_ctrl_server.launch
```

#### 步骤2：验证服务可用性
```bash
# 检查服务是否存在
rosservice list | grep joint_moveit_ctrl_piper

# 测试服务调用
rosservice call /joint_moveit_ctrl_piper "joint_states: [0.0,0.0,0.0,0.0,0.0,0.0]
gripper: 0.0
max_velocity: 0.1
max_acceleration: 0.1"
```

#### 步骤3：使用改进的启动文件
```bash
# 使用改进的launch文件，同时启动所有服务
roslaunch serial_bridge camera_arm_control_improved.launch
```

## 启动顺序建议

### 正确的启动顺序
1. **启动 ROS master**
   ```bash
   roscore
   ```

2. **启动 MoveIt 配置**
   ```bash
   roslaunch piper_moveit piper_no_gripper_moveit/launch/demo.launch
   ```

3. **启动 MoveIt 控制服务**
   ```bash
   roslaunch moveit_ctrl moveit_ctrl_server.launch
   ```

4. **启动串口桥接**
   ```bash
   roslaunch serial_bridge serial_bridge.launch
   ```

5. **启动机械臂控制节点**
   ```bash
   roslaunch arm_control_node arm_control_node.launch
   ```

### 一键启动（推荐）
```bash
roslaunch serial_bridge camera_arm_control_improved.launch
```

## 调试技巧

### 1. 检查节点状态
```bash
rosnode list
rosnode info /joint_moveit_ctrl_server
```

### 2. 检查话题
```bash
rostopic list
rostopic echo /joint_moveit_ctrl_piper
```

### 3. 检查服务
```bash
rosservice list
rosservice info /joint_moveit_ctrl_piper
```

### 4. 查看日志
```bash
roslaunch serial_bridge camera_arm_control_improved.launch --screen
```

## 常见错误及解决方案

### 错误1：ImportError: No module named 'moveit_commander'
**解决方案**：安装 MoveIt 依赖
```bash
sudo apt-get install ros-noetic-moveit-commander
```

### 错误2：Failed to initialize planning scene
**解决方案**：确保 MoveIt 配置正确加载
```bash
roslaunch piper_moveit piper_no_gripper_moveit/launch/demo.launch
```

### 错误3：Serial port not found
**解决方案**：检查串口设备权限和配置
```bash
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
```

## 联系支持
如果问题仍然存在，请：
1. 检查 ROS 版本兼容性
2. 查看完整的错误日志
3. 确认硬件连接正常
4. 验证所有依赖包已正确安装
