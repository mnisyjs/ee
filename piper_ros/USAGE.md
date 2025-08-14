# 使用说明

## 系统启动

### 1. 启动完整系统
```bash
roslaunch serial_bridge camera_arm_control.launch
```

这将启动：
- MoveIt控制服务
- 串口桥接节点
- 机械臂控制节点

### 2. 测试串口通信
```bash
# 测试向STM32发送速度指令
roslaunch serial_bridge send_velocity_test.launch
```

## 参数配置

### 串口参数
- `com_name`: 串口设备名（默认：`/dev/ttyUSB0`）
- `baud_rate`: 波特率（默认：115200）

### 机械臂参数
- `position_tolerance`: 位置容差（默认：0.5m）
- `orientation_tolerance`: 角度容差（默认：0.3rad）
- `optimal_distance`: 最佳操作距离（默认：1.0m）

## 话题和服务

### 主要话题
- `/cmd_vel`: 速度控制指令
- `/chassis/target_pose`: 小车目标位置
- `/arm_control/status`: 机械臂控制状态

### 主要服务
- `/joint_moveit_ctrl_piper`: Piper机械臂控制服务
- `/joint_moveit_ctrl_gripper`: 夹爪控制服务

## 故障排除

### 常见问题
1. **串口权限问题**
   ```bash
   sudo chmod 666 /dev/ttyUSB0
   ```

2. **MoveIt服务不可用**
   - 确保MoveIt配置正确加载
   - 检查依赖包是否安装完整

3. **串口设备不存在**
   ```bash
   ls -l /dev/ttyUSB*
   ```

### 调试命令
```bash
# 检查节点状态
rosnode list

# 检查话题
rostopic list

# 检查服务
rosservice list

# 查看日志
roslaunch serial_bridge camera_arm_control.launch --screen
```
