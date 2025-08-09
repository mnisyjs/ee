# 相机-机械臂-小车控制系统

## 系统架构

本系统实现了基于相机检测的智能采摘控制流程：

```
相机检测目标 → 机械臂节点判断位置 → 小车运动控制 → 机械臂执行操作
```

### 节点组成

1. **camera_simulator** - 相机模拟节点
   - 发布话题：`/camera/target_pose` (geometry_msgs/PoseStamped)
   - 功能：模拟相机检测到的目标点位

2. **arm_control_node** - 机械臂控制节点
   - 订阅话题：
     - `/camera/target_pose` - 相机目标点位
     - `/chassis/arrival_status` - 小车到位状态
     - `/chassis/status` - 小车状态
     - `/camera_obstacle` - 障碍检测
   - 发布话题：
     - `/chassis/target_pose` - 小车目标位置
     - `/chassis/emergency_stop` - 紧急停止
     - `/arm_control/status` - 机械臂状态
   - 功能：评估位置、规划小车运动、控制机械臂操作

3. **chassis_control_node** - 小车控制节点
   - 订阅话题：
     - `/chassis/target_pose` - 目标位置
     - `/chassis/emergency_stop` - 紧急停止
   - 发布话题：
     - `/chassis/arrival_status` - 到位状态
     - `/chassis/status` - 小车状态
   - 功能：通过串口控制小车运动

4. **joint_moveit_ctrl_server** - MoveIt控制服务器
   - 提供机械臂运动控制服务
   - 支持piper规划组

## 控制流程

### 1. 目标检测
- 相机检测到目标点位
- 发布到 `/camera/target_pose` 话题

### 2. 位置评估
- 机械臂节点接收目标点位
- 评估当前位置是否适合操作
- 如果位置不合适，计算适合的小车位置

### 3. 小车运动
- 发布小车目标位置到 `/chassis/target_pose`
- 小车控制节点通过串口发送运动指令
- 等待小车到达目标位置

### 4. 机械臂操作
- 小车到位后，机械臂节点执行采摘操作
- 通过MoveIt服务控制机械臂运动
- 完成采摘、放置等任务

## 启动方法

### 使用launch文件启动
```bash
roslaunch serial_bridge camera_arm_control.launch
```

### 手动启动各个节点
```bash
# 终端1：启动相机模拟
rosrun serial_bridge camera_simulator.py

# 终端2：启动小车控制
rosrun serial_bridge main

# 终端3：启动机械臂控制
rosrun arm_control_node arm_control_node

# 终端4：启动MoveIt控制服务器
rosrun moveit_ctrl joint_moveit_ctrl_server.py
```

## 参数配置

### 机械臂控制节点参数
- `position_tolerance`: 位置容差 (默认: 0.5m)
- `orientation_tolerance`: 角度容差 (默认: 0.3rad)
- `optimal_distance`: 最佳操作距离 (默认: 1.0m)

### 小车控制节点参数
- `com_name`: 串口设备名 (默认: /dev/ttyUSB0)
- `position_tolerance`: 位置容差 (默认: 0.1m)
- `orientation_tolerance`: 角度容差 (默认: 0.1rad)

## 话题说明

### 输入话题
- `/camera/target_pose`: 相机检测到的目标点位
- `/camera_obstacle`: 障碍检测结果
- `/handeye/ik_result`: 手眼标定结果（可选）

### 输出话题
- `/chassis/target_pose`: 小车目标位置
- `/chassis/emergency_stop`: 紧急停止指令
- `/arm_control/status`: 机械臂任务状态

### 状态话题
- `/chassis/arrival_status`: 小车到位状态
- `/chassis/status`: 小车运行状态

## 状态机

机械臂控制节点包含以下状态：
- `IDLE`: 空闲状态
- `EVALUATE_POSITION`: 评估位置
- `MOVE_CHASSIS`: 移动小车
- `WAIT_FOR_CHASSIS`: 等待小车到位
- `EXECUTE_ARM_OPERATION`: 执行机械臂操作
- `MOVE_TO_FRUIT`: 移动到水果位置
- `CUT_FRUIT`: 剪断水果
- `RETRACT`: 撤回机械臂
- `MOVE_TO_BASKET`: 移动到果篮
- `PLACE_FRUIT`: 放置水果
- `EMERGENCY_STOP`: 紧急停止

## 注意事项

1. 确保串口设备权限正确
2. 确保MoveIt配置正确
3. 根据实际硬件调整参数
4. 可以替换相机模拟节点为真实的相机节点
5. 可以添加更多的安全检查机制 