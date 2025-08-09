# 增强版相机-机械臂-小车控制系统

## 系统架构升级

本系统在原有基础上进行了重要改进，现在支持**果树碰撞检测**和**智能位置规划**：

```
相机检测(苹果+果树) → 位置评估(碰撞检测) → 智能路径规划 → 小车运动 → 机械臂操作
```

## 主要改进

### 1. 新的消息类型 - `CameraTargets`

创建了自定义消息类型来同时传递苹果和果树信息：

```
# CameraTargets.msg
Header header
geometry_msgs/PoseStamped apple_pose  # 苹果位置
geometry_msgs/PoseStamped tree_pose   # 果树位置
string target_type                    # 目标类型
int32 apple_id                        # 苹果ID
int32 tree_id                         # 果树ID
float32 confidence                    # 检测置信度
```

### 2. 增强的位置评估 - `evaluatePosition`

**方法作用详解：**
- **获取当前位置**：通过里程计 (`/odom`) 获取小车实时位置
- **距离检查**：计算小车与苹果的距离，判断是否在机械臂最佳操作范围内
- **角度检查**：验证小车朝向是否适合机械臂够到目标
- **碰撞风险评估**：检查操作过程中是否会与果树发生碰撞
- **返回结果**：综合评估后返回位置是否合适

**代码示例：**
```cpp
bool ArmControlNode::evaluatePosition(const arm_control_node::CameraTargets::ConstPtr& targets) {
    // 1. 获取当前小车位置（通过里程计）
    double chassis_x = current_chassis_pose_.pose.position.x;
    double chassis_y = current_chassis_pose_.pose.position.y;
    double chassis_yaw = tf::getYaw(current_chassis_pose_.pose.orientation);
    
    // 2. 计算与苹果的距离和角度
    double distance_to_apple = sqrt(pow(apple_x - chassis_x, 2) + pow(apple_y - chassis_y, 2));
    double angle_to_apple = atan2(apple_y - chassis_y, apple_x - chassis_x);
    
    // 3. 检查距离和角度是否合适
    // 4. 检查碰撞风险
    // 5. 返回综合评估结果
}
```

### 3. 智能小车路径规划 - `planChassisMovement`

**改进逻辑：**
- **多候选位置生成**：围绕苹果生成8个候选位置（每45度一个）
- **碰撞风险检测**：为每个候选位置检查与果树的碰撞风险
- **最优位置选择**：选择距离果树最远且安全的位置
- **安全策略**：如果所有位置都有碰撞风险，则放弃当前苹果

**核心算法：**
```cpp
void ArmControlNode::planChassisMovement(const arm_control_node::CameraTargets::ConstPtr& targets) {
    // 1. 生成8个候选位置
    for (int i = 0; i < 8; ++i) {
        double angle = (2.0 * M_PI * i) / 8;
        // 计算候选位置
        candidate.x = apple_x - optimal_distance_ * cos(angle);
        candidate.y = apple_y - optimal_distance_ * sin(angle);
    }
    
    // 2. 选择最安全的位置
    for (each candidate) {
        if (!checkTreeCollisionRisk(apple, tree, candidate)) {
            // 计算与果树的距离，选择最远的
            if (tree_distance > max_tree_distance) {
                best_position = candidate;
            }
        }
    }
    
    // 3. 发布最佳位置或放弃苹果
}
```

### 4. 碰撞检测算法 - `checkTreeCollisionRisk`

**检测逻辑：**
- **距离检测**：检查小车到果树的距离是否小于到苹果的距离
- **路径检测**：检查机械臂操作路径是否会经过果树附近
- **安全余量**：考虑50cm安全余量，确保操作空间充足

**数学原理：**
```cpp
bool checkTreeCollisionRisk(apple_pos, tree_pos, chassis_pos) {
    double distance_to_apple = sqrt((apple_x - chassis_x)² + (apple_y - chassis_y)²);
    double distance_to_tree = sqrt((tree_x - chassis_x)² + (tree_y - chassis_y)²);
    
    // 检查1：果树距离过近
    if (distance_to_tree < distance_to_apple + safety_margin) {
        return true; // 有碰撞风险
    }
    
    // 检查2：果树在操作路径上
    double line_distance = |cross_product| / distance_to_apple;
    if (line_distance < safety_margin) {
        return true; // 有碰撞风险
    }
    
    return false; // 安全
}
```

## 话题变更

### 新话题
- `/camera/targets` (arm_control_node/CameraTargets) - 苹果+果树位置信息
- `/odom` (nav_msgs/Odometry) - 小车里程计信息

### 保持不变的话题
- `/chassis/target_pose` - 小车目标位置
- `/chassis/arrival_status` - 小车到位状态
- `/arm_control/status` - 机械臂任务状态

## 使用示例

### 1. 编译新消息
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. 启动系统
```bash
roslaunch serial_bridge camera_arm_control.launch
```

### 3. 监控话题
```bash
# 查看相机检测结果
rostopic echo /camera/targets

# 查看位置评估结果
rostopic echo /arm_control/status

# 查看小车位置
rostopic echo /odom
```

## 参数调整

### 位置评估参数
- `position_tolerance`: 位置容差（默认：0.5m）
- `orientation_tolerance`: 角度容差（默认：0.3rad）
- `optimal_distance`: 最佳操作距离（默认：1.0m）

### 安全参数
- `safety_margin`: 碰撞检测安全余量（硬编码：0.5m）
- `num_candidates`: 候选位置数量（硬编码：8个）

## 决策流程图

```
相机检测到苹果+果树
        ↓
    位置评估
   ├─ 距离合适？
   ├─ 角度合适？
   └─ 无碰撞风险？
        ↓
   [是] 直接操作
        ↓
   [否] 生成8个候选位置
        ↓
    碰撞风险检测
   ├─ 检查每个候选位置
   └─ 选择距果树最远的安全位置
        ↓
   [找到安全位置] 发布小车目标
        ↓
   [无安全位置] 放弃当前苹果
```

## 注意事项

1. **里程计要求**：确保 `/odom` 话题正常发布
2. **消息编译**：需要先编译 `arm_control_node` 包生成新消息
3. **参数调整**：根据实际机械臂工作空间调整参数
4. **安全优先**：系统优先考虑安全，会主动放弃有风险的苹果
5. **性能考虑**：碰撞检测算法已优化，实时性良好

## 扩展功能

- 可以增加更多候选位置（当前8个）
- 可以添加动态安全余量计算
- 可以集成更复杂的路径规划算法
- 可以添加多机械臂协调功能
