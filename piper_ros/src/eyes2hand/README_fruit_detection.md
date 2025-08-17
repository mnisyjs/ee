# 果实检测系统使用说明

## 系统概述

这个果实检测系统实现了以下功能：
1. **果树位置管理**：从配置文件读取固定的果树位置信息
2. **果实检测模拟**：模拟相机检测绿色果实的过程
3. **手眼变换**：将相机检测到的果实位置转换为机械臂控制指令
4. **统一通信**：所有节点使用统一的话题名称和消息类型

## 系统架构

```
fruit_position_node (模拟相机) 
    ↓ 发布 /camera/targets
handeye_transform_node (手眼变换)
    ↓ 发布 /handeye/ik_result
arm_control_node (机械臂控制)
```

## 文件说明

### 核心节点
- `fruit_position_node.py`: 模拟相机检测，发布果实和果树位置
- `handeye_transform_node.py`: 手眼变换，计算逆运动学解
- `arm_control_node.cpp`: 机械臂控制主节点

### 配置文件
- `fruit_trees.yaml`: 果树位置配置文件
- `fruit_detection_system.launch`: 系统启动文件

### 消息类型
- `CameraTargets.msg`: 包含苹果和果树位置信息的消息

## 使用方法

### 1. 配置果树位置

编辑 `config/fruit_trees.yaml` 文件，设置果树的位置信息：

```yaml
fruit_trees:
  tree_1:
    id: 1
    name: "苹果树1"
    position:
      x: 2.0
      y: 1.5
      z: 0.0
    fruit_zone:
      min_height: 0.8
      max_height: 2.2
```

### 2. 启动系统

```bash
# 分别启动各个节点
rosrun eyes2hand fruit_position_node.py
rosrun eyes2hand handeye_transform_node.py
rosrun arm_control_node arm_control_node
```

### 3. 监控话题

```bash
# 查看相机目标话题
rostopic echo /camera/targets

# 查看手眼变换结果
rostopic echo /handeye/ik_result

# 查看机械臂状态
rostopic echo /arm_control/status
```

## 话题说明

| 话题名称 | 消息类型 | 发布者 | 订阅者 | 说明 |
|---------|---------|--------|--------|------|
| `/camera/targets` | `CameraTargets` | `fruit_position_node` | `handeye_transform_node`, `arm_control_node` | 相机检测到的目标信息 |
| `/handeye/ik_result` | `HandEyeIK` | `handeye_transform_node` | `arm_control_node` | 手眼变换和逆运动学结果 |
| `/arm_control/status` | `String` | `arm_control_node` | - | 机械臂控制状态 |

## 消息字段说明

### CameraTargets 消息
- `apple_pose`: 苹果的位置和姿态
- `tree_pose`: 果树的位置和姿态  
- `target_type`: 目标类型（"apple" 或 "tree"）
- `apple_id`: 苹果ID
- `tree_id`: 果树ID
- `confidence`: 检测置信度

## 自定义配置

### 修改检测频率
在启动文件中修改 `detection_rate` 参数：

```xml
<param name="detection_rate" value="0.5" />  <!-- 每2秒检测一次 -->
```

### 添加新的果树
在 `fruit_trees.yaml` 中添加新的果树配置：

```yaml
tree_4:
  id: 4
  name: "苹果树4"
  position:
    x: 6.0
    y: 3.5
    z: 0.0
  fruit_zone:
    min_height: 0.9
    max_height: 2.3
```

### 修改手眼变换矩阵
在 `handeye_transform_node.py` 中修改 `get_handeye_matrix()` 方法，返回实际的标定结果。

## 故障排除

### 常见问题

1. **配置文件加载失败**
   - 检查 `fruit_trees.yaml` 文件路径
   - 确保YAML格式正确

2. **话题通信失败**
   - 使用 `rostopic list` 检查话题是否存在
   - 使用 `rostopic info` 检查话题的发布者和订阅者

3. **手眼变换失败**
   - 检查手眼标定矩阵是否正确
   - 确认坐标系设置是否一致

### 调试命令

```bash
# 检查节点状态
rosnode list
rosnode info /fruit_position_node

# 检查话题状态
rostopic list
rostopic info /camera/targets

# 实时查看日志
rosrun rqt_console rqt_console
```

## 扩展功能

### 1. 真实相机集成
替换 `fruit_position_node.py` 中的模拟检测，集成真实的相机检测算法。

### 2. 多果实跟踪
扩展消息类型，支持同时检测和跟踪多个果实。

### 3. 动态果树管理
添加动态添加/删除果树的功能，支持运行时配置更新。

### 4. 视觉反馈
集成RViz可视化，实时显示检测到的果实和果树位置。
