## 项目概述

本仓库包含一套基于 ROS Noetic 的移动底盘 + Piper 机械臂集成系统，涵盖真实设备控制、MoveIt 规划、Gazebo/MuJoCo 仿真、手眼标定/逆解与串口桥接等功能。

- **ROS 版本**: Noetic（Ubuntu 20.04）
- **主要功能包根目录**: `piper_ros/src`
- **真实臂控制入口**: `piper_ros/src/piper`
- **MoveIt 规划**: `piper_ros/src/piper_moveit`
- **仿真**: `piper_ros/src/piper_sim`
- **URDF/模型**: `piper_ros/src/piper_description`
- **手眼模块**: `piper_ros/src/eyes2hand`
- **臂-小车一体流程示例**: `piper_ros/src/serial_bridge/launch/camera_arm_control.launch`

竞赛用途，请勿对外传播或商用。

### 构建顺序重要说明（编译依赖修复）

为解决编译时 `arm_control_node.h` 需要 `HandEyeIK.h` 的问题，建议首次编译前在 `piper_ros/src/arm_control_node` 下新建空文件 `CATKIN_IGNORE`，先完成其他包的编译；随后删除该文件并再次编译整个工作空间。

---

## 环境与先决条件

- Ubuntu 20.04 + ROS Noetic（不建议在 Windows 原生环境直接构建；如需在 Windows 使用，请通过 WSL 或远程 Ubuntu 主机）
- 已安装 `git`、`build-essential` 等基础工具

### ROS 与依赖安装

```bash
source /opt/ros/noetic/setup.bash
sudo apt update
sudo apt install -y python3-wstool python3-catkin-tools python3-rosdep \
    ros-noetic-ruckig \
    ros-noetic-eigen-stl-containers ros-noetic-geometric-shapes ros-noetic-pybind11-catkin \
    ros-noetic-moveit-resources-panda-moveit-config ros-noetic-ompl ros-noetic-warehouse-ros \
    ros-noetic-eigenpy ros-noetic-rosparam-shortcuts ros-noetic-moveit-msgs ros-noetic-srdfdom

# Python 依赖（CAN 与 Piper SDK）
pip3 install -U python-can piper_sdk
```

> 如遇 MoveIt 源码编译困难，可直接使用官方二进制包：
> `sudo apt install ros-$ROS_DISTRO-moveit`。
> 然后在 `piper_ros/src/piper_moveit/moveit-1.1.11` 路径下创建空文件 `CATKIN_IGNORE`（或直接删除该目录），清理 `build/` 与 `devel/` 后重新编译。

---

## 获取与构建

```bash
git clone <你的仓库地址>
cd ee/piper_ros
catkin_make
source devel/setup.bash
```

> 首次编译建议按“构建顺序重要说明”操作，避免 `HandEyeIK.h` 先后依赖问题。

---

## 硬件准备与 CAN 激活（真实机械臂）

真实机械臂控制基于 USB-CAN。请先在 `piper_ros` 根目录下执行脚本完成 CAN 设备激活。

### 单个 USB-CAN 设备

```bash
cd ee/piper_ros
bash can_activate.sh can0 1000000
```

### 多个 USB-CAN 设备

```bash
cd ee/piper_ros
sudo ethtool -i can0 | grep bus        # 记录 bus-info，如 1-2:1.0
bash find_all_can_port.sh              # 如需确认 USB 地址对应的 can 名称
bash can_activate.sh can_piper 1000000 "1-2:1.0"
```

执行 `ifconfig` 确认已出现 `can_piper`（或你的命名）。

---

## 快速开始

### 1) 启动单臂控制节点（真实机械臂）

```bash
cd ee/piper_ros
source devel/setup.bash
roslaunch piper start_single_piper.launch can_port:=can0 auto_enable:=true
# 或带 RViz 的组合展示
roslaunch piper start_single_piper_rviz.launch can_port:=can0 auto_enable:=true
```

常用参数（在 `piper_ros/src/piper/launch` 中定义）：

- `can_port`: CAN 口名称（如 `can0` 或上文重命名的 `can_piper`）
- `auto_enable`: 是否开机自动使能
- `gripper_val_mutiple`: 夹爪比例因子（RViz joint7 范围 0~0.04，对应实际 0~0.08m，通常取 2）

节点启动后，你将获得如下主题与服务（节选）：

- 主题：`/joint_states`（下发关节控制）、`/joint_states_single`（反馈）、`/pos_cmd`（末端位姿控制）、`/arm_status`、`/end_pose`、`/end_pose_euler`
- 服务：`/enable_srv`、`/stop_srv`、`/reset_srv`、`/go_zero_srv`、`/gripper_srv`

示例：

```bash
# 使能
rosservice call /enable_srv "enable_request: true"
# 停止
rosservice call /stop_srv
# 复位（掉电下垂，注意安全）
rosservice call /reset_srv
# 回零（非 MIT 模式）
rosservice call /go_zero_srv "is_mit_mode: false"
# 发送关节位置（注意安全范围与周边环境）
rostopic pub /joint_states sensor_msgs/JointState "name: [''] \
position: [0.2,0.2,-0.2,0.3,-0.2,0.5,0.01] \
velocity: [0,0,0,0,0,0,10] \
effort:   [0,0,0,0,0,0,0.5]" -1
```

### 2) MoveIt 交互与服务

```bash
# 先启动控制节点（推荐夹爪倍数为 2）
roslaunch piper start_single_piper.launch gripper_val_mutiple:=2

# 再启动 MoveIt（有夹爪）
roslaunch piper_with_gripper_moveit demo.launch
# 或无 RViz：use_rviz:=false
```

服务调用示例（由 `moveit_ctrl` 提供）：

```bash
# 关节控制
rosservice call /joint_moveit_ctrl_arm "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5] \
max_velocity: 0.5 max_acceleration: 0.5"

# 末端位姿（四元数或欧拉角）
rosservice call /joint_moveit_ctrl_endpose "joint_endpose: [x,y,z,qx,qy,qz,qw] \
max_velocity: 0.5 max_acceleration: 0.5"

# 夹爪控制（0~0.035，对应实际 0~0.07m）
rosservice call /joint_moveit_ctrl_gripper "gripper: 0.035 \
max_velocity: 0.5 max_acceleration: 0.5"

# 机械臂+夹爪联合
rosservice call /joint_moveit_ctrl_piper "joint_states: [0.2,0.2,-0.2,0.3,-0.2,0.5] \
gripper: 0.035 max_velocity: 0.5 max_acceleration: 0.5"
```

### 3) 一键联调示例（底盘 + 机械臂 + MoveIt + 可选手眼）

使用集成的示例启动：`piper_ros/src/serial_bridge/launch/camera_arm_control.launch`

```bash
roslaunch serial_bridge camera_arm_control.launch
```

该启动包含：

- 底盘控制节点 `serial_bridge`（串口 `/dev/ttyUSB0`，可按需修改）
- 机械臂控制节点 `arm_control_node`（订阅视觉与手眼结果，调用 MoveIt 服务）
- MoveIt 控制服务 `moveit_ctrl/joint_moveit_ctrl_server.py`
- 可选：手眼变换节点 `eyes2hand/handeye_transform_node.py`（默认关闭，可改 `if` 条件开启）

> `arm_control_node` 的内部状态机会综合相机目标、底盘到位与障碍风险，调用服务执行抓取与投放流程。

---

## 仿真

### Gazebo（有/无夹爪）

```bash
cd ee/piper_ros && source devel/setup.bash
roslaunch piper_gazebo piper_gazebo.launch                # 有夹爪
roslaunch piper_gazebo piper_no_gripper_gazebo.launch     # 无夹爪

# 使用 RViz GUI 进行关节控制
roslaunch piper_description display_urdf.launch           # 有夹爪
roslaunch piper_description display_no_gripper_urdf.launch# 无夹爪
```

Gazebo 控制主题（节选）：`/joint_states`、`/gazebo/jointX_position_controller/command`。

### MuJoCo（有/无夹爪）

请按 `piper_ros/src/piper_sim/README.md` 完成 MuJoCo 与 `mujoco-py` 安装后运行：

```bash
roslaunch piper_mujoco piper_mujoco.launch                # 有夹爪
roslaunch piper_mujoco piper_no_gripper_mujoco.launch     # 无夹爪
```

---

## 手眼标定与逆解（可选）

- 手眼变换与逆解示例：`piper_ros/src/eyes2hand/scripts/handeye_transform_node.py`
- Pinocchio 逆解（独立环境）：`piper_ros/src/piper/scripts/piper_pinocchio/README.MD`

如需开启手眼节点，请在你的启动文件中加入：

```xml
<node name="handeye_transform_node" pkg="eyes2hand" type="handeye_transform_node.py" output="screen" />
```

并根据标定结果替换节点中的手眼外参矩阵。

---

## URDF 与固件版本对应

固件 S-V1.6-3 版本前后 DH 参数存在 2° 差异（j2/j3 坐标系偏移）。当前默认使用新 URDF。

- 旧固件（< S-V1.6-3）：`piper_description_old.urdf`
- 新固件（>= S-V1.6-3）：`piper_description.urdf`

---

## 常见问题（FAQ）

- **无法使能，CAN 报错**
  - 现象：
    ```
    使能状态: False
    <class 'can.exceptions.CanError'> Message NOT sent
    ```
  - 排查：检查绿端子接线、重新插拔 USB-CAN、重启机械臂电源，重新执行 CAN 激活脚本。

- **MoveIt 编译失败**
  - 采用二进制安装 MoveIt，并对 `piper_moveit/moveit-1.1.11` 目录放置 `CATKIN_IGNORE` 后重编译。

- **构建依赖顺序问题（HandEyeIK）**
  - 首次构建前在 `arm_control_node` 下创建 `CATKIN_IGNORE`，编译完成后删除并再次编译。

- **RViz 与夹爪行程不一致**
  - RViz 中 `joint7` 范围 0~0.04，对应实际 0~0.08m。请将参数 `gripper_val_mutiple` 设为 2。

更多细节请参见：

- `piper_ros/README.MD`
- `piper_ros/src/piper_moveit/README.md`
- `piper_ros/src/piper_sim/README.md`
- `piper_ros/Q&A.MD`

---

## 许可证

详见 `piper_ros/LICENSE`。

