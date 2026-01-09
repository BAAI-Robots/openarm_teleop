# OpenArm ROS2集成说明

本文档介绍如何使用OpenArm项目中新增的ROS2功能，将电机关节状态和末端执行器位姿发布到ROS2话题。

## 功能特性

新增的ROS2集成功能可以实时发布以下信息：

1. **关节状态** (`/leader/joint_states` 和 `/follower/joint_states`)
   - 关节位置 (position)
   - 关节速度 (velocity)
   - 关节力矩 (effort)

2. **末端执行器位姿** (`/leader/ee_pose` 和 `/follower/ee_pose`)
   - 位置 (x, y, z)
   - 姿态 (四元数表示)

## 修改内容总结

### 新增文件

1. **src/ros2_publisher.hpp** - ROS2发布器头文件
2. **src/ros2_publisher.cpp** - ROS2发布器实现文件

### 修改文件

1. **CMakeLists.txt** - 添加了ROS2依赖包
2. **control/openarm_unilateral_control.cpp** - 集成ROS2发布功能
3. **control/openarm_bilateral_control.cpp** - 集成ROS2发布功能

## 编译步骤

### 前置条件

确保已安装ROS2 Humble和相关依赖：

```bash
# 如果还没有安装ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# 安装额外的ROS2包
sudo apt install ros-humble-sensor-msgs \
                 ros-humble-geometry-msgs \
                 ros-humble-tf2 \
                 ros-humble-tf2-geometry-msgs
```

### 编译项目

```bash
# 进入项目目录
cd /home/robot/openarm_teleop

# 创建并进入build目录
mkdir -p build
cd build

# 配置cmake（需要source ROS2环境）
source /opt/ros/humble/setup.bash
cmake ..

# 编译
make -j$(nproc)
```

## 使用方法

### 1. 单边控制 (Unilateral Control)

```bash
# source ROS2环境
source /opt/ros/humble/setup.bash

# 运行单边控制
cd /home/robot/openarm_teleop
./build/unilateral_control <leader_urdf> <follower_urdf> [arm_side] [leader_can] [follower_can]

# 示例
./build/unilateral_control /tmp/openarm_urdf_gen/v10_leader.urdf \
                           /tmp/openarm_urdf_gen/v10_follower.urdf \
                           right_arm can0 can1
```

### 2. 双边控制 (Bilateral Control)

```bash
# source ROS2环境
source /opt/ros/humble/setup.bash

# 运行双边控制
cd /home/robot/openarm_teleop
./build/bilateral_control <leader_urdf> <follower_urdf> [arm_side] [leader_can] [follower_can]

# 示例
./build/bilateral_control /tmp/openarm_urdf_gen/v10_leader.urdf \
                          /tmp/openarm_urdf_gen/v10_follower.urdf \
                          right_arm can0 can1
```

## ROS2话题说明

### 查看发布的话题

在另一个终端运行：

```bash
# source ROS2环境
source /opt/ros/humble/setup.bash

# 查看所有话题
ros2 topic list

# 预期输出：
# /leader/joint_states
# /leader/ee_pose
# /follower/joint_states
# /follower/ee_pose
```

### 查看关节状态

```bash
# 查看leader的关节状态
ros2 topic echo /leader/joint_states

# 查看follower的关节状态
ros2 topic echo /follower/joint_states
```

输出格式：
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: leader_base_link
name:
- joint1
- joint2
- joint3
- joint4
- joint5
- joint6
- gripper_joint
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### 查看末端执行器位姿

```bash
# 查看leader的末端位姿
ros2 topic echo /leader/ee_pose

# 查看follower的末端位姿
ros2 topic echo /follower/ee_pose
```

输出格式：
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: leader_base_link
pose:
  position:
    x: 0.5
    y: 0.0
    z: 0.3
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

### 查看话题频率

```bash
# 查看发布频率（约50Hz）
ros2 topic hz /leader/joint_states
ros2 topic hz /leader/ee_pose
```

## 在RViz中可视化

可以使用RViz来可视化机器人状态：

```bash
# 启动RViz
source /opt/ros/humble/setup.bash
rviz2

# 在RViz中：
# 1. Add -> By topic -> /leader/joint_states -> JointState
# 2. Add -> By topic -> /leader/ee_pose -> Pose
# 3. 设置Fixed Frame为 "leader_base_link"
```

## 性能说明

- **控制频率**: 500 Hz (主控制循环)
- **ROS2发布频率**: 50 Hz (每10个控制周期发布一次)
- **延迟**: 极小（实时系统）

发布频率设置为50Hz是为了平衡实时性和ROS2系统负载。如需修改频率，可以调整代码中的 `publish_counter_` 阈值。

## 故障排除

### 问题1: 编译错误 - 找不到ROS2包

**解决方案**:
```bash
# 确保已source ROS2环境
source /opt/ros/humble/setup.bash

# 检查ROS2包是否安装
ros2 pkg list | grep sensor_msgs
ros2 pkg list | grep geometry_msgs
```

### 问题2: 运行时ROS2初始化失败

**解决方案**:
```bash
# 确保在运行程序前source了ROS2环境
source /opt/ros/humble/setup.bash

# 检查ROS2是否正常运行
ros2 topic list
```

### 问题3: 看不到话题

**解决方案**:
```bash
# 检查程序是否正常启动
# 应该看到类似输出：
# "ROS2 publisher initialized for leader"
# "ROS2 publisher initialized for follower"

# 在另一个终端检查话题
source /opt/ros/humble/setup.bash
ros2 topic list
```

### 问题4: 关节名称不匹配

如果您的机器人关节名称与代码中默认的不同，需要修改：

在 `control/openarm_unilateral_control.cpp` 和 `control/openarm_bilateral_control.cpp` 中：

```cpp
// 修改这两行以匹配您的机器人关节名称
std::vector<std::string> arm_joint_names = {"joint1", "joint2", "joint3",
                                            "joint4", "joint5", "joint6"};
std::vector<std::string> hand_joint_names = {"gripper_joint"};
```

## 代码架构

```
ROS2集成架构：

┌─────────────────────────────────────────┐
│   Control Thread (500Hz)                │
│   - unilateral_step() / bilateral_step()│
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│   LeaderArmThread / FollowerArmThread   │
│   - 获取电机状态                         │
│   - 计算末端位姿 (GetEECordinate)       │
│   - 每10次循环发布一次 (50Hz)           │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│   RobotStatePublisher                   │
│   - publish_joint_states()              │
│   - publish_ee_pose()                   │
└──────────────┬──────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│   ROS2 Topics                           │
│   - /leader/joint_states                │
│   - /leader/ee_pose                     │
│   - /follower/joint_states              │
│   - /follower/ee_pose                   │
└─────────────────────────────────────────┘
```

## 进一步开发

### 添加更多ROS2功能

如需添加更多ROS2功能，可以扩展 `RobotStatePublisher` 类：

1. 在 `src/ros2_publisher.hpp` 中添加新的发布器
2. 在 `src/ros2_publisher.cpp` 中实现发布逻辑
3. 在控制线程中调用新的发布方法

### 示例：添加力矩传感器数据发布

```cpp
// 在ros2_publisher.hpp中添加
void publish_wrench(const Eigen::Vector3d &force, const Eigen::Vector3d &torque);

// 在ros2_publisher.cpp中实现
void RobotStatePublisher::publish_wrench(const Eigen::Vector3d &force, 
                                         const Eigen::Vector3d &torque) {
    auto wrench_msg = geometry_msgs::msg::WrenchStamped();
    wrench_msg.header.stamp = node_->now();
    wrench_msg.wrench.force.x = force.x();
    wrench_msg.wrench.force.y = force.y();
    wrench_msg.wrench.force.z = force.z();
    wrench_msg.wrench.torque.x = torque.x();
    wrench_msg.wrench.torque.y = torque.y();
    wrench_msg.wrench.torque.z = torque.z();
    wrench_pub_->publish(wrench_msg);
}
```

## 参考资料

- [OpenArm官方文档](https://docs.openarm.dev/)
- [ROS2 Humble文档](https://docs.ros.org/en/humble/)
- [sensor_msgs/JointState](https://docs.ros2.org/latest/api/sensor_msgs/msg/JointState.html)
- [geometry_msgs/PoseStamped](https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html)

## 许可证

本代码遵循Apache License 2.0许可证。详见项目根目录的LICENSE.txt文件。

## 联系方式

如有问题，请联系：
- 邮箱: openarm@enactic.ai
- Discord: https://discord.gg/FsZaZ4z3We
