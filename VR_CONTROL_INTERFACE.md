# VR控制接口文档

## 概述

本文档介绍OpenArm的VR控制接口。该接口提供了完整的ROS2话题订阅和发布功能，允许VR控制器通过标准ROS2消息控制机器人。

## 功能特性

### 1. 订阅的控制命令话题

VR控制器可以通过以下话题发送命令：

#### 关节控制命令
- **话题**: `/robot/joint_command`
- **消息类型**: `sensor_msgs/msg/JointState`
- **说明**: 直接控制机器人关节位置
- **消息格式**:
```yaml
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_joint']
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 关节角度 (弧度)
velocity: []  # 可选
effort: []    # 可选
```

#### 末端位姿控制命令
- **话题**: `/robot/ee_pose_command`
- **消息类型**: `geometry_msgs/msg/PoseStamped`
- **说明**: 控制机器人末端执行器的位置和姿态
- **注意**: ⚠️ **需要逆运动学求解器**（当前未实现）
- **消息格式**:
```yaml
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'base_link'
pose:
  position:
    x: 0.5
    y: 0.0
    z: 0.3
  orientation:  # 四元数
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0
```

#### 夹爪控制命令
- **话题**: `/robot/gripper_command`
- **消息类型**: `std_msgs/msg/Float64MultiArray`
- **说明**: 控制夹爪开合
- **消息格式**:
```yaml
data: [0.0]  # 夹爪位置，范围通常 0.0(闭合) ~ 1.0(打开)
```

### 2. 发布的状态话题

机器人会实时发布以下状态信息：

#### 关节状态
- **话题**: `/robot/joint_states`
- **消息类型**: `sensor_msgs/msg/JointState`
- **频率**: 50 Hz
- **说明**: 当前所有关节的位置、速度、力矩

#### 末端执行器位姿
- **话题**: `/robot/ee_pose`
- **消息类型**: `geometry_msgs/msg/PoseStamped`
- **频率**: 50 Hz
- **说明**: 末端执行器的当前位置和姿态

## 代码接口

### 现成代码

以下代码已经实现并可直接使用：

1. **vr_control_interface.hpp/.cpp** - VR控制接口类
2. **ros2_publisher.hpp/.cpp** - ROS2状态发布器
3. **vr_control_example.cpp** - 完整的使用示例

### 核心类：VRControlInterface

```cpp
#include <vr_control_interface.hpp>

// 创建VR控制接口
auto vr_interface = std::make_shared<openarm::ros2::VRControlInterface>(
    "vr_control_node", "robot");

// 初始化
vr_interface->init(argc, argv);

// 设置关节命令回调
vr_interface->set_joint_command_callback(
    [](const std::vector<double>& joint_positions) {
        // 处理关节命令
        // joint_positions包含所有关节的目标位置
    }
);

// 设置末端位姿命令回调
vr_interface->set_ee_pose_command_callback(
    [](const Eigen::Vector3d& position, 
       const Eigen::Quaterniond& orientation) {
        // 处理末端位姿命令
        // ⚠️ 需要实现逆运动学求解
    }
);

// 设置夹爪命令回调
vr_interface->set_gripper_command_callback(
    [](double gripper_value) {
        // 处理夹爪命令
    }
);

// 在主循环中处理回调
while (running) {
    vr_interface->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
}
```

## 编译和运行

### 编译

```bash
cd /home/robot/openarm_teleop/build
source /opt/ros/humble/setup.bash
cmake .. && make -j$(nproc)
```

### 运行VR控制示例

```bash
# 生成URDF（如果还没有）
source ~/ros2_ws/install/setup.bash
mkdir -p /tmp/openarm_urdf_gen
xacro ~/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro \
      bimanual:=true -o /tmp/openarm_urdf_gen/v10_leader.urdf

# 运行VR控制程序
cd /home/robot/openarm_teleop
./build/vr_control_example can0 /tmp/openarm_urdf_gen/v10_leader.urdf right_arm
```

## 测试VR接口

### 发送关节命令

在另一个终端测试关节控制：

```bash
source /opt/ros/humble/setup.bash

# 发送关节命令（所有关节回零位）
ros2 topic pub --once /robot/joint_command sensor_msgs/msg/JointState \
"{
  name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_joint'],
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}"
```

### 发送末端位姿命令

```bash
# 发送末端位姿命令（注意：需要实现逆运动学）
ros2 topic pub --once /robot/ee_pose_command geometry_msgs/msg/PoseStamped \
"{
  header: {frame_id: 'base_link'},
  pose: {
    position: {x: 0.5, y: 0.0, z: 0.3},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"
```

### 发送夹爪命令

```bash
# 发送夹爪命令
ros2 topic pub --once /robot/gripper_command std_msgs/msg/Float64MultiArray \
"{data: [0.5]}"
```

### 查看机器人状态

```bash
# 查看关节状态
ros2 topic echo /robot/joint_states

# 查看末端位姿
ros2 topic echo /robot/ee_pose

# 查看话题频率
ros2 topic hz /robot/joint_states
```

## VR开发者需要实现的部分

### 1. VR端控制器映射

VR开发者需要：
- 读取VR手柄/控制器的位置和姿态
- 将VR控制器数据映射到机器人关节或末端位姿
- 发布命令到相应的ROS2话题

### 2. 逆运动学求解器（末端控制）

⚠️ **重要**: 如果需要使用末端位姿控制，需要实现逆运动学求解器。

可选方案：
1. **使用KDL逆运动学**（推荐）
2. **使用MoveIt!**
3. **使用TRAC-IK**
4. **自定义IK求解器**

示例（KDL IK，需要添加）：
```cpp
// 伪代码 - 需要实现
bool solve_ik(const Eigen::Vector3d& target_position,
              const Eigen::Quaterniond& target_orientation,
              std::vector<double>& joint_solution) {
    // 使用KDL或其他库求解IK
    // 将target_position和target_orientation转换为关节角度
    // 返回是否求解成功
}
```

## 集成到现有系统

如需将VR接口集成到现有的`unilateral_control`或`bilateral_control`中：

### 修改示例

在`openarm_unilateral_control.cpp`中添加：

```cpp
#include <vr_control_interface.hpp>

// 在main函数中
auto vr_interface = std::make_shared<openarm::ros2::VRControlInterface>(
    "vr_control", "leader");
vr_interface->init(argc, argv);

// 设置回调
vr_interface->set_joint_command_callback(
    [leader_state](const std::vector<double>& joint_positions) {
        // 设置leader状态的参考值
        // ... 实现代码 ...
    }
);

// 在主循环中
while (keep_running) {
    vr_interface->spin_some();
    // ... 其他代码 ...
}
```

## API参考

### VRControlInterface 类

#### 构造函数
```cpp
VRControlInterface(const std::string& node_name, 
                   const std::string& robot_name);
```

#### 主要方法
- `bool init(int argc, char** argv)` - 初始化ROS2节点
- `void set_joint_command_callback(JointCommandCallback callback)` - 设置关节命令回调
- `void set_ee_pose_command_callback(EEPoseCommandCallback callback)` - 设置末端位姿命令回调
- `void set_gripper_command_callback(GripperCommandCallback callback)` - 设置夹爪命令回调
- `void spin_some()` - 处理ROS2回调
- `bool is_ok() const` - 检查节点状态

#### 回调函数类型
```cpp
using JointCommandCallback = std::function<void(const std::vector<double>&)>;
using EEPoseCommandCallback = std::function<void(const Eigen::Vector3d&, 
                                                 const Eigen::Quaterniond&)>;
using GripperCommandCallback = std::function<void(double)>;
```

## 话题列表总结

| 话题名称 | 类型 | 方向 | 频率 | 说明 |
|---------|------|------|------|------|
| `/robot/joint_command` | sensor_msgs/JointState | 订阅 | - | 接收关节命令 |
| `/robot/ee_pose_command` | geometry_msgs/PoseStamped | 订阅 | - | 接收末端位姿命令 |
| `/robot/gripper_command` | std_msgs/Float64MultiArray | 订阅 | - | 接收夹爪命令 |
| `/robot/joint_states` | sensor_msgs/JointState | 发布 | 50Hz | 发布关节状态 |
| `/robot/ee_pose` | geometry_msgs/PoseStamped | 发布 | 50Hz | 发布末端位姿 |

## 注意事项

1. ✅ **关节控制接口** - 已完全实现，可直接使用
2. ⚠️ **末端位姿控制** - 接口已实现，但需要添加逆运动学求解器
3. ✅ **状态发布** - 已完全实现，实时发布关节和末端状态
4. 🔧 **VR端实现** - 需要VR开发者实现控制器数据采集和命令发布

## 下一步

### VR开发者需要做的：
1. 实现VR手柄数据采集
2. 将VR数据映射到机器人命令
3. 发布命令到相应ROS2话题
4. （可选）实现逆运动学求解器用于末端控制

### 机器人端已完成：
1. ✅ ROS2命令订阅接口
2. ✅ ROS2状态发布接口
3. ✅ 关节控制执行
4. ✅ 示例程序和文档

## 支持和问题

如有问题，请查看：
- 示例程序：`control/vr_control_example.cpp`
- 接口定义：`src/vr_control_interface.hpp`
- 相关文档：`ROS2_INTEGRATION.md`, `QUICK_GUIDE.md`

---

**总结**: 所有关节控制和状态发布的接口都已实现并可用。VR开发者只需实现VR端的数据采集和命令发布即可。末端位姿控制的接口已预留，但需要添加逆运动学求解器才能使用。
