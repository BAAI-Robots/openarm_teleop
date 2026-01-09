# OpenArm VR控制 - 5分钟上手指南

## ⚡ 最快启动方式

### 一键启动（推荐）

```bash
cd /home/robot/openarm_teleop
./quick_start_vr.sh
```

然后选择：
- **选项1**：启动机器人端（在机器人上运行）
- **选项2**：启动VR客户端示例（在VR设备上运行）

---

## 🎯 手动启动步骤

### 机器人端（3步）

```bash
# 1. 配置CAN总线
sudo ip link set can0 down && \
sudo ip link set can0 type can bitrate 1000000 && \
sudo ip link set can0 up

# 2. 生成URDF
mkdir -p /tmp/openarm_urdf_gen
xacro ~/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro \
      bimanual:=true -o /tmp/openarm_urdf_gen/v10_leader.urdf

# 3. 启动VR控制程序
cd /home/robot/openarm_teleop/build
./vr_control_example
```

### VR端（Python示例）

```bash
cd /home/robot/openarm_teleop
python3 vr_client_example.py
```

或使用命令行测试：

```bash
# 关节控制
ros2 topic pub /robot/joint_command sensor_msgs/msg/JointState '{
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}' --once

# 末端控制（自动IK）
ros2 topic pub /robot/ee_pose_command geometry_msgs/msg/PoseStamped '{
  pose: {
    position: {x: 0.3, y: 0.2, z: 0.5},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  }
}' --once

# 夹爪控制
ros2 topic pub /robot/gripper_command std_msgs/msg/Float64MultiArray '{
  data: [0.5]
}' --once
```

---

## 📡 ROS2话题总览

### VR → 机器人（控制命令）

| 话题 | 类型 | 功能 |
|------|------|------|
| `/robot/joint_command` | `sensor_msgs/JointState` | 关节角度控制 |
| `/robot/ee_pose_command` | `geometry_msgs/PoseStamped` | 末端位姿控制（自动IK） |
| `/robot/gripper_command` | `std_msgs/Float64MultiArray` | 夹爪控制 |

### 机器人 → VR（状态反馈，50Hz）

| 话题 | 类型 | 功能 |
|------|------|------|
| `/robot/joint_states` | `sensor_msgs/JointState` | 关节状态 |
| `/robot/ee_pose` | `geometry_msgs/PoseStamped` | 末端位姿 |

---

## 💻 VR端Python代码模板

### 最小示例（10行）

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

rclpy.init()
node = Node('vr_controller')

# 创建发布器
pose_pub = node.create_publisher(PoseStamped, '/robot/ee_pose_command', 10)

# 发送末端位姿命令
msg = PoseStamped()
msg.pose.position.x = 0.3
msg.pose.position.y = 0.2
msg.pose.position.z = 0.5
msg.pose.orientation.w = 1.0

pose_pub.publish(msg)  # IK自动求解！
```

### 完整示例（带状态反馈）

参考文件：[vr_client_example.py](vr_client_example.py)

---

## 🧪 快速测试

### 测试1：IK求解器

```bash
cd /home/robot/openarm_teleop/build
./ik_test
```

### 测试2：命令行控制

**终端1** - 启动机器人：
```bash
./quick_start_vr.sh  # 选择选项1
```

**终端2** - 发送命令：
```bash
ros2 topic pub /robot/ee_pose_command geometry_msgs/msg/PoseStamped '{
  pose: {position: {x: 0.3, y: 0.2, z: 0.5}, orientation: {w: 1.0}}
}' --once
```

**终端3** - 监控状态：
```bash
ros2 topic echo /robot/joint_states
```

---

## 🔥 三种控制方式对比

| 方式 | 优点 | 适用场景 |
|------|------|----------|
| **关节控制** | 直接控制，精确 | 预定义动作、关节空间规划 |
| **末端控制（IK）** | 直观，自动求解 | VR交互、末端空间规划 |
| **混合控制** | 灵活 | 复杂任务 |

---

## ⚠️ 常见问题

### Q: 机器人无响应？
```bash
# 检查ROS2话题
ros2 topic list | grep robot

# 查看话题信息
ros2 topic info /robot/joint_command
```

### Q: IK求解失败？
- 检查目标位置是否在工作空间内（<0.6m）
- 使用仅位置IK（放宽姿态约束）
- 查看机器人端日志

### Q: CAN总线错误？
```bash
# 重新配置CAN
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 检查状态
ip -details link show can0
```

---

## 📚 详细文档

- 📖 **[README.md](README.md)** - 完整README
- 🎮 **[VR_CONTROL_INTERFACE.md](VR_CONTROL_INTERFACE.md)** - VR接口详解
- 🧮 **[IK_SOLVER_GUIDE.md](IK_SOLVER_GUIDE.md)** - IK求解器指南
- ⚡ **[IK_QUICK_REFERENCE.md](IK_QUICK_REFERENCE.md)** - IK快速参考

---

## 🎯 下一步

1. **熟悉Python示例**：`python3 vr_client_example.py`
2. **修改参数测试**：调整位置、姿态、速度
3. **集成到VR系统**：使用ROS2在Unity/Unreal中集成
4. **阅读详细文档**：了解高级功能

---

**现在开始控制机器人吧！** 🚀

```bash
./quick_start_vr.sh
```
