# OpenArm IK求解器 - 快速参考

## 🎯 三行代码使用IK

```cpp
dynamics.InitIKSolver(true);  // 1. 初始化IK求解器

std::vector<double> solution;
bool ok = dynamics.SolveIKQuaternion(position, quat, current_joints, solution);  // 2. 求解

if (ok) robot_state->set_references(solution);  // 3. 应用
```

---

## 📡 ROS2话题接口

### VR发送末端控制命令
```bash
ros2 topic pub /robot/ee_pose_command geometry_msgs/msg/PoseStamped '{
  pose: {
    position: {x: 0.3, y: 0.2, z: 0.5},
    orientation: {w: 1.0, x: 0.0, y: 0.0, z: 0.0}
  }
}'
```

**自动触发IK求解** → 关节运动

---

## 🚀 快速测试

```bash
# 1. 测试IK功能
cd /home/robot/openarm_teleop/build
./ik_test

# 2. 运行VR控制（集成IK）
./vr_control_example

# 3. 另一终端发送位姿命令
ros2 topic pub /robot/ee_pose_command ...
```

---

## 📊 可用方案对比

| 方案 | 状态 | 速度 | 成功率 | 推荐度 |
|------|------|------|--------|--------|
| **KDL LMA** | ✅ 已集成 | 1-5ms | 85-95% | ⭐⭐⭐⭐⭐ |
| MoveIt | ✅ 已安装 | 5-20ms | 95%+ | ⭐⭐⭐⭐ |
| TRAC-IK | 未安装 | <1ms | 98%+ | ⭐⭐⭐⭐⭐ |

---

## 🔧 三种IK接口

### 1. 四元数（推荐用于ROS2）
```cpp
dynamics.SolveIKQuaternion(
    Eigen::Vector3d(x, y, z),       // 位置
    Eigen::Quaterniond(w, x, y, z), // 姿态
    current_joints,                  // 当前关节（初值）
    solution                         // 输出
);
```

### 2. 旋转矩阵
```cpp
dynamics.SolveIK(position, rotation_matrix, current_joints, solution);
```

### 3. 仅位置（忽略姿态）
```cpp
dynamics.SolveIKPositionOnly(position, current_joints, solution);
```

---

## ⚡ 性能提示

- ✅ 使用当前关节作为初值（最优）
- ✅ LMA算法比数值迭代快3倍
- ✅ 适合500Hz控制循环
- ⚠️ 某些边界姿态可能无解

---

## 📂 关键文件

```
src/controller/
├── ik_solver.hpp      # IK求解器类
├── ik_solver.cpp      # IK实现
├── dynamics.hpp       # IK接口
└── dynamics.cpp       # IK集成

control/
├── vr_control_example.cpp  # VR控制（已集成IK）
└── ik_test.cpp             # IK测试

docs/
├── IK_SOLVER_GUIDE.md           # 详细文档（300行）
└── IK_IMPLEMENTATION_SUMMARY.md  # 实现总结
```

---

## 🎮 VR开发者：你只需要

### 发送ROS2消息

**末端位姿控制**（IK自动求解）：
```python
pose_pub.publish(PoseStamped)  # → /robot/ee_pose_command
```

**或关节控制**（直接控制）：
```python
joint_pub.publish(JointState)  # → /robot/joint_command
```

**机器人端已经完成**：
- ✅ IK求解器集成
- ✅ ROS2接口实现
- ✅ 实时控制循环
- ✅ 状态发布（50Hz）

---

## 🐛 故障排查

### IK求解失败？
1. 检查目标位置是否在工作空间内（通常<0.6m）
2. 尝试`SolveIKPositionOnly`（仅约束位置）
3. 使用当前关节作为初值

### 需要更高性能？
```bash
# 安装TRAC-IK（速度快2-3倍）
sudo apt install ros-humble-trac-ik-kinematics-plugin
```

---

## 📞 获取帮助

- 详细文档：[IK_SOLVER_GUIDE.md](IK_SOLVER_GUIDE.md)
- 实现总结：[IK_IMPLEMENTATION_SUMMARY.md](IK_IMPLEMENTATION_SUMMARY.md)
- 测试程序：`./ik_test`
- VR示例：`./vr_control_example`

---

**现在你可以直接使用末端位姿控制机器人了！** 🎉
