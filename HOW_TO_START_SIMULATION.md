# OpenArm仿真系统使用指南

## 🚀 快速启动

### 方法1：一键启动（推荐）
```bash
cd /home/robot/openarm_teleop
./start_complete_system.sh
```

这将启动：
- ✅ simulation_node（7-DOF运动学仿真）
- ✅ robot_state_publisher（TF树生成）
- ✅ RViz（3D可视化）
- ✅ keyboard_mouse_control（键鼠控制）

### 方法2：分步启动

#### 终端1：启动仿真节点
```bash
cd /home/robot/openarm_teleop
python3.10 simulation_node.py
```

#### 终端2：启动robot_state_publisher
```bash
cd /home/robot/openarm_teleop
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(cat config/openarm_v10_follower_no_hand.urdf)" \
    -r joint_states:=/robot/joint_states
```

#### 终端3：启动RViz（可选）
```bash
source ~/ros2_ws/install/setup.bash
rviz2 -d /home/robot/openarm_teleop/config/simulation.rviz
```

#### 终端4：启动键盘控制
```bash
cd /home/robot/openarm_teleop
python3.10 keyboard_mouse_control.py
```

---

## 🎮 键盘鼠标控制说明

### 鼠标控制模式

**启动后操作流程：**
1. **点击窗口** → 进入控制模式（鼠标锁定）
   - 鼠标被锁定在窗口中央
   - 光标隐藏
   - 显示 "🔒 MOUSE LOCKED"

2. **按ESC键** → 释放鼠标
   - 光标恢复显示
   - 可以移动到其他窗口
   - 显示 "🖱️ MOUSE FREE"

3. **再次点击** → 重新进入控制模式

### 控制键位

#### 鼠标（锁定时生效）
- **左右移动** → Yaw（偏航）
- **上下移动** → Pitch（俯仰）

#### 键盘移动
- **W** → 向前（+X）
- **S** → 向后（-X）
- **A** → 向左（+Y）
- **D** → 向右（-Y）
- **Shift** → 向下（-Z）
- **Space** → 向上（+Z）

#### 姿态控制
- **Q** → Roll左旋
- **E** → Roll右旋
- **鼠标** → Pitch/Yaw（需锁定鼠标）

#### 其他功能
- **G** → 打开/关闭夹爪
- **R** → 重置到初始位姿
- **ESC** → 释放鼠标（不退出程序）

### UI显示

界面显示以下信息：
```
🔒 MOUSE LOCKED (或 🖱️ MOUSE FREE)
(Press ESC to release) (或 Click window to control)

Position: X=0.300  Y=0.200  Z=0.400
Orientation: Roll=0.0°  Pitch=0.0°  Yaw=0.0°
Gripper: Open (绿色) / Closed (红色)

Controls:
  W/S - Forward/Backward
  A/D - Left/Right
  ...

Statistics:
  Commands sent: 123
```

---

## 🤖 系统组件说明

### 1. simulation_node.py
**功能**：7-DOF运动学仿真引擎
- 发布14个关节状态（左臂7+右臂7）
- 执行正向/逆向运动学
- 接受末端位姿命令

**ROS2话题**：
- 发布：`/robot/joint_states` (50Hz)
- 发布：`/robot/ee_pose` (50Hz)
- 订阅：`/robot/ee_pose_command`

### 2. robot_state_publisher
**功能**：根据关节状态生成TF变换树
- 读取URDF模型
- 发布TF变换（~16Hz）
- 连接所有link的坐标系

### 3. RViz
**功能**：3D可视化
- 显示机器人模型
- 显示TF坐标轴
- 实时更新关节状态

### 4. keyboard_mouse_control.py
**功能**：键鼠交互控制
- FPS风格鼠标控制
- 实时末端位姿命令发布
- 可视化UI界面

---

## 🔍 验证系统状态

### 检查节点
```bash
ros2 node list
# 应该看到：
# /openarm_simulation
# /robot_state_publisher
```

### 检查话题
```bash
ros2 topic list | grep robot
# 应该看到：
# /robot/joint_states
# /robot/ee_pose
# /robot/ee_pose_command
# /robot/gripper_command
```

### 检查频率
```bash
# 关节状态（应该~50Hz）
ros2 topic hz /robot/joint_states

# TF变换（应该~16Hz）
ros2 topic hz /tf
```

### 查看关节状态
```bash
ros2 topic echo /robot/joint_states
# 应该看到14个关节：
# openarm_left_joint1~7
# openarm_right_joint1~7
```

---

## ⚠️ 常见问题

### Q1: 键盘控制窗口无响应
**解决**：点击窗口进入控制模式（看到"MOUSE LOCKED"提示）

### Q2: 鼠标无法移出窗口
**解决**：按ESC键释放鼠标

### Q3: RViz显示网格缺失
**解决**：确保已source工作空间
```bash
source ~/ros2_ws/install/setup.bash
rviz2 -d config/simulation.rviz
```

### Q4: IK频繁失败（-101错误）
**原因**：7-DOF冗余臂的正常现象
- 成功率80-90%是正常的
- 失败通常发生在奇异点或工作空间边界
- 不影响正常使用

详见：[KINEMATICS_7DOF.md](KINEMATICS_7DOF.md)

### Q5: 机器人在RViz中不动
**检查**：
```bash
# 1. 检查关节状态是否发布
ros2 topic hz /robot/joint_states

# 2. 检查robot_state_publisher订阅
ros2 node info /robot_state_publisher

# 3. 验证话题重映射
# 应该订阅 /robot/joint_states 而不是 /joint_states
```

### Q6: 左臂关节卡在原点
**正常现象**：左臂保持零位（不控制），只有右臂接受IK命令

---

## 🎯 控制技巧

### 1. 平滑移动
- 按住移动键不要松开
- 避免频繁切换方向
- 使用小步长连续移动

### 2. 姿态调整
- 先锁定鼠标（点击窗口）
- 缓慢移动鼠标调整pitch/yaw
- 使用Q/E微调roll

### 3. 避免IK失败
- 不要移动到极限位置
- 避免快速大幅度移动
- 保持在工作空间中心区域

### 4. 重置姿态
- 如果机器人姿态异常，按R键重置
- 重置后位姿：X=0.3, Y=0.2, Z=0.4

---

## 📊 性能参数

| 项目 | 数值 |
|------|------|
| 关节状态频率 | 50 Hz |
| 末端位姿频率 | 50 Hz |
| TF变换频率 | ~16 Hz |
| UI帧率 | 60 FPS |
| IK求解时间 | 1-5 ms |
| IK成功率 | 80-90% |
| 移动速度 | 2 mm/帧 |
| 旋转速度 | 0.02 rad/帧 |
| 鼠标灵敏度 | 0.001 |

---

## 📝 日志输出

### 正常启动日志
```
=== OpenArm Simulation Node ===
Publish Rate: 50.0 Hz
✓ robot_description published
✓ KDL chain loaded: 7 joints (7-DOF arm)
✓ IK solver: LMA (maxiter=500, eps=1e-5)
✓ Simulation node ready!
```

### 控制器启动日志
```
OpenArm Keyboard Mouse Control
✓ Initialization complete!
✓ Robot connected! You can now control the robot.

💡 TIP: Click the window to enter control mode
💡 Press ESC to release mouse (not exit)

🔒 Mouse LOCKED - Control mode active
```

---

## 🔧 高级配置

### 调整控制参数
编辑 `keyboard_mouse_control.py`：
```python
self.move_speed = 0.002          # 米/帧（默认2mm）
self.rotate_speed = 0.02         # 弧度/帧
self.mouse_sensitivity = 0.001   # 鼠标灵敏度
```

### 调整工作空间限制
编辑 `keyboard_mouse_control.py`：
```python
# 位置限制（米）
controller.current_position[0] = max(-0.6, min(0.6, x))  # X
controller.current_position[1] = max(-0.6, min(0.6, y))  # Y
controller.current_position[2] = max(0.1, min(0.8, z))   # Z
```

### 修改IK参数
编辑 `simulation_node.py`：
```python
self.ik_solver = ChainIkSolverPos_LMA(
    self.kdl_chain,
    maxiter=500,      # 迭代次数（越大越慢但成功率越高）
    eps=1e-5,         # 位置精度
    eps_joints=1e-5   # 关节精度
)
```

---

## 🆘 紧急停止

如需停止所有进程：
```bash
# 方法1：在start_complete_system.sh终端按Ctrl+C

# 方法2：手动杀死进程
pkill -f simulation_node
pkill -f robot_state_publisher
pkill -f rviz
pkill -f keyboard_mouse_control
```

---

## 📚 相关文档

- [README.md](README.md) - 项目总览
- [KINEMATICS_7DOF.md](KINEMATICS_7DOF.md) - 7自由度运动学详解
- [SIMULATION_GUIDE.md](SIMULATION_GUIDE.md) - 仿真系统架构
- [QUICKSTART_VR.md](QUICKSTART_VR.md) - VR控制说明
- [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - 故障排除

---

**祝使用愉快！** 🎉

如有问题，请查看 [TROUBLESHOOTING.md](TROUBLESHOOTING.md) 或运行：
```bash
python3.10 test_7dof_ik.py  # 测试IK性能
python3.10 debug_joints.py  # 调试关节状态
```
