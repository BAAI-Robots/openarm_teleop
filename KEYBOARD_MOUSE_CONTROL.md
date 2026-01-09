# 键盘鼠标控制 - 使用说明

## 🎮 概述

类似Minecraft的键盘鼠标控制界面，通过pygame窗口实时控制OpenArm机器人末端执行器。

**特点：**
- ✨ 直观的Minecraft风格操控
- 🎨 实时UI显示位姿和状态
- 🚀 60FPS流畅控制
- 🤖 自动IK求解（无需手动计算关节角度）
- 👁️ 可配合RViz可视化

---

## 🚀 快速启动

### 方法1：一键启动（推荐）

```bash
cd /home/robot/openarm_teleop
./launch_keyboard_control.sh
```

**选择启动模式：**
1. **完整模式** - 自动启动机器人、键盘控制、RViz（使用tmux多窗口）
2. **仅键盘控制** - 只运行控制界面（机器人需单独启动）
3. **仅RViz** - 只显示可视化
4. **测试模式** - 无需真实机器人，测试控制界面

### 方法2：手动启动

**终端1** - 启动机器人：
```bash
cd /home/robot/openarm_teleop/build
./vr_control_example
```

**终端2** - 启动键盘控制：
```bash
cd /home/robot/openarm_teleop
python3 keyboard_mouse_control.py
```

**终端3**（可选）- 启动RViz：
```bash
ros2 run rviz2 rviz2
```

---

## 🎯 控制方式

### 位置控制

| 按键 | 功能 | 说明 |
|------|------|------|
| **W** | 前进 | 沿X轴正方向移动 |
| **S** | 后退 | 沿X轴负方向移动 |
| **A** | 左移 | 沿Y轴正方向移动 |
| **D** | 右移 | 沿Y轴负方向移动 |
| **Space** | 上升 | 沿Z轴正方向移动 |
| **Shift** | 下降 | 沿Z轴负方向移动 |

### 姿态控制

| 输入 | 功能 | 说明 |
|------|------|------|
| **Q** | Roll左旋 | 绕X轴逆时针旋转 |
| **E** | Roll右旋 | 绕X轴顺时针旋转 |
| **鼠标左右** | Yaw | 绕Z轴旋转（左右转头） |
| **鼠标上下** | Pitch | 绕Y轴旋转（抬头低头） |

### 其他控制

| 按键 | 功能 | 说明 |
|------|------|------|
| **G** | 切换夹爪 | 打开/关闭夹爪 |
| **R** | 重置位姿 | 回到初始位置 (0.3, 0.2, 0.4) |
| **ESC** | 退出 | 关闭程序 |

---

## 📊 UI界面说明

pygame窗口显示以下信息：

### 状态区域
- **连接状态**：显示与机器人的连接状态
- **当前位姿**：实时显示末端执行器位置和姿态
  - Position: X, Y, Z (米)
  - Orientation: Roll, Pitch, Yaw (度)
- **夹爪状态**：显示夹爪开/关状态

### 控制提示
- 完整的按键功能说明
- 实时鼠标移动指示器

### 统计信息
- 已发送命令数量
- 实时FPS显示

---

## 🔧 参数调整

### 修改控制速度

编辑 `keyboard_mouse_control.py`：

```python
# 在 __init__ 方法中
self.move_speed = 0.002        # 位置移动速度 (米/帧)
self.rotate_speed = 0.02       # 旋转速度 (弧度/帧)
self.mouse_sensitivity = 0.001 # 鼠标灵敏度
```

### 修改初始位姿

```python
# 在 __init__ 方法中
self.current_position = [0.3, 0.2, 0.4]     # [x, y, z]
self.current_orientation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw]
```

### 修改工作空间限制

```python
# 在主循环的限制范围部分
controller.current_position[0] = max(-0.6, min(0.6, ...))  # X轴
controller.current_position[1] = max(-0.6, min(0.6, ...))  # Y轴
controller.current_position[2] = max(0.1, min(0.8, ...))   # Z轴
```

---

## 🎨 配合RViz使用

### RViz配置步骤

1. **启动RViz**
   ```bash
   ros2 run rviz2 rviz2
   ```

2. **添加显示项**
   - Add → RobotModel
   - Add → TF
   - Add → Axes

3. **配置参数**
   - Fixed Frame: `world` 或 `base_link`
   - Robot Description: `/robot_description`（如果有）

4. **订阅话题**
   - `/robot/joint_states` - 关节状态
   - `/robot/ee_pose` - 末端位姿

### 实时可视化

RViz会实时显示：
- 机器人模型姿态
- 末端执行器位置
- 坐标系变换

---

## 🐛 故障排查

### Q: pygame窗口打不开？

```bash
# 注意：必须使用 Python 3.10（ROS2 Humble要求）
python3.10 -m pip install pygame

# 或使用apt（但可能不是3.10版本）
sudo apt install python3-pygame
```

### Q: 机器人无响应？

1. 检查机器人端是否运行：
   ```bash
   ros2 topic list | grep robot
   ```

2. 查看连接状态（在pygame窗口顶部）

3. 检查ROS2话题：
   ```bash
   ros2 topic echo /robot/ee_pose_command
   ```

### Q: 控制不流畅？

- 降低帧率：修改 `clock.tick(60)` 为 `clock.tick(30)`
- 检查CPU占用
- 关闭不必要的程序

### Q: IK求解失败？

- 检查目标位置是否在工作空间内
- 查看机器人端日志
- 尝试较小的移动步长

### Q: 鼠标控制不灵敏？

调整灵敏度：
```python
self.mouse_sensitivity = 0.002  # 增加数值提高灵敏度
```

---

## 📈 性能指标

| 指标 | 数值 | 说明 |
|------|------|------|
| **控制频率** | 60 FPS | pygame主循环 |
| **命令发送** | 动态 | 仅在移动时发送 |
| **延迟** | <20ms | 键盘到机器人响应 |
| **IK求解** | 1-5ms | LMA算法 |

---

## 🎓 使用技巧

### 1. 精确控制
- 使用**短促点击**进行微调
- 按住按键实现连续移动
- **R键**快速回到安全位置

### 2. 姿态控制
- **鼠标控制Pitch/Yaw**更直观
- **Q/E控制Roll**适合细节调整
- 组合使用达到任意姿态

### 3. 高效操作
- 先用键盘移动到大致位置
- 再用鼠标调整姿态
- **G键**快速操作夹爪

### 4. 安全实践
- 首次使用先在测试模式熟悉控制
- 注意工作空间限制（防止碰撞）
- 异常时立即按**ESC**退出

---

## 🚀 高级功能

### 录制和回放

可以扩展脚本实现录制功能：

```python
# 记录轨迹
trajectory = []
if recording:
    trajectory.append({
        'position': current_position.copy(),
        'orientation': current_orientation.copy(),
        'timestamp': time.time()
    })
```

### 多机器人控制

通过修改话题前缀实现多机器人控制：

```python
# 机器人1
self.ee_pose_pub = self.create_publisher(
    PoseStamped, '/robot1/ee_pose_command', 10)

# 机器人2
self.ee_pose_pub = self.create_publisher(
    PoseStamped, '/robot2/ee_pose_command', 10)
```

### 自定义UI

修改 `draw_ui()` 函数添加自定义显示内容。

---

## 📚 相关文档

- [README.md](README.md) - 项目总览
- [QUICKSTART_VR.md](QUICKSTART_VR.md) - VR控制快速入门
- [VR_CONTROL_INTERFACE.md](VR_CONTROL_INTERFACE.md) - ROS2接口详解
- [IK_SOLVER_GUIDE.md](IK_SOLVER_GUIDE.md) - IK求解器文档

---

## 💡 示例场景

### 场景1：拾取物体
1. 用**WASD**移动到物体上方
2. 用**Shift**下降到合适高度
3. 用鼠标调整姿态对准
4. 按**G**关闭夹爪抓取
5. 用**Space**提升物体

### 场景2：绘制轨迹
1. 按**R**重置到起始位置
2. 按住**W**沿直线前进
3. 配合**A/D**绘制曲线
4. 按**G**控制画笔升降

### 场景3：精确对位
1. 用**WASD**粗略定位
2. 短促点击微调位置
3. 用鼠标精确调整角度
4. 按**G**执行操作

---

## ✅ 总结

**键盘鼠标控制提供：**
- ✨ 直观的Minecraft风格操控
- 🎮 实时交互式控制
- 🤖 自动IK求解
- 👁️ RViz可视化支持
- 🎯 适合测试、演示、教学

**立即开始：**
```bash
./launch_keyboard_control.sh
```

**享受类Minecraft的机器人控制体验！** 🎮🤖
