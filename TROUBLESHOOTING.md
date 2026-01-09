# 仿真系统测试和故障排除指南

## ✅ 已修复的问题

1. **7自由度支持** - 正确配置7个关节（6个臂关节 + 1个gripper）
2. **初始位姿** - 改为非零位姿避免奇异点：`[0.0, -0.5, 0.8, 0.0, 0.5, 0.0, 0.0]`
3. **Frame ID** - 末端位姿相对于 `openarm_right_link0`
4. **KDL链** - 更新为OpenArm v10的真实DH参数

## 🚀 正确的启动步骤

### 方法1：使用启动脚本（推荐）

```bash
# 终端1：启动完整系统
./start_full_simulation.sh

# 等待3-5秒，然后在终端2启动键盘控制
python3.10 keyboard_mouse_control.py
```

### 方法2：手动分步启动（用于调试）

**终端1 - 仿真节点：**
```bash
cd /home/robot/openarm_teleop
python3.10 simulation_node.py
```

**终端2 - robot_state_publisher：**
```bash
source /opt/ros/humble/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(cat /home/robot/openarm_teleop/config/openarm_v10_follower_no_hand.urdf)"
```

**终端3 - 测试关节状态：**
```bash
python3.10 test_joint_states.py
```

**终端4 - 键盘控制：**
```bash
python3.10 keyboard_mouse_control.py
```

**终端5 - RViz（可选）：**
```bash
rviz2 -d /home/robot/openarm_teleop/config/simulation.rviz
```

## 🔍 验证系统工作

### 1. 检查ROS2节点

```bash
ros2 node list
# 应该看到:
#   /openarm_simulation
#   /robot_state_publisher
```

### 2. 检查发布的话题

```bash
ros2 topic list
# 应该看到:
#   /robot/joint_states
#   /robot/ee_pose
#   /robot/ee_pose_command
#   /tf
#   /tf_static
```

### 3. 监控关节状态

```bash
ros2 topic echo /robot/joint_states
# 应该看到7个关节的实时数据
```

### 4. 检查TF树

```bash
# 查看所有frames
ros2 run tf2_tools view_frames

# 检查特定变换
ros2 run tf2_ros tf2_echo openarm_right_link0 openarm_right_link6
```

## 🐛 常见问题

### Q: "Could not load mesh resource 'package://openarm_description/...'"

**原因**: RViz找不到mesh文件，因为ROS2工作空间没有source

**解决**:
```bash
# 启动RViz前必须source工作空间
source ~/ros2_ws/install/setup.bash
rviz2 -d config/simulation.rviz

# 或使用启动脚本（已自动source）
./start_full_simulation.sh
```

**注意**: 所有启动脚本已更新，会自动source工作空间。

### Q: "No transform from [openarm_left_link1] to [world]"

**原因**: robot_state_publisher还没有启动或者URDF没有正确加载

**解决**:
```bash
# 确保按顺序启动:
# 1. simulation_node.py (发布robot_description参数)
# 2. robot_state_publisher (使用robot_description生成TF)
# 3. 等待2-3秒让TF树建立
```

### Q: 机械臂不动

**原因**: 
1. 键盘控制窗口没有获得焦点
2. IK求解失败
3. 目标位姿超出工作空间

**解决**:
1. **点击pygame窗口**确保获得焦点
2. 查看仿真节点日志：`[WARN] IK failed with code: ...`
3. 按 `R` 键重置到初始位姿
4. 使用小步长移动（WASD，不要连续按住）

### Q: RViz显示"Global Status: Warn"

**原因**: Fixed Frame设置错误

**解决**:
RViz配置已更新为使用 `world` 作为Fixed Frame。如果手动启动RViz，设置：
- Global Options → Fixed Frame → `world`

### Q: IK求解失败率高

**原因**: 目标位姿超出工作空间或接近奇异点

**解决**:
```bash
# 修改keyboard_mouse_control.py中的移动步长
# 减小 move_speed 和 rotate_speed
```

## 📊 系统架构

```
simulation_node.py
├── 发布: /robot/joint_states (7个关节)
├── 发布: /robot/ee_pose  
├── 订阅: /robot/ee_pose_command
└── 发布: robot_description参数
    ↓
robot_state_publisher
├── 订阅: /robot/joint_states
├── 读取: robot_description参数
└── 发布: /tf, /tf_static (所有link之间的变换)
    ↓
RViz
├── 订阅: /tf, /tf_static
├── 读取: robot_description
└── 显示: 3D机器人模型
```

## 🎯 调试技巧

### 实时监控IK求解

在simulation_node.py中取消注释：
```python
# self.get_logger().info('IK solved successfully', throttle_duration_sec=1.0)
```

### 查看详细的TF信息

```bash
# 以PDF格式生成TF树
ros2 run tf2_tools view_frames
evince frames.pdf

# 持续监控特定变换
watch -n 0.5 'ros2 run tf2_ros tf2_echo world openarm_right_link7'
```

### 检查关节限位

```bash
# 查看URDF中的关节限位
grep -A5 '<limit' config/openarm_v10_follower_no_hand.urdf | grep openarm_right
```

## 📝 关键参数

### simulation_node.py

- `publish_rate`: 50 Hz（关节状态发布频率）
- `joint_positions`: 初始关节位置（非零避免奇异点）
- KDL链：6个关节用于IK（joint7是gripper，不参与IK）

### keyboard_mouse_control.py

- `move_speed`: 0.002 m/frame（位置控制步长）
- `rotate_speed`: 0.02 rad/frame（旋转控制步长）
- `mouse_sensitivity`: 0.001（鼠标灵敏度）

## ✅ 验证清单

运行完整系统前确认：

- [ ] Python 3.10环境
- [ ] pygame已安装
- [ ] PyKDL已安装
- [ ] ROS2 Humble已source
- [ ] URDF文件存在: `config/openarm_v10_follower_no_hand.urdf`
- [ ] simulation_node.py可执行
- [ ] keyboard_mouse_control.py可执行

## 🆘 仍有问题？

1. 查看所有节点日志
2. 运行test_joint_states.py查看关节状态
3. 使用test_tf.sh检查TF树
4. 检查URDF: `check_urdf config/openarm_v10_follower_no_hand.urdf`
