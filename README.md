# OpenArm Teleoperation - KDL仿真系统

OpenArm双臂机器人的键盘鼠标控制仿真系统，基于KDL运动学库。

## 🚀 快速启动

```bash
cd /home/robot/openarm_teleop
./start_complete_system.sh
```

**启动后会自动打开**：
- ROS2仿真节点（7-DOF运动学）
- Robot State Publisher（TF树）
- RViz可视化
- 键盘鼠标控制界面

## 🎮 控制方式

### 键盘
- **W/S**: 前进/后退
- **A/D**: 左移/右移
- **Shift/Space**: 下降/上升
- **Q/E**: 左滚/右滚
- **R**: 重置到初始位置

### 鼠标
1. **点击Pygame窗口** → 锁定鼠标（显示🔒）
2. **移动鼠标** → 控制Pitch/Yaw旋转
3. **按ESC** → 释放鼠标（显示🖱️）

## 📊 系统架构

```
simulation_node.py (ROS2节点)
├── 7-DOF运动学 (KDL IK/FK)
├── 发布 /robot/joint_states (14关节: 左臂7 + 右臂7)
├── 订阅 /robot/ee_pose_command
└── 只控制右臂，左臂固定显示

keyboard_mouse_control.py (控制界面)
├── Pygame GUI (60 FPS)
├── 键盘鼠标输入
└── 发布目标位姿到 /robot/ee_pose_command

robot_state_publisher
└── 生成TF树用于RViz显示

RViz
└── 3D可视化机械臂模型
```

## 📁 核心文件

### Python脚本
- `simulation_node.py` - 主仿真节点（7-DOF IK/FK）
- `keyboard_mouse_control.py` - 键鼠控制界面
- `debug_joints.py` - 关节状态调试工具
- `test_7dof_ik.py` - IK求解器测试

### 启动脚本
- `start_complete_system.sh` - 一键启动完整系统

### 配置文件
- `config/openarm_v10_follower_no_hand.urdf` - 双臂机器人URDF
- `config/simulation.rviz` - RViz配置

### 文档
- `HOW_TO_START_SIMULATION.md` - 详细启动和使用指南
- `KINEMATICS_7DOF.md` - 7自由度运动学说明
- `KEYBOARD_MOUSE_CONTROL.md` - 控制界面使用说明
- `ROS2_INTEGRATION.md` - ROS2集成技术细节
- `TROUBLESHOOTING.md` - 常见问题排查

## ⚙️ 技术参数

| 参数 | 值 |
|------|-----|
| 自由度 | 7-DOF（右臂）|
| IK求解器 | KDL LMA |
| IK成功率 | 80-90% |
| 发布频率 | 50 Hz |
| UI帧率 | 60 FPS |
| 控制延迟 | <20ms |

## 🔧 依赖要求

### 系统依赖
- ROS2 Humble
- Python 3.10

### Python包
```bash
pip3 install pygame numpy
sudo apt install ros-humble-kdl-parser python3-pykdl
```

### ROS2包
- openarm_description
- openarm_bimanual_moveit_config

## 🐛 常见问题

### Q: 机器人模型显示不正确？
**A**: 确保已source ROS2工作空间：
```bash
source ~/ros2_ws/install/setup.bash
./start_complete_system.sh
```

### Q: IK求解失败？
**A**: 正常情况。7-DOF冗余机械臂的IK成功率为80-90%。失败时机械臂不会移动，尝试其他目标位置即可。

### Q: 键盘没反应？
**A**: 
1. 确保Pygame窗口已激活（点击窗口）
2. 检查终端是否有错误信息
3. 重启系统：Ctrl+C后重新运行

### Q: 鼠标控制无效？
**A**: 需要先点击Pygame窗口锁定鼠标（显示🔒图标），按ESC释放。

## 📚 详细文档

- [完整启动指南](HOW_TO_START_SIMULATION.md)
- [7-DOF运动学详解](KINEMATICS_7DOF.md)
- [控制界面说明](KEYBOARD_MOUSE_CONTROL.md)
- [故障排查](TROUBLESHOOTING.md)

## 🤝 贡献

参见 [CONTRIBUTING.md](CONTRIBUTING.md)

## 📄 许可证

参见 [LICENSE.txt](LICENSE.txt)

## 🎯 项目特点

✅ **即开即用** - 一条命令启动完整系统  
✅ **实时响应** - 低延迟键鼠控制  
✅ **可视化** - RViz实时显示机器人状态  
✅ **双臂支持** - 完整的双臂模型（控制右臂）  
✅ **7-DOF** - 完整的7自由度运动学  
✅ **纯Python** - 易于修改和扩展  

---

**版本**: 1.0  
**更新日期**: 2026-01-09
