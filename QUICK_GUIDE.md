# OpenArm 快速使用指南

## 问题已解决 ✓

### 1. 编译问题
**问题**: `make[1]: *** 没有规则可制作目标"CMakeFiles/Makefile2"`

**解决方案**: 已重新配置并成功编译
```bash
cd /home/robot/openarm_teleop
rm -rf build
mkdir build && cd build
source /opt/ros/humble/setup.bash
cmake ..
make -j$(nproc)
```

**状态**: ✅ 编译成功，所有可执行文件已生成

### 2. URDF文件问题
**问题**: `[ERROR] Leader URDF not found: /tmp/openarm_urdf_gen/v10_leader.urdf`

**原因**: URDF文件需要通过xacro从模板生成

**URDF文件位置**:
- 源文件（xacro模板）: `/home/robot/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro`
- 生成的URDF文件: `/tmp/openarm_urdf_gen/v10_leader.urdf` 和 `v10_follower.urdf`

**状态**: ✅ 已生成URDF文件

---

## 快速启动方法

### 方法1: 使用新的快速启动脚本 (推荐)

```bash
cd /home/robot/openarm_teleop

# 单边控制 (unilateral)
./quick_start_ros2.sh right_arm can0 can1 unilateral

# 双边控制 (bilateral)
./quick_start_ros2.sh right_arm can0 can1 bilateral

# 左手臂
./quick_start_ros2.sh left_arm can1 can3 unilateral
```

脚本会自动：
1. Source ROS2环境
2. 生成URDF文件
3. 检查可执行文件
4. 启动程序
5. 退出时清理临时文件

### 方法2: 手动运行

#### 步骤1: 生成URDF文件
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

mkdir -p /tmp/openarm_urdf_gen

# 生成URDF
xacro ~/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro \
      bimanual:=true \
      -o /tmp/openarm_urdf_gen/v10_leader.urdf

# 复制为follower
cp /tmp/openarm_urdf_gen/v10_leader.urdf \
   /tmp/openarm_urdf_gen/v10_follower.urdf
```

#### 步骤2: 运行控制程序
```bash
cd /home/robot/openarm_teleop

# 单边控制
./build/unilateral_control \
    /tmp/openarm_urdf_gen/v10_leader.urdf \
    /tmp/openarm_urdf_gen/v10_follower.urdf \
    right_arm can0 can1

# 或双边控制
./build/bilateral_control \
    /tmp/openarm_urdf_gen/v10_leader.urdf \
    /tmp/openarm_urdf_gen/v10_follower.urdf \
    right_arm can0 can1
```

### 方法3: 使用原始launch脚本

```bash
cd /home/robot/openarm_teleop

# 单边控制
./script/launch_unilateral.sh right_arm can0 can1

# 双边控制
./script/launch_bilateral.sh right_arm can0 can1
```

---

## 查看ROS2话题

程序运行后，在另一个终端：

```bash
source /opt/ros/humble/setup.bash

# 查看所有话题
ros2 topic list

# 查看关节状态
ros2 topic echo /leader/joint_states
ros2 topic echo /follower/joint_states

# 查看末端执行器位姿
ros2 topic echo /leader/ee_pose
ros2 topic echo /follower/ee_pose

# 查看话题频率（约50Hz）
ros2 topic hz /leader/joint_states
```

---

## 编译项目（如有修改）

```bash
cd /home/robot/openarm_teleop/build
source /opt/ros/humble/setup.bash
cmake .. && make -j$(nproc)
```

---

## 文件结构

```
/home/robot/openarm_teleop/
├── build/                           # 编译输出目录
│   ├── unilateral_control          # 单边控制可执行文件 ✓
│   ├── bilateral_control           # 双边控制可执行文件 ✓
│   ├── gravity_comp                # 重力补偿 ✓
│   └── ...
├── src/
│   ├── ros2_publisher.hpp          # ROS2发布器头文件 ✓
│   ├── ros2_publisher.cpp          # ROS2发布器实现 ✓
│   └── ...
├── control/
│   ├── openarm_unilateral_control.cpp  # 已集成ROS2 ✓
│   ├── openarm_bilateral_control.cpp   # 已集成ROS2 ✓
│   └── ...
├── quick_start_ros2.sh             # 快速启动脚本 ✓
├── ROS2_INTEGRATION.md             # ROS2集成详细文档
└── QUICK_GUIDE.md                  # 本文档

/home/robot/ros2_ws/
└── src/openarm_description/
    └── urdf/
        └── robot/
            └── v10.urdf.xacro      # URDF模板文件

/tmp/openarm_urdf_gen/              # 临时URDF文件
├── v10_leader.urdf                 # 已生成 ✓
└── v10_follower.urdf               # 已生成 ✓
```

---

## 常见问题

### Q1: 找不到URDF文件
**A**: 使用quick_start_ros2.sh脚本会自动生成，或按照方法2手动生成

### Q2: 编译错误
**A**: 确保已source ROS2环境：
```bash
source /opt/ros/humble/setup.bash
```

### Q3: ROS2话题看不到
**A**: 确保：
1. 程序已启动且运行正常
2. 在另一个终端source了ROS2环境
3. 程序输出显示 "ROS2 publisher initialized"

### Q4: CAN接口错误
**A**: 检查CAN接口是否正确配置：
```bash
ip link show can0
ip link show can1
```

如需配置CAN：
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
```

---

## 进一步信息

- 详细的ROS2集成文档: [ROS2_INTEGRATION.md](ROS2_INTEGRATION.md)
- OpenArm官方文档: https://docs.openarm.dev/
- 项目README: [README.md](README.md)

---

## 系统状态

✅ 所有问题已解决
✅ 编译成功
✅ URDF文件已生成
✅ ROS2集成完成
✅ 可以正常使用
