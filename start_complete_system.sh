#!/bin/bash
# 完整系统启动脚本（仿真+RViz+键盘控制）
# 修复：发布双臂14个关节，robot_state_publisher话题重映射

set -e  # 遇到错误立即退出

echo "=========================================="
echo "  OpenArm完整仿真系统启动"
echo "=========================================="
echo "组件："
echo "  1. simulation_node.py - 双臂运动学仿真（14关节）"
echo "  2. robot_state_publisher - TF树生成"
echo "  3. RViz - 3D可视化"
echo "  4. keyboard_mouse_control.py - Minecraft风格控制"
echo ""
echo "控制说明："
echo "  WASD  - 前后左右移动"
echo "  鼠标  - 改变俯仰/偏航角"
echo "  Q/E   - 横滚角"
echo "  Shift - 下降"
echo "  Space - 上升"
echo "  G     - 抓取器开/关"
echo "  R     - 重置到初始位置"
echo "=========================================="
echo ""

# 清理旧进程
echo "清理旧进程..."
pkill -f simulation_node 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f rviz 2>/dev/null || true
pkill -f keyboard_mouse_control 2>/dev/null || true
sleep 1

# 检查Python版本
if ! command -v python3.10 &> /dev/null; then
    echo "错误: 需要Python 3.10！"
    echo "请安装: sudo apt install python3.10"
    exit 1
fi

# 检查URDF文件
if [ ! -f "config/openarm_v10_follower_no_hand.urdf" ]; then
    echo "错误: URDF文件不存在!"
    exit 1
fi

# 1. 启动仿真节点
echo "[1/4] 启动simulation_node（双臂14关节）..."
python3.10 simulation_node.py &
SIM_PID=$!
sleep 2

# 验证仿真节点是否运行
if ! ps -p $SIM_PID > /dev/null; then
    echo "错误: simulation_node启动失败!"
    exit 1
fi
echo "  ✓ simulation_node运行中 (PID: $SIM_PID)"

# 2. 启动robot_state_publisher（重映射话题）
echo "[2/4] 启动robot_state_publisher（TF树）..."
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(cat config/openarm_v10_follower_no_hand.urdf)" \
    -r joint_states:=/robot/joint_states &  # 关键：重映射到/robot/joint_states
RSP_PID=$!
sleep 2

if ! ps -p $RSP_PID > /dev/null; then
    echo "错误: robot_state_publisher启动失败!"
    kill $SIM_PID 2>/dev/null || true
    exit 1
fi
echo "  ✓ robot_state_publisher运行中 (PID: $RSP_PID)"

# 3. 启动RViz
echo "[3/4] 启动RViz可视化..."
source ~/ros2_ws/install/setup.bash
rviz2 -d config/simulation.rviz &
RVIZ_PID=$!
sleep 2

if ! ps -p $RVIZ_PID > /dev/null; then
    echo "警告: RViz启动失败，继续运行其他组件..."
else
    echo "  ✓ RViz运行中 (PID: $RVIZ_PID)"
fi

# 4. 启动键盘控制
echo "[4/4] 启动键盘控制（Minecraft风格）..."
python3.10 keyboard_mouse_control.py &
KB_PID=$!
sleep 1

if ! ps -p $KB_PID > /dev/null; then
    echo "警告: keyboard_mouse_control启动失败!"
else
    echo "  ✓ keyboard_mouse_control运行中 (PID: $KB_PID)"
fi

echo ""
echo "=========================================="
echo "  系统启动完成！"
echo "=========================================="
echo "PIDs:"
echo "  simulation_node: $SIM_PID"
echo "  robot_state_publisher: $RSP_PID"
echo "  RViz: $RVIZ_PID"
echo "  keyboard_control: $KB_PID"
echo ""
echo "验证系统状态："
echo "  ros2 topic hz /robot/joint_states    # 应该~50Hz"
echo "  ros2 topic hz /tf                     # 应该~16Hz"
echo "  ros2 topic list | grep robot          # 查看所有话题"
echo ""
echo "键盘控制窗口现在应该打开"
echo "移动鼠标/按键控制机械臂"
echo ""
echo "按Ctrl+C停止所有进程..."

# 捕获Ctrl+C并清理
trap 'echo ""; echo "停止所有进程..."; kill $SIM_PID $RSP_PID $RVIZ_PID $KB_PID 2>/dev/null; exit' INT TERM

# 保持脚本运行
wait
