#!/bin/bash
# 双臂演示系统启动脚本
# 左臂画圆，右臂画方，自动可视化

set -e

echo "=========================================="
echo "  OpenArm 双臂轨迹演示系统"
echo "=========================================="
echo "演示内容："
echo "  • 左臂: 画圆 (XZ平面, 半径4cm)"
echo "  • 右臂: 画方 (XZ平面, 边长6cm)"
echo "  • 周期: 10秒/圈"
echo ""
echo "组件："
echo "  1. robot_state_publisher - TF树生成"
echo "  2. RViz - 3D可视化"
echo "  3. dual_arm_demo.py - 双臂轨迹控制"
echo "=========================================="
echo ""

# 清理旧进程
echo "清理旧进程..."
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f rviz 2>/dev/null || true
pkill -f dual_arm_demo 2>/dev/null || true
pkill -f simulation_node 2>/dev/null || true
pkill -f keyboard_mouse_control 2>/dev/null || true
sleep 1

# 检查Python版本
if ! command -v python3.10 &> /dev/null; then
    echo "错误: 需要Python 3.10！"
    exit 1
fi

# 检查URDF文件
if [ ! -f "config/openarm_v10_follower_no_hand.urdf" ]; then
    echo "错误: URDF文件不存在!"
    exit 1
fi

# 检查运动学库
if [ ! -f "openarm_kinematics.py" ]; then
    echo "错误: openarm_kinematics.py 不存在!"
    exit 1
fi

# 1. 启动robot_state_publisher
echo "[1/3] 启动robot_state_publisher（TF树）..."
source ~/ros2_ws/install/setup.bash
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(cat config/openarm_v10_follower_no_hand.urdf)" \
    -r joint_states:=/robot/joint_states &
RSP_PID=$!
sleep 2

if ! ps -p $RSP_PID > /dev/null; then
    echo "错误: robot_state_publisher启动失败!"
    exit 1
fi
echo "  ✓ robot_state_publisher运行中 (PID: $RSP_PID)"

# 2. 启动RViz
echo "[2/3] 启动RViz可视化..."
if [ -f "config/simulation.rviz" ]; then
    rviz2 -d config/simulation.rviz &
else
    rviz2 &
fi
RVIZ_PID=$!
sleep 2

if ! ps -p $RVIZ_PID > /dev/null; then
    echo "警告: RViz启动失败，但继续运行..."
    RVIZ_PID=""
else
    echo "  ✓ RViz运行中 (PID: $RVIZ_PID)"
fi

# 3. 启动双臂演示
echo "[3/3] 启动双臂轨迹演示..."
python3.10 dual_arm_demo.py &
DEMO_PID=$!
sleep 2

if ! ps -p $DEMO_PID > /dev/null; then
    echo "错误: dual_arm_demo启动失败!"
    kill $RSP_PID $RVIZ_PID 2>/dev/null || true
    exit 1
fi
echo "  ✓ dual_arm_demo运行中 (PID: $DEMO_PID)"

echo ""
echo "=========================================="
echo "  🚀 系统启动完成！"
echo "=========================================="
echo "运行中的进程:"
echo "  robot_state_publisher: $RSP_PID"
if [ -n "$RVIZ_PID" ]; then
    echo "  RViz:                  $RVIZ_PID"
fi
echo "  dual_arm_demo:         $DEMO_PID"
echo ""
echo "监控命令："
echo "  ros2 topic hz /robot/joint_states    # 查看发布频率"
echo "  ros2 topic echo /robot/joint_states  # 查看关节数据"
echo "  ros2 topic list                      # 查看所有话题"
echo ""
echo "RViz设置："
echo "  1. Fixed Frame: world"
echo "  2. 添加 RobotModel"
echo "  3. 添加 TF (可选)"
echo ""
echo "按 Ctrl+C 停止所有进程..."
echo "=========================================="

# 捕获Ctrl+C并清理
cleanup() {
    echo ""
    echo "停止所有进程..."
    kill $DEMO_PID 2>/dev/null || true
    kill $RSP_PID 2>/dev/null || true
    if [ -n "$RVIZ_PID" ]; then
        kill $RVIZ_PID 2>/dev/null || true
    fi
    echo "清理完成。再见！"
    exit 0
}

trap cleanup INT TERM

# 保持脚本运行
wait
