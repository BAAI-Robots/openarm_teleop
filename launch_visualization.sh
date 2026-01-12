#!/bin/bash

echo "======================================================================"
echo "  OpenArm 双臂可视化系统启动脚本"
echo "======================================================================"

# 设置ROS2环境
source ~/ros2_ws/install/setup.bash

# 检查dual_arm_demo.py是否正在运行
if pgrep -f "dual_arm_demo.py" > /dev/null; then
    echo "✓ dual_arm_demo.py 已在运行"
else
    echo "✗ dual_arm_demo.py 未运行，请先启动它"
    exit 1
fi

# 启动robot_state_publisher
echo ""
echo "正在启动 robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher \
    --ros-args \
    -p robot_description:="$(cat /home/robot/openarm_teleop/config/openarm_v10_follower_no_hand.urdf)" \
    -r joint_states:=/robot/joint_states &

RSP_PID=$!
sleep 2

# 检查robot_state_publisher是否成功启动
if ps -p $RSP_PID > /dev/null; then
    echo "✓ robot_state_publisher 启动成功 (PID: $RSP_PID)"
else
    echo "✗ robot_state_publisher 启动失败"
    exit 1
fi

# 启动RViz
echo ""
echo "正在启动 RViz2..."
echo "======================================================================"
echo "  RViz 配置说明:"
echo "  1. Fixed Frame 设置为: world"
echo "  2. 添加 RobotModel 显示"
echo "  3. 观察左臂画圆、右臂画方"
echo "======================================================================"

rviz2

# 清理
echo ""
echo "关闭可视化系统..."
kill $RSP_PID 2>/dev/null
echo "完成！"
