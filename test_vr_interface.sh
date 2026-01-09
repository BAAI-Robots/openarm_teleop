#!/bin/bash
# VR控制接口测试脚本

source /opt/ros/humble/setup.bash

echo "======================================"
echo "  VR Control Interface Test Script"
echo "======================================"
echo ""

# 检查话题是否存在
echo "[1] Checking if VR control program is running..."
if ros2 topic list | grep -q "/robot/joint_command"; then
    echo "✅ VR control program is running"
else
    echo "❌ VR control program NOT running"
    echo "   Please start: ./build/vr_control_example can0 /tmp/openarm_urdf_gen/v10_leader.urdf right_arm"
    exit 1
fi

echo ""
echo "[2] Available topics:"
ros2 topic list | grep "/robot"

echo ""
echo "[3] Testing joint command..."
ros2 topic pub --once /robot/joint_command sensor_msgs/msg/JointState \
"{
  name: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'gripper_joint'],
  position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
}" && echo "✅ Joint command sent"

sleep 1

echo ""
echo "[4] Testing gripper command..."
ros2 topic pub --once /robot/gripper_command std_msgs/msg/Float64MultiArray \
"{data: [0.5]}" && echo "✅ Gripper command sent"

sleep 1

echo ""
echo "[5] Checking robot state..."
echo "Joint states (first message):"
ros2 topic echo /robot/joint_states --once

echo ""
echo "[6] Checking EE pose..."
echo "EE pose (first message):"
ros2 topic echo /robot/ee_pose --once

echo ""
echo "======================================"
echo "  Test Complete!"
echo "======================================"
echo ""
echo "To monitor continuously:"
echo "  ros2 topic echo /robot/joint_states"
echo "  ros2 topic echo /robot/ee_pose"
