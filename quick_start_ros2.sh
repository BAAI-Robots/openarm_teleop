#!/bin/bash
# Quick start script for OpenArm with ROS2 integration
# 快速启动脚本 - 带ROS2集成的OpenArm

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== OpenArm ROS2 Quick Start ===${NC}"

# 配置参数
ARM_SIDE=${1:-right_arm}
LEADER_CAN=${2:-can0}
FOLLOWER_CAN=${3:-can1}
CONTROL_MODE=${4:-unilateral}  # unilateral or bilateral

ARM_TYPE="v10"
TMPDIR="/tmp/openarm_urdf_gen"
WS_DIR="$HOME/ros2_ws"
BUILD_DIR="$HOME/openarm_teleop/build"

echo -e "${YELLOW}配置参数:${NC}"
echo "  控制模式: $CONTROL_MODE"
echo "  手臂侧面: $ARM_SIDE"
echo "  Leader CAN: $LEADER_CAN"
echo "  Follower CAN: $FOLLOWER_CAN"
echo ""

# 检查ROS2工作空间
if [ ! -d "$WS_DIR" ]; then
    echo -e "${RED}[错误] 找不到ROS2工作空间: $WS_DIR${NC}"
    exit 1
fi

# 检查openarm_description包
if [ ! -d "$WS_DIR/src/openarm_description" ]; then
    echo -e "${RED}[错误] 找不到openarm_description包${NC}"
    exit 1
fi

# Source ROS2环境
echo -e "${GREEN}[1/4] Source ROS2环境...${NC}"
source /opt/ros/humble/setup.bash
source "$WS_DIR/install/setup.bash"

# 生成URDF文件
echo -e "${GREEN}[2/4] 生成URDF文件...${NC}"
mkdir -p "$TMPDIR"

XACRO_PATH="$WS_DIR/src/openarm_description/urdf/robot/${ARM_TYPE}.urdf.xacro"
LEADER_URDF="$TMPDIR/${ARM_TYPE}_leader.urdf"
FOLLOWER_URDF="$TMPDIR/${ARM_TYPE}_follower.urdf"

if ! xacro "$XACRO_PATH" bimanual:=true -o "$LEADER_URDF"; then
    echo -e "${RED}[错误] 生成URDF文件失败${NC}"
    exit 1
fi
cp "$LEADER_URDF" "$FOLLOWER_URDF"

echo -e "${GREEN}  ✓ URDF文件已生成${NC}"
echo "    Leader:   $LEADER_URDF"
echo "    Follower: $FOLLOWER_URDF"

# 检查二进制文件
echo -e "${GREEN}[3/4] 检查编译的程序...${NC}"
if [ "$CONTROL_MODE" = "bilateral" ]; then
    BIN_PATH="$BUILD_DIR/bilateral_control"
else
    BIN_PATH="$BUILD_DIR/unilateral_control"
fi

if [ ! -f "$BIN_PATH" ]; then
    echo -e "${RED}[错误] 找不到编译的程序: $BIN_PATH${NC}"
    echo -e "${YELLOW}提示: 请先编译项目${NC}"
    echo "  cd ~/openarm_teleop/build"
    echo "  cmake .. && make -j\$(nproc)"
    exit 1
fi

echo -e "${GREEN}  ✓ 程序文件: $BIN_PATH${NC}"

# 显示ROS2话题信息
echo -e "${GREEN}[4/4] 启动程序...${NC}"
echo ""
echo -e "${YELLOW}程序运行后，可以在另一个终端查看ROS2话题:${NC}"
echo "  ros2 topic list"
echo "  ros2 topic echo /leader/joint_states"
echo "  ros2 topic echo /leader/ee_pose"
echo "  ros2 topic echo /follower/joint_states"
echo "  ros2 topic echo /follower/ee_pose"
echo ""
echo -e "${GREEN}按Ctrl+C停止程序${NC}"
echo "========================================"
echo ""

# 运行程序
"$BIN_PATH" "$LEADER_URDF" "$FOLLOWER_URDF" "$ARM_SIDE" "$LEADER_CAN" "$FOLLOWER_CAN"

# 清理
echo ""
echo -e "${GREEN}清理临时文件...${NC}"
rm -rf "$TMPDIR"

echo -e "${GREEN}程序已退出${NC}"
