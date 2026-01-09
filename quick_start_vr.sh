#!/bin/bash
# OpenArm VR控制快速启动脚本

set -e

echo "======================================="
echo "  OpenArm VR Control - Quick Start"
echo "======================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# 检查ROS2环境
check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        echo -e "${YELLOW}[INFO] Sourcing ROS2 environment...${NC}"
        source /opt/ros/humble/setup.bash
    fi
    echo -e "${GREEN}✓ ROS2 environment: $ROS_DISTRO${NC}"
}

# 配置CAN总线
setup_can() {
    echo ""
    echo -e "${YELLOW}[INFO] Configuring CAN interfaces...${NC}"
    
    # CAN0
    if ip link show can0 > /dev/null 2>&1; then
        sudo ip link set can0 down 2>/dev/null || true
        sudo ip link set can0 type can bitrate 1000000
        sudo ip link set can0 up
        echo -e "${GREEN}✓ CAN0 configured (1Mbps)${NC}"
    else
        echo -e "${RED}✗ CAN0 interface not found${NC}"
    fi
    
    # CAN1 (optional)
    if ip link show can1 > /dev/null 2>&1; then
        sudo ip link set can1 down 2>/dev/null || true
        sudo ip link set can1 type can bitrate 1000000
        sudo ip link set can1 up
        echo -e "${GREEN}✓ CAN1 configured (1Mbps)${NC}"
    else
        echo -e "${YELLOW}⚠ CAN1 interface not found (optional)${NC}"
    fi
}

# 生成URDF文件
generate_urdf() {
    echo ""
    echo -e "${YELLOW}[INFO] Generating URDF files...${NC}"
    
    URDF_DIR="/tmp/openarm_urdf_gen"
    mkdir -p "$URDF_DIR"
    
    if [ -f "$HOME/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro" ]; then
        xacro "$HOME/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro" \
              bimanual:=true \
              -o "$URDF_DIR/v10_leader.urdf"
        
        if [ -f "$URDF_DIR/v10_leader.urdf" ]; then
            echo -e "${GREEN}✓ URDF generated: $URDF_DIR/v10_leader.urdf${NC}"
        else
            echo -e "${RED}✗ Failed to generate URDF${NC}"
            exit 1
        fi
    else
        echo -e "${RED}✗ xacro file not found${NC}"
        echo "Please install openarm_description package"
        exit 1
    fi
}

# 编译项目
build_project() {
    echo ""
    echo -e "${YELLOW}[INFO] Building project...${NC}"
    
    cd /home/robot/openarm_teleop/build
    
    if [ ! -f "Makefile" ]; then
        echo -e "${YELLOW}Running cmake...${NC}"
        cmake ..
    fi
    
    make vr_control_example
    
    if [ -f "vr_control_example" ]; then
        echo -e "${GREEN}✓ vr_control_example built successfully${NC}"
    else
        echo -e "${RED}✗ Build failed${NC}"
        exit 1
    fi
}

# 主菜单
main_menu() {
    echo ""
    echo "======================================="
    echo "  Select Mode"
    echo "======================================="
    echo "1) Start VR Control (Robot Side)"
    echo "2) Start VR Client (Python Demo)"
    echo "3) Test IK Solver"
    echo "4) Setup Only (CAN + URDF)"
    echo "5) Run Full Test Suite"
    echo "0) Exit"
    echo "======================================="
    echo ""
    read -p "Enter choice: " choice
    
    case $choice in
        1)
            start_vr_control
            ;;
        2)
            start_vr_client
            ;;
        3)
            test_ik_solver
            ;;
        4)
            echo -e "${GREEN}Setup completed!${NC}"
            ;;
        5)
            run_full_test
            ;;
        0)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid choice${NC}"
            main_menu
            ;;
    esac
}

# 启动VR控制（机器人端）
start_vr_control() {
    echo ""
    echo "======================================="
    echo "  Starting VR Control (Robot Side)"
    echo "======================================="
    echo ""
    echo "This will:"
    echo "  - Initialize OpenArm hardware"
    echo "  - Start 500Hz control loop"
    echo "  - Publish robot state at 50Hz"
    echo "  - Listen for VR commands"
    echo ""
    echo "ROS2 Topics:"
    echo "  Publishing:"
    echo "    - /robot/joint_states (50Hz)"
    echo "    - /robot/ee_pose (50Hz)"
    echo ""
    echo "  Subscribing:"
    echo "    - /robot/joint_command"
    echo "    - /robot/ee_pose_command"
    echo "    - /robot/gripper_command"
    echo ""
    echo -e "${YELLOW}Press Ctrl+C to stop${NC}"
    echo ""
    read -p "Press Enter to continue..."
    
    cd /home/robot/openarm_teleop/build
    ./vr_control_example
}

# 启动VR客户端（Python示例）
start_vr_client() {
    echo ""
    echo "======================================="
    echo "  Starting VR Client (Python Demo)"
    echo "======================================="
    echo ""
    echo "Make sure robot side is running first!"
    echo "  Terminal 1: ./quick_start_vr.sh -> Select option 1"
    echo "  Terminal 2: ./quick_start_vr.sh -> Select option 2"
    echo ""
    read -p "Press Enter to continue..."
    
    cd /home/robot/openarm_teleop
    python3 vr_client_example.py
}

# 测试IK求解器
test_ik_solver() {
    echo ""
    echo "======================================="
    echo "  Testing IK Solver"
    echo "======================================="
    echo ""
    
    cd /home/robot/openarm_teleop/build
    ./ik_test
    
    echo ""
    echo -e "${GREEN}IK test completed${NC}"
    read -p "Press Enter to return to menu..."
    main_menu
}

# 运行完整测试套件
run_full_test() {
    echo ""
    echo "======================================="
    echo "  Running Full Test Suite"
    echo "======================================="
    echo ""
    
    cd /home/robot/openarm_teleop
    
    if [ -f "test_vr_interface.sh" ]; then
        chmod +x test_vr_interface.sh
        ./test_vr_interface.sh
    else
        echo -e "${RED}✗ test_vr_interface.sh not found${NC}"
    fi
    
    echo ""
    read -p "Press Enter to return to menu..."
    main_menu
}

# 主流程
main() {
    # 步骤1：检查ROS2环境
    check_ros2
    
    # 步骤2：配置CAN总线
    setup_can
    
    # 步骤3：生成URDF
    generate_urdf
    
    # 步骤4：编译项目
    build_project
    
    # 步骤5：显示菜单
    main_menu
}

# 执行
main
