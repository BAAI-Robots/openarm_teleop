#!/usr/bin/env python3
"""
VR控制客户端示例

该脚本展示如何在VR端通过ROS2控制OpenArm机器人。

使用方法：
1. 确保机器人端已启动: ./build/vr_control_example
2. 运行此脚本: python3 vr_client_example.py
3. 选择控制模式（关节控制、末端控制、夹爪控制）

依赖：
    sudo apt install ros-humble-rclpy python3-geometry-msgs python3-sensor-msgs
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import time
import math


class VRControlClient(Node):
    """VR控制客户端 - 发送命令到机器人"""
    
    def __init__(self):
        super().__init__('vr_control_client')
        
        # 创建发布器 - 控制机器人
        self.joint_cmd_pub = self.create_publisher(
            JointState, '/robot/joint_command', 10)
        
        self.ee_pose_cmd_pub = self.create_publisher(
            PoseStamped, '/robot/ee_pose_command', 10)
        
        self.gripper_cmd_pub = self.create_publisher(
            Float64MultiArray, '/robot/gripper_command', 10)
        
        # 创建订阅器 - 接收机器人状态
        self.joint_state_sub = self.create_subscription(
            JointState, '/robot/joint_states', 
            self.joint_state_callback, 10)
        
        self.ee_pose_sub = self.create_subscription(
            PoseStamped, '/robot/ee_pose', 
            self.ee_pose_callback, 10)
        
        self.latest_joint_state = None
        self.latest_ee_pose = None
        
        self.get_logger().info('VR Control Client initialized')
        self.get_logger().info('Waiting for robot state...')
    
    def joint_state_callback(self, msg):
        """接收关节状态反馈"""
        self.latest_joint_state = msg
        # 取消注释以查看详细状态
        # self.get_logger().info(f'Received joint states: {len(msg.position)} joints')
    
    def ee_pose_callback(self, msg):
        """接收末端位姿反馈"""
        self.latest_ee_pose = msg
        # 取消注释以查看详细状态
        # pos = msg.pose.position
        # self.get_logger().info(f'EE pose: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]')
    
    def send_joint_command(self, joint_positions):
        """
        发送关节控制命令
        
        Args:
            joint_positions: 关节角度列表 [j1, j2, j3, j4, j5, j6, gripper]
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = joint_positions
        
        self.joint_cmd_pub.publish(msg)
        self.get_logger().info(f'Sent joint command: {joint_positions}')
    
    def send_ee_pose_command(self, x, y, z, qw=1.0, qx=0.0, qy=0.0, qz=0.0):
        """
        发送末端位姿控制命令（自动IK求解）
        
        Args:
            x, y, z: 目标位置 (米)
            qw, qx, qy, qz: 目标姿态四元数
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        msg.pose.orientation.w = qw
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        
        self.ee_pose_cmd_pub.publish(msg)
        self.get_logger().info(f'Sent EE pose: pos=[{x:.3f}, {y:.3f}, {z:.3f}]')
    
    def send_gripper_command(self, gripper_value):
        """
        发送夹爪控制命令
        
        Args:
            gripper_value: 0.0(闭合) ~ 1.0(打开)
        """
        msg = Float64MultiArray()
        msg.data = [gripper_value]
        
        self.gripper_cmd_pub.publish(msg)
        self.get_logger().info(f'Sent gripper command: {gripper_value}')
    
    def wait_for_robot(self, timeout=5.0):
        """等待机器人状态连接"""
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_joint_state is not None:
                self.get_logger().info('✓ Robot connection established!')
                return True
        
        self.get_logger().warning('⚠ Timeout waiting for robot state')
        return False


def demo_joint_control(client):
    """演示1：关节控制"""
    print("\n=== Demo 1: Joint Control ===")
    print("Moving to zero position...")
    
    # 零位
    client.send_joint_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(2)
    
    # 示例动作
    print("Moving to example position...")
    client.send_joint_command([0.5, -0.3, 0.2, 0.1, -0.2, 0.3, 0.0])
    time.sleep(2)
    
    print("Returning to zero...")
    client.send_joint_command([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    time.sleep(2)


def demo_ee_pose_control(client):
    """演示2：末端位姿控制（自动IK）"""
    print("\n=== Demo 2: End-Effector Pose Control (with IK) ===")
    
    # 位姿1
    print("Moving to pose 1...")
    client.send_ee_pose_command(0.3, 0.2, 0.4, qw=1.0, qx=0.0, qy=0.0, qz=0.0)
    time.sleep(3)
    
    # 位姿2
    print("Moving to pose 2...")
    client.send_ee_pose_command(0.35, 0.15, 0.45, qw=0.707, qx=0.0, qy=0.707, qz=0.0)
    time.sleep(3)
    
    # 位姿3
    print("Moving to pose 3...")
    client.send_ee_pose_command(0.25, 0.25, 0.4, qw=1.0, qx=0.0, qy=0.0, qz=0.0)
    time.sleep(3)


def demo_gripper_control(client):
    """演示3：夹爪控制"""
    print("\n=== Demo 3: Gripper Control ===")
    
    print("Opening gripper...")
    client.send_gripper_command(1.0)  # 完全打开
    time.sleep(1)
    
    print("Closing gripper...")
    client.send_gripper_command(0.0)  # 完全闭合
    time.sleep(1)
    
    print("Half open...")
    client.send_gripper_command(0.5)  # 半开
    time.sleep(1)


def demo_continuous_trajectory(client):
    """演示4：连续轨迹（圆形运动）"""
    print("\n=== Demo 4: Continuous Circular Trajectory ===")
    
    center_x, center_y, center_z = 0.3, 0.2, 0.4
    radius = 0.05
    duration = 5.0  # 秒
    rate = 50  # Hz
    
    print(f"Drawing a circle for {duration} seconds...")
    
    start_time = time.time()
    while time.time() - start_time < duration:
        t = time.time() - start_time
        angle = 2 * math.pi * t / duration
        
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)
        z = center_z
        
        client.send_ee_pose_command(x, y, z)
        time.sleep(1.0 / rate)


def interactive_mode(client):
    """交互式控制模式"""
    print("\n=== Interactive Mode ===")
    print("Commands:")
    print("  j - Joint control")
    print("  e - End-effector pose control")
    print("  g - Gripper control")
    print("  s - Show current state")
    print("  q - Quit")
    
    while True:
        cmd = input("\nEnter command: ").strip().lower()
        
        if cmd == 'q':
            break
        elif cmd == 's':
            if client.latest_joint_state:
                print(f"Joint positions: {client.latest_joint_state.position}")
            if client.latest_ee_pose:
                pos = client.latest_ee_pose.pose.position
                print(f"EE position: [{pos.x:.3f}, {pos.y:.3f}, {pos.z:.3f}]")
        elif cmd == 'j':
            print("Enter 7 joint angles (separated by space):")
            try:
                angles = [float(x) for x in input().split()]
                if len(angles) == 7:
                    client.send_joint_command(angles)
                else:
                    print("Error: Need exactly 7 values")
            except ValueError:
                print("Error: Invalid input")
        elif cmd == 'e':
            print("Enter target position (x y z):")
            try:
                x, y, z = [float(x) for x in input().split()]
                client.send_ee_pose_command(x, y, z)
            except ValueError:
                print("Error: Invalid input")
        elif cmd == 'g':
            print("Enter gripper value (0.0-1.0):")
            try:
                value = float(input().strip())
                client.send_gripper_command(value)
            except ValueError:
                print("Error: Invalid input")
        else:
            print("Unknown command")


def main():
    print("=== OpenArm VR Control Client ===")
    print("Make sure robot side is running: ./build/vr_control_example\n")
    
    rclpy.init()
    
    try:
        client = VRControlClient()
        
        # 等待机器人连接
        if not client.wait_for_robot():
            print("Failed to connect to robot. Please check:")
            print("  1. Robot program is running")
            print("  2. ROS2 network is configured correctly")
            return
        
        # 菜单
        while True:
            print("\n" + "="*50)
            print("Select demo mode:")
            print("  1 - Joint Control Demo")
            print("  2 - End-Effector Pose Control Demo (with IK)")
            print("  3 - Gripper Control Demo")
            print("  4 - Continuous Trajectory Demo")
            print("  5 - Interactive Mode")
            print("  0 - Exit")
            print("="*50)
            
            choice = input("Enter choice: ").strip()
            
            if choice == '0':
                break
            elif choice == '1':
                demo_joint_control(client)
            elif choice == '2':
                demo_ee_pose_control(client)
            elif choice == '3':
                demo_gripper_control(client)
            elif choice == '4':
                demo_continuous_trajectory(client)
            elif choice == '5':
                interactive_mode(client)
            else:
                print("Invalid choice")
            
            # 处理ROS2回调
            rclpy.spin_once(client, timeout_sec=0.1)
        
        print("\nShutting down...")
        client.destroy_node()
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
