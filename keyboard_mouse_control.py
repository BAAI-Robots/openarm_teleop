#!/usr/bin/env python3.10
"""
键盘鼠标控制OpenArm - 类Minecraft操控体验

控制方式：
    点击窗口     - 进入控制模式（鼠标锁定）
    ESC         - 释放鼠标（退出控制模式）
    W/S         - 前/后移动
    A/D         - 左/右移动
    Shift/Space - 下/上移动
    Q/E         - Roll左旋/右旋
    鼠标移动     - Pitch和Yaw控制（锁定时）
    R           - 重置位姿
    G           - 打开/关闭夹爪
    Ctrl+C      - 退出程序

依赖安装：
    sudo apt install python3-pygame
    pip3 install pygame

使用方法：
    # 终端1：启动机器人端
    cd /home/robot/openarm_teleop/build
    ./vr_control_example

    # 终端2：启动键鼠控制
    python3 keyboard_mouse_control.py

    # 终端3（可选）：启动RViz可视化
    ros2 launch openarm_description display.launch.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import pygame
import math
import sys
import time
from threading import Lock


class KeyboardMouseController(Node):
    """键盘鼠标控制器节点"""
    
    def __init__(self):
        super().__init__('keyboard_mouse_controller')
        
        # 创建发布器
        self.ee_pose_pub = self.create_publisher(
            PoseStamped, '/robot/ee_pose_command', 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, '/robot/gripper_command', 10)
        
        # 订阅当前状态
        self.ee_pose_sub = self.create_subscription(
            PoseStamped, '/robot/ee_pose',
            self.ee_pose_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/robot/joint_states',
            self.joint_state_callback, 10)
        
        # 当前末端位姿（世界坐标系）
        self.current_position = [0.3, 0.2, 0.4]  # [x, y, z]
        self.current_orientation = [0.0, 0.0, 0.0]  # [roll, pitch, yaw] (欧拉角)
        self.pose_lock = Lock()
        
        # 控制参数
        self.move_speed = 0.002  # 米/帧
        self.rotate_speed = 0.02  # 弧度/帧
        self.mouse_sensitivity = 0.001
        
        # 夹爪状态
        self.gripper_open = True
        self.gripper_value = 1.0  # 0.0=闭合, 1.0=打开
        
        # 统计信息
        self.commands_sent = 0
        self.last_ee_pose = None
        self.robot_connected = False
        
        # 鼠标控制状态
        self.mouse_grabbed = False
        
        self.get_logger().info('Keyboard Mouse Controller initialized')
        self.get_logger().info('Waiting for robot connection...')
    
    def ee_pose_callback(self, msg):
        """接收末端位姿反馈"""
        self.last_ee_pose = msg
        if not self.robot_connected:
            self.robot_connected = True
            self.get_logger().info('✓ Robot connected!')
            
            # 初始化为当前位姿
            with self.pose_lock:
                self.current_position = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                ]
                # 从四元数转换到欧拉角
                q = msg.pose.orientation
                self.current_orientation = self.quaternion_to_euler(
                    q.w, q.x, q.y, q.z)
    
    def joint_state_callback(self, msg):
        """接收关节状态反馈"""
        pass  # 仅用于检测连接
    
    def quaternion_to_euler(self, w, x, y, z):
        """四元数转欧拉角 (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """欧拉角转四元数 (w, x, y, z)"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (w, x, y, z)
    
    def send_ee_pose(self):
        """发送末端位姿命令"""
        with self.pose_lock:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            
            # 设置位置
            msg.pose.position.x = self.current_position[0]
            msg.pose.position.y = self.current_position[1]
            msg.pose.position.z = self.current_position[2]
            
            # 设置姿态（欧拉角转四元数）
            qw, qx, qy, qz = self.euler_to_quaternion(
                self.current_orientation[0],
                self.current_orientation[1],
                self.current_orientation[2]
            )
            msg.pose.orientation.w = qw
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            
            self.ee_pose_pub.publish(msg)
            self.commands_sent += 1
    
    def send_gripper_command(self):
        """发送夹爪命令"""
        msg = Float64MultiArray()
        msg.data = [self.gripper_value]
        self.gripper_pub.publish(msg)
    
    def toggle_gripper(self):
        """切换夹爪开/关"""
        self.gripper_open = not self.gripper_open
        self.gripper_value = 1.0 if self.gripper_open else 0.0
        self.send_gripper_command()
    
    def reset_pose(self):
        """重置到初始位姿"""
        with self.pose_lock:
            self.current_position = [0.3, 0.2, 0.4]
            self.current_orientation = [0.0, 0.0, 0.0]
        self.send_ee_pose()
        self.get_logger().info('Pose reset to default')


def draw_ui(screen, controller, mouse_dx, mouse_dy, mouse_grabbed):
    """绘制UI界面"""
    font = pygame.font.Font(None, 24)
    small_font = pygame.font.Font(None, 20)
    
    # 背景
    screen.fill((20, 20, 30))
    
    # 标题
    title = font.render("OpenArm - Keyboard Mouse Control", True, (100, 200, 255))
    screen.blit(title, (20, 20))
    
    # 连接状态
    if controller.robot_connected:
        status = small_font.render("Status: Connected ✓", True, (100, 255, 100))
    else:
        status = small_font.render("Status: Waiting for robot...", True, (255, 200, 100))
    screen.blit(status, (20, 60))
    
    # 当前位姿
    y_offset = 100
    pos = controller.current_position
    ori = controller.current_orientation
    
    pose_title = font.render("Current Pose:", True, (255, 255, 255))
    screen.blit(pose_title, (20, y_offset))
    y_offset += 35
    
    position_text = small_font.render(
        f"Position: X={pos[0]:.3f}  Y={pos[1]:.3f}  Z={pos[2]:.3f}", 
        True, (200, 200, 200))
    screen.blit(position_text, (30, y_offset))
    y_offset += 25
    
    orientation_text = small_font.render(
        f"Orientation: Roll={math.degrees(ori[0]):.1f}°  "
        f"Pitch={math.degrees(ori[1]):.1f}°  Yaw={math.degrees(ori[2]):.1f}°",
        True, (200, 200, 200))
    screen.blit(orientation_text, (30, y_offset))
    y_offset += 40
    
    # 夹爪状态
    gripper_state = "Open" if controller.gripper_open else "Closed"
    gripper_color = (100, 255, 100) if controller.gripper_open else (255, 100, 100)
    gripper_text = small_font.render(f"Gripper: {gripper_state}", True, gripper_color)
    screen.blit(gripper_text, (20, y_offset))
    y_offset += 40
    
    # 鼠标控制状态提示
    if mouse_grabbed:
        mouse_status_text = font.render("🔒 MOUSE LOCKED", True, (100, 255, 100))
        hint_text = small_font.render("(Press ESC to release)", True, (150, 150, 150))
    else:
        mouse_status_text = font.render("🖱️  MOUSE FREE", True, (255, 200, 100))
        hint_text = small_font.render("(Click window to control)", True, (150, 150, 150))
    screen.blit(mouse_status_text, (20, y_offset))
    y_offset += 30
    screen.blit(hint_text, (30, y_offset))
    y_offset += 50
    
    # 控制提示
    controls_title = font.render("Controls:", True, (255, 255, 255))
    screen.blit(controls_title, (20, y_offset))
    y_offset += 35
    
    controls = [
        "W/S - Forward/Backward",
        "A/D - Left/Right",
        "Shift/Space - Down/Up",
        "Q/E - Roll Left/Right",
        "Mouse - Pitch & Yaw (when locked)",
        "G - Toggle Gripper",
        "R - Reset Pose",
        "ESC - Release Mouse"
    ]
    
    for control in controls:
        text = small_font.render(control, True, (180, 180, 180))
        screen.blit(text, (30, y_offset))
        y_offset += 22
    
    # 统计信息
    y_offset = 500
    stats_title = font.render("Statistics:", True, (255, 255, 255))
    screen.blit(stats_title, (20, y_offset))
    y_offset += 30
    
    stats_text = small_font.render(
        f"Commands sent: {controller.commands_sent}", True, (200, 200, 200))
    screen.blit(stats_text, (30, y_offset))
    
    # 鼠标移动指示器（仅在锁定时显示）
    if mouse_grabbed and (abs(mouse_dx) > 0 or abs(mouse_dy) > 0):
        mouse_text = small_font.render(
            f"Mouse: dx={mouse_dx:.3f}, dy={mouse_dy:.3f}", 
            True, (255, 255, 100))
        screen.blit(mouse_text, (400, 60))
    
    pygame.display.flip()


def main():
    """主函数"""
    print("=" * 60)
    print("  OpenArm Keyboard Mouse Control")
    print("=" * 60)
    print("\nInitializing...")
    
    # 初始化ROS2
    rclpy.init()
    controller = KeyboardMouseController()
    
    # 初始化Pygame
    pygame.init()
    screen = pygame.display.set_mode((800, 600))
    pygame.display.set_caption("OpenArm Keyboard Mouse Control")
    clock = pygame.time.Clock()
    
    # 鼠标控制状态
    mouse_grabbed = False
    pygame.mouse.set_visible(True)
    pygame.event.set_grab(False)
    
    print("\n✓ Initialization complete!")
    print("\nWaiting for robot connection...")
    print("Make sure robot side is running: ./build/vr_control_example\n")
    
    # 等待机器人连接
    timeout = 10.0
    start_time = time.time()
    while not controller.robot_connected:
        rclpy.spin_once(controller, timeout_sec=0.1)
        if time.time() - start_time > timeout:
            print("⚠ Timeout waiting for robot!")
            print("You can still use the controller, but commands won't be sent.")
            break
    
    if controller.robot_connected:
        print("✓ Robot connected! You can now control the robot.\n")
    
    # 主循环
    running = True
    mouse_dx = 0.0
    mouse_dy = 0.0
    screen_center = (400, 300)
    
    print("\n" + "="*60)
    print("  💡 TIP: Click the window to enter control mode")
    print("  💡 Press ESC to release mouse (not exit)")
    print("="*60 + "\n")
    
    try:
        while running:
            # 处理事件
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    # 点击窗口进入控制模式
                    if not mouse_grabbed:
                        mouse_grabbed = True
                        pygame.mouse.set_visible(False)
                        pygame.event.set_grab(True)
                        pygame.mouse.set_pos(screen_center)
                        print("🔒 Mouse LOCKED - Control mode active")
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        # ESC释放鼠标
                        if mouse_grabbed:
                            mouse_grabbed = False
                            pygame.mouse.set_visible(True)
                            pygame.event.set_grab(False)
                            print("🖱️  Mouse RELEASED - Click window to re-enter")
                        else:
                            # 如果鼠标未锁定，ESC退出程序
                            running = False
                    elif event.key == pygame.K_r:
                        controller.reset_pose()
                    elif event.key == pygame.K_g:
                        controller.toggle_gripper()
            
            # 获取键盘状态
            keys = pygame.key.get_pressed()
            
            # 获取鼠标移动（仅在锁定时生效）
            mouse_dx = 0.0
            mouse_dy = 0.0
            if mouse_grabbed:
                mouse_pos = pygame.mouse.get_pos()
                mouse_dx = (mouse_pos[0] - screen_center[0]) * controller.mouse_sensitivity
                mouse_dy = (mouse_pos[1] - screen_center[1]) * controller.mouse_sensitivity
                # 重置鼠标到中心
                pygame.mouse.set_pos(screen_center)
            
            # 控制位置
            moved = False
            with controller.pose_lock:
                # W/S - 前后
                if keys[pygame.K_w]:
                    controller.current_position[0] += controller.move_speed
                    moved = True
                if keys[pygame.K_s]:
                    controller.current_position[0] -= controller.move_speed
                    moved = True
                
                # A/D - 左右
                if keys[pygame.K_a]:
                    controller.current_position[1] += controller.move_speed
                    moved = True
                if keys[pygame.K_d]:
                    controller.current_position[1] -= controller.move_speed
                    moved = True
                
                # Shift/Space - 上下
                if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                    controller.current_position[2] -= controller.move_speed
                    moved = True
                if keys[pygame.K_SPACE]:
                    controller.current_position[2] += controller.move_speed
                    moved = True
                
                # Q/E - Roll
                if keys[pygame.K_q]:
                    controller.current_orientation[0] += controller.rotate_speed
                    moved = True
                if keys[pygame.K_e]:
                    controller.current_orientation[0] -= controller.rotate_speed
                    moved = True
                
                # 鼠标控制 Pitch 和 Yaw（仅在锁定时生效）
                if mouse_grabbed:
                    if abs(mouse_dx) > 0.0001:
                        controller.current_orientation[2] += mouse_dx  # Yaw
                        moved = True
                    if abs(mouse_dy) > 0.0001:
                        controller.current_orientation[1] += mouse_dy  # Pitch
                        moved = True
                
                # 限制范围
                # 位置限制
                controller.current_position[0] = max(-0.6, min(0.6, controller.current_position[0]))
                controller.current_position[1] = max(-0.6, min(0.6, controller.current_position[1]))
                controller.current_position[2] = max(0.1, min(0.8, controller.current_position[2]))
                
                # 姿态限制
                for i in range(3):
                    while controller.current_orientation[i] > math.pi:
                        controller.current_orientation[i] -= 2 * math.pi
                    while controller.current_orientation[i] < -math.pi:
                        controller.current_orientation[i] += 2 * math.pi
            
            # 如果有移动，发送命令
            if moved:
                controller.send_ee_pose()
            
            # 绘制UI
            draw_ui(screen, controller, mouse_dx, mouse_dy, mouse_grabbed)
            
            # 处理ROS2回调
            rclpy.spin_once(controller, timeout_sec=0.001)
            
            # 控制帧率
            clock.tick(60)  # 60 FPS
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        print("\nShutting down...")
        pygame.quit()
        controller.destroy_node()
        rclpy.shutdown()
        print("✓ Shutdown complete")


if __name__ == '__main__':
    main()
