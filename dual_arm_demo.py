#!/usr/bin/env python3
"""
OpenArm 双臂演示：左臂画圆，右臂画方
使用逆运动学控制，发布到ROS2进行可视化
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
from openarm_kinematics import OpenArmRightKinematics
import time

class DualArmDemo(Node):
    def __init__(self):
        super().__init__('dual_arm_demo')
        
        # 创建关节状态发布器
        self.joint_pub = self.create_publisher(JointState, '/robot/joint_states', 10)
        
        # 创建左右臂运动学对象
        self.right_arm = OpenArmRightKinematics()
        self.left_arm = OpenArmRightKinematics()  # 左臂使用相同的运动学结构
        
        # 定义初始位置（经过测试的可达位置）
        # 使用关节角度 [0.2, 0.3, 0, 0.8, 0, 0, 0] 对应的末端位置
        self.init_left_center = np.array([0.032, 0.345, 0.265])    # 左臂圆心（对称到Y正向）
        self.init_right_center = np.array([0.032, -0.345, 0.265])  # 右臂方形中心
        
        # 轨迹参数（已验证可达）
        self.circle_radius = 0.04   # 圆的半径 4cm
        self.square_size = 0.06     # 方形边长 6cm  
        self.period = 10.0          # 运动周期 10秒
        
        # 初始关节角度（已验证的可达配置）
        self.q_left = np.array([0.2, 0.3, 0.0, 0.8, 0.0, 0.0, 0.0])
        self.q_right = np.array([0.2, 0.3, 0.0, 0.8, 0.0, 0.0, 0.0])
        
        # 创建定时器 (50Hz)
        self.timer = self.create_timer(0.02, self.update_trajectory)
        
        self.start_time = time.time()
        
        self.get_logger().info('🤖 双臂演示启动!')
        self.get_logger().info('   左臂: 画圆 (半径=4cm, XZ平面)')
        self.get_logger().info('   右臂: 画方 (边长=6cm, XZ平面)')
        self.get_logger().info('   周期: 10秒')
    
    def generate_circle_point(self, t):
        """生成圆形轨迹上的点"""
        angle = 2 * np.pi * t / self.period
        x = self.init_left_center[0] + self.circle_radius * np.cos(angle)
        y = self.init_left_center[1]
        z = self.init_left_center[2] + self.circle_radius * np.sin(angle)
        return np.array([x, y, z])
    
    def generate_square_point(self, t):
        """生成方形轨迹上的点"""
        # 归一化时间到[0, 1]
        t_norm = (t % self.period) / self.period
        
        half_size = self.square_size / 2
        cx, cy, cz = self.init_right_center
        
        # 四条边，每条边占25%时间
        if t_norm < 0.25:
            # 底边：左->右
            progress = t_norm * 4
            x = cx - half_size + progress * self.square_size
            y = cy
            z = cz - half_size
        elif t_norm < 0.5:
            # 右边：下->上
            progress = (t_norm - 0.25) * 4
            x = cx + half_size
            y = cy
            z = cz - half_size + progress * self.square_size
        elif t_norm < 0.75:
            # 上边：右->左
            progress = (t_norm - 0.5) * 4
            x = cx + half_size - progress * self.square_size
            y = cy
            z = cz + half_size
        else:
            # 左边：上->下
            progress = (t_norm - 0.75) * 4
            x = cx - half_size
            y = cy
            z = cz + half_size - progress * self.square_size
        
        return np.array([x, y, z])
    
    def update_trajectory(self):
        """更新轨迹并发布关节状态"""
        current_time = time.time() - self.start_time
        
        # 生成目标位置
        target_left = self.generate_circle_point(current_time)
        target_right = self.generate_square_point(current_time)
        
        # 左臂逆运动学
        q_left_new = self.left_arm.inverse_kinematics(
            target_left, 
            q_init=self.q_left,
            max_iter=50,
            tolerance=5e-4
        )
        
        if q_left_new is not None:
            self.q_left = q_left_new
        
        # 右臂逆运动学
        q_right_new = self.right_arm.inverse_kinematics(
            target_right,
            q_init=self.q_right,
            max_iter=50,
            tolerance=5e-4
        )
        
        if q_right_new is not None:
            self.q_right = q_right_new
        
        # 发布关节状态
        self.publish_joint_states()
        
        # 每秒打印一次状态
        if int(current_time * 10) % 10 == 0:
            self.get_logger().info(
                f't={current_time:.1f}s | '
                f'左臂: [{target_left[0]:.3f}, {target_left[1]:.3f}, {target_left[2]:.3f}] | '
                f'右臂: [{target_right[0]:.3f}, {target_right[1]:.3f}, {target_right[2]:.3f}]'
            )
    
    def publish_joint_states(self):
        """发布14个关节的状态 (左臂7个 + 右臂7个)"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 关节名称 (左臂7个 + 右臂7个)
        msg.name = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3',
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6',
            'openarm_left_joint7',
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3',
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6',
            'openarm_right_joint7'
        ]
        
        # 关节角度
        msg.position = list(self.q_left) + list(self.q_right)
        
        # 速度和力矩（可选，设为0）
        msg.velocity = [0.0] * 14
        msg.effort = [0.0] * 14
        
        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    print("=" * 70)
    print("  OpenArm 双臂轨迹演示")
    print("=" * 70)
    print("  左臂: 画圆 (XZ平面，半径4cm)")
    print("  右臂: 画方 (XZ平面，边长6cm)")
    print("  发布话题: /robot/joint_states")
    print("  可视化: 需要启动 RViz + robot_state_publisher")
    print("=" * 70)
    print("\n启动中...\n")
    
    node = DualArmDemo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\n停止演示...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
