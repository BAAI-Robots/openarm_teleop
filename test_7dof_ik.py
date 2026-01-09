#!/usr/bin/env python3.10
"""测试7-DOF IK求解器"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
import time

class IKTester(Node):
    def __init__(self):
        super().__init__('ik_tester')
        
        # 订阅末端位姿反馈
        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            '/robot/ee_pose',
            self.ee_pose_callback,
            10
        )
        
        # 发布目标位姿
        self.ee_command_pub = self.create_publisher(
            PoseStamped,
            '/robot/ee_pose_command',
            10
        )
        
        self.current_ee_pose = None
        self.test_count = 0
        self.success_count = 0
        
        # 等待初始位姿
        self.get_logger().info('等待机器人初始位姿...')
        time.sleep(2)
        
        # 开始测试
        self.timer = self.create_timer(2.0, self.test_ik)
        
    def ee_pose_callback(self, msg):
        """接收当前末端位姿"""
        self.current_ee_pose = msg
        
    def test_ik(self):
        """测试IK求解"""
        if self.current_ee_pose is None:
            return
            
        self.test_count += 1
        
        # 创建测试目标（在当前位置附近小范围移动）
        target = PoseStamped()
        target.header.frame_id = 'openarm_right_link0'
        target.header.stamp = self.get_clock().now().to_msg()
        
        # 添加小偏移
        offset_x = 0.05 * math.sin(self.test_count * 0.5)
        offset_y = 0.05 * math.cos(self.test_count * 0.5)
        offset_z = 0.02 * math.sin(self.test_count * 0.3)
        
        target.pose.position.x = self.current_ee_pose.pose.position.x + offset_x
        target.pose.position.y = self.current_ee_pose.pose.position.y + offset_y
        target.pose.position.z = self.current_ee_pose.pose.position.z + offset_z
        
        # 保持姿态不变
        target.pose.orientation = self.current_ee_pose.pose.orientation
        
        # 发布命令
        self.ee_command_pub.publish(target)
        self.success_count += 1
        
        self.get_logger().info(
            f'测试 {self.test_count}: 目标位置 '
            f'[{target.pose.position.x:.3f}, {target.pose.position.y:.3f}, {target.pose.position.z:.3f}]'
        )
        
        if self.test_count >= 20:
            self.get_logger().info(f'\n测试完成！共 {self.test_count} 次，成功发送 {self.success_count} 个命令')
            self.get_logger().info('请检查终端是否有IK失败警告')
            rclpy.shutdown()

def main():
    rclpy.init()
    node = IKTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == '__main__':
    main()
