#!/usr/bin/env python3.10

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')
        self.sub = self.create_subscription(
            JointState, '/robot/joint_states', self.callback, 10)
        self.count = 0
        
    def callback(self, msg):
        self.count += 1
        if self.count == 1:
            print(f"\n发布的关节数量: {len(msg.name)}")
            print(f"关节名称: {msg.name}")
            print(f"关节位置: {[f'{p:.3f}' for p in msg.position]}")
            print("\n详细信息:")
            for name, pos in zip(msg.name, msg.position):
                print(f"  {name}: {pos:.4f} rad ({pos*180/3.14159:.1f}°)")
            rclpy.shutdown()

def main():
    rclpy.init()
    node = DebugNode()
    print("等待关节状态...")
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()
