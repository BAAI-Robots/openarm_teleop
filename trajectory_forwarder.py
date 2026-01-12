#!/usr/bin/env python3
"""
轨迹转发节点 - 订阅 MoveIt 的命令话题并转发到实际的控制器
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from moveit_msgs.msg import DisplayTrajectory
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory


class TrajectoryForwarder(Node):
    def __init__(self):
        super().__init__('trajectory_forwarder')
        
        # 使用可重入回调组
        self.callback_group = ReentrantCallbackGroup()
        
        # 创建 action 客户端连接到实际的控制器
        self.left_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/left_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        self.right_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/right_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=self.callback_group
        )
        
        # 订阅 MoveIt 发布的显示轨迹
        self.display_traj_sub = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.display_trajectory_callback,
            10,
            callback_group=self.callback_group
        )
        
        # 标志位用于自动执行
        self.auto_execute = True
        
        self.get_logger().info('Trajectory Forwarder Node Started')
        self.get_logger().info('Listening to /display_planned_path...')
        self.get_logger().info('Waiting for controller action servers...')
        
        # 等待控制器 action 服务器
        self.left_client.wait_for_server(timeout_sec=5.0)
        self.right_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info('Connected to controller action servers!')
    
    def display_trajectory_callback(self, msg):
        """处理来自 MoveIt 的 DisplayTrajectory 消息"""
        if not self.auto_execute:
            return
            
        self.get_logger().info(f'Received planned trajectory with {len(msg.trajectory)} robot trajectories')
        
        # 遍历所有机器人轨迹
        for robot_traj in msg.trajectory:
            for joint_traj in robot_traj.joint_trajectory:
                if len(joint_traj.points) == 0:
                    continue
                    
                joint_names = joint_traj.joint_names
                
                # 检查是左臂还是右臂
                if any('left' in name for name in joint_names):
                    self.get_logger().info(f'Executing left arm trajectory with {len(joint_traj.points)} points')
                    self.send_trajectory(self.left_client, joint_traj, 'left')
                    
                elif any('right' in name for name in joint_names):
                    self.get_logger().info(f'Executing right arm trajectory with {len(joint_traj.points)} points')
                    self.send_trajectory(self.right_client, joint_traj, 'right')
    
    def send_trajectory(self, client, joint_traj, arm_name):
        """发送轨迹到控制器"""
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = joint_traj
        
        future = client.send_goal_async(goal_msg)
        future.add_done_callback(lambda f: self.goal_response_callback(f, arm_name))
    
    def goal_response_callback(self, future, arm_name):
        """处理目标响应"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'{arm_name} arm goal rejected!')
            return
        
        self.get_logger().info(f'{arm_name} arm goal accepted, executing...')
        
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f: self.goal_result_callback(f, arm_name))
    
    def goal_result_callback(self, future, arm_name):
        """处理执行结果"""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'{arm_name} arm trajectory execution succeeded!')
        else:
            self.get_logger().error(f'{arm_name} arm trajectory execution failed with status: {status}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryForwarder()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
