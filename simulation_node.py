#!/usr/bin/env python3.10
"""
OpenArm Simulation Node
只使用KDL进行仿真，不连接真实硬件
订阅末端位姿命令，发布关节状态和末端位姿
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
from PyKDL import Chain, ChainFkSolverPos_recursive, ChainIkSolverPos_LMA
from PyKDL import JntArray, Frame, Vector, Rotation, Joint, Segment
from builtin_interfaces.msg import Time
import os

class OpenArmSimulation(Node):
    def __init__(self):
        super().__init__('openarm_simulation')
        
        # 参数
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('urdf_path', '/home/robot/openarm_teleop/config/openarm_v10_follower_no_hand.urdf')
        
        publish_rate = self.get_parameter('publish_rate').value
        urdf_path = self.get_parameter('urdf_path').value
        
        self.get_logger().info('=== OpenArm Simulation Node ===')
        self.get_logger().info(f'Publish Rate: {publish_rate} Hz')
        self.get_logger().info(f'URDF: {urdf_path}')
        
        # 加载并发布URDF
        self.publish_robot_description(urdf_path)
        
        # 初始化关节状态（双臂系统：左臂7个+右臂7个=14个关节）
        self.left_joint_names = [
            'openarm_left_joint1', 'openarm_left_joint2', 'openarm_left_joint3', 
            'openarm_left_joint4', 'openarm_left_joint5', 'openarm_left_joint6', 
            'openarm_left_joint7'
        ]
        self.right_joint_names = [
            'openarm_right_joint1', 'openarm_right_joint2', 'openarm_right_joint3', 
            'openarm_right_joint4', 'openarm_right_joint5', 'openarm_right_joint6', 
            'openarm_right_joint7'
        ]
        
        # 左臂：保持零位（不控制）
        self.left_joint_positions = [0.0] * 7
        
        # 右臂：稍微抬起的姿态（避免奇异点）
        self.right_joint_positions = [0.0, -0.5, 0.8, 0.0, 0.5, 0.0, 0.0]
        
        # 合并为完整的关节列表
        self.joint_names = self.left_joint_names + self.right_joint_names
        self.joint_positions = self.left_joint_positions + self.right_joint_positions
        self.joint_velocities = [0.0] * 14
        
        # 初始化KDL链（7个关节全部用于IK，无单独gripper）
        self.get_logger().info('Initializing kinematics...')
        self.kdl_chain = self.create_kdl_chain()
        self.fk_solver = ChainFkSolverPos_recursive(self.kdl_chain)
        
        # 配置IK求解器（提高成功率）
        # LMA参数：maxiter=迭代次数, eps=收敛阈值, eps_joints=关节空间收敛阈值
        self.ik_solver = ChainIkSolverPos_LMA(
            self.kdl_chain,
            maxiter=500,      # 增加迭代次数（默认500）
            eps=1e-5,         # 位置精度1e-5m = 0.01mm
            eps_joints=1e-5   # 关节精度
        )
        self.get_logger().info(f'✓ KDL chain loaded: {self.kdl_chain.getNrOfJoints()} joints (7-DOF arm)')
        self.get_logger().info('✓ IK solver: LMA (maxiter=500, eps=1e-5)')
        
        # 计算初始末端位姿（使用右臂的前7个关节）
        self.current_ee_pose = self.compute_forward_kinematics(self.right_joint_positions)
        
        # 发布者
        self.joint_state_pub = self.create_publisher(JointState, '/robot/joint_states', 10)
        self.ee_pose_pub = self.create_publisher(PoseStamped, '/robot/ee_pose', 10)
        
        # 订阅者
        self.ee_command_sub = self.create_subscription(
            PoseStamped,
            '/robot/ee_pose_command',
            self.ee_pose_command_callback,
            10
        )
        
        # 定时器
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_states)
        
        self.get_logger().info('✓ Simulation node ready!')
        self.get_logger().info('Subscribing to: /robot/ee_pose_command')
        self.get_logger().info('Publishing to: /robot/joint_states, /robot/ee_pose')
    
    def publish_robot_description(self, urdf_path):
        """发布robot_description参数供RViz使用"""
        try:
            if os.path.exists(urdf_path):
                with open(urdf_path, 'r') as f:
                    urdf_content = f.read()
                
                # 设置参数
                self.declare_parameter('robot_description', urdf_content)
                self.get_logger().info('✓ robot_description published')
            else:
                self.get_logger().warn(f'URDF file not found: {urdf_path}')
                self.get_logger().warn('RViz visualization may not work properly')
        except Exception as e:
            self.get_logger().error(f'Failed to load URDF: {e}')
    
    def create_kdl_chain(self):
        """创建KDL运动学链（OpenArm v10的7个臂关节，7-DOF冗余臂）"""
        chain = Chain()
        
        # 基于真实OpenArm v10 URDF的关节参数
        # Joint 1: Base rotation (Z axis)
        # xyz="0.0 0.0 0.0625", axis="0 0 1"
        chain.addSegment(Segment(
            Joint(Joint.RotZ),
            Frame(Vector(0, 0, 0.0625))
        ))
        
        # Joint 2: Shoulder pitch (X axis, 因为rpy有旋转)
        # xyz="-0.0301 0.0 0.06", rpy="1.57079632679 0 0", axis="-1 0 0"
        chain.addSegment(Segment(
            Joint(Joint.RotX),
            Frame(Rotation.RotX(1.5708), Vector(-0.0301, 0, 0.06))
        ))
        
        # Joint 3: Elbow rotation (Z axis)
        # xyz="0.0301 0.0 0.06625", axis="0 0 1"
        chain.addSegment(Segment(
            Joint(Joint.RotZ),
            Frame(Vector(0.0301, 0, 0.06625))
        ))
        
        # Joint 4: Wrist Y rotation (Y axis)
        # xyz="-0.0 0.0315 0.15375", axis="0 1 0"
        chain.addSegment(Segment(
            Joint(Joint.RotY),
            Frame(Vector(0, 0.0315, 0.15375))
        ))
        
        # Joint 5: Wrist Z rotation (Z axis)
        # xyz="0.0 -0.0315 0.0955", axis="0 0 1"
        chain.addSegment(Segment(
            Joint(Joint.RotZ),
            Frame(Vector(0, -0.0315, 0.0955))
        ))
        
        # Joint 6: Wrist X rotation (X axis)
        # xyz="0.0375 0.0 0.1205", axis="1 0 0"
        chain.addSegment(Segment(
            Joint(Joint.RotX),
            Frame(Vector(0.0375, 0, 0.1205))
        ))
        
        # Joint 7: Wrist final Y rotation (Y axis)
        # xyz="-0.0375 0.0 0.0", axis="0 1 0"
        chain.addSegment(Segment(
            Joint(Joint.RotY),
            Frame(Vector(-0.0375, 0, 0))
        ))
        
        return chain
    
    def compute_forward_kinematics(self, joint_positions):
        """计算正运动学（使用全部7个关节）"""
        q = JntArray(7)
        for i in range(7):
            q[i] = joint_positions[i]
        
        frame = Frame()
        self.fk_solver.JntToCart(q, frame)
        
        # 转换为PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'openarm_right_link0'  # 相对于右臂基座
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        
        # 位置
        pose_msg.pose.position.x = frame.p.x()
        pose_msg.pose.position.y = frame.p.y()
        pose_msg.pose.position.z = frame.p.z()
        
        # 姿态（转换为四元数）
        quat = frame.M.GetQuaternion()
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]
        
        return pose_msg
    
    def ee_pose_command_callback(self, msg):
        """处理末端位姿命令（使用7-DOF IK求解）"""
        # 提取目标位姿
        target_frame = Frame()
        target_frame.p = Vector(
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
        target_frame.M = Rotation.Quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )
        
        # 初始猜测（当前右臂7个关节位置）
        q_init = JntArray(7)
        for i in range(7):
            q_init[i] = self.right_joint_positions[i]
        
        # IK求解（7-DOF）
        q_out = JntArray(7)
        result = self.ik_solver.CartToJnt(q_init, target_frame, q_out)
        
        if result >= 0:  # 成功
            # 更新右臂全部7个关节位置
            for i in range(7):
                self.right_joint_positions[i] = q_out[i]
            
            # 更新末端位姿
            self.current_ee_pose = self.compute_forward_kinematics(self.right_joint_positions)
        else:
            self.get_logger().warn(f'IK failed with code: {result}', throttle_duration_sec=2.0)
    
    def publish_states(self):
        """发布关节状态和末端位姿（双臂系统）"""
        # 更新右臂的joint_positions到总列表（左臂保持零位）
        for i in range(7):
            self.joint_positions[7 + i] = self.right_joint_positions[i]
        
        # 发布关节状态（14个关节：左臂7个+右臂7个）
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = self.joint_names  # 14个关节名称
        joint_state_msg.position = self.joint_positions  # 14个位置
        joint_state_msg.velocity = self.joint_velocities  # 14个速度
        self.joint_state_pub.publish(joint_state_msg)
        
        # 发布末端位姿
        self.current_ee_pose.header.stamp = self.get_clock().now().to_msg()
        self.ee_pose_pub.publish(self.current_ee_pose)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OpenArmSimulation()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n[INFO] Shutting down simulation...')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
