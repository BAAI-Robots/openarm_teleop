#!/usr/bin/env python3
"""
OpenArm 7-DOF 机械臂运动学库
基于URDF文件提取的右臂参数，实现DH参数计算、正运动学和逆运动学
"""

import numpy as np
from typing import Tuple, Optional, List
from dataclasses import dataclass

@dataclass
class DHParameter:
    """DH参数 (修正DH表示法)"""
    a: float      # 连杆长度 (沿x_{i-1}轴)
    alpha: float  # 连杆扭角 (绕x_{i-1}轴旋转)
    d: float      # 连杆偏距 (沿z_i轴)
    theta: float  # 关节角 (绕z_i轴旋转，变量)

class OpenArmRightKinematics:
    """
    OpenArm右臂7自由度运动学
    
    从URDF提取的关节配置:
    - Joint1: 绕Z轴旋转 (肩部旋转)
    - Joint2: 绕-X轴旋转 (肩部俯仰)
    - Joint3: 绕Z轴旋转 (肘部旋转)
    - Joint4: 绕Y轴旋转 (肘部俯仰)
    - Joint5: 绕Z轴旋转 (腕部旋转)
    - Joint6: 绕X轴旋转 (腕部俯仰)
    - Joint7: 绕Y轴旋转 (腕部偏航)
    """
    
    def __init__(self):
        """初始化OpenArm右臂运动学参数"""
        
        # 从URDF提取的连杆长度 (单位: 米)
        # Right arm joint offsets from URDF
        self.L0 = 0.0625   # link0 -> link1: z offset
        self.L1 = 0.06     # link1 -> link2: z offset  
        self.L2 = 0.0301   # link1 -> link2: x offset
        self.L3 = 0.06625  # link2 -> link3: z offset
        self.L4 = 0.15375  # link3 -> link4: z offset
        self.L5 = 0.0315   # link3 -> link4: y offset
        self.L6 = 0.0955   # link4 -> link5: z offset
        self.L7 = 0.1205   # link5 -> link6: z offset
        self.L8 = 0.0375   # link5 -> link6: x offset
        
        # 关节限位 (单位: 弧度)
        self.joint_limits = [
            (-1.396263, 3.490659),      # Joint 1
            (-0.174533, 3.316125),      # Joint 2
            (-1.570796, 1.570796),      # Joint 3
            (0.0, 2.443461),            # Joint 4
            (-1.570796, 1.570796),      # Joint 5
            (-0.785398, 0.785398),      # Joint 6
            (-1.570796, 1.570796),      # Joint 7
        ]
        
        # 计算DH参数表
        self.dh_params = self._compute_dh_parameters()
        
    def _compute_dh_parameters(self) -> List[DHParameter]:
        """
        计算7-DOF机械臂的DH参数表 (修正DH表示法)
        
        修正DH参数的定义:
        - a_i: 沿x_i从z_i到z_{i+1}的距离
        - alpha_i: 绕x_i从z_i到z_{i+1}的角度
        - d_i: 沿z_i从x_{i-1}到x_i的距离
        - theta_i: 绕z_i从x_{i-1}到x_i的角度
        
        基于URDF右臂关节配置:
        Joint1: origin="0 0 0.0625", axis="0 0 1"
        Joint2: origin="-0.0301 0 0.06", rpy="1.5708 0 0", axis="-1 0 0"
        Joint3: origin="0.0301 0 0.06625", axis="0 0 1"
        Joint4: origin="0 0.0315 0.15375", axis="0 1 0"
        Joint5: origin="0 -0.0315 0.0955", axis="0 0 1"
        Joint6: origin="0.0375 0 0.1205", axis="1 0 0"
        Joint7: origin="-0.0375 0 0", axis="0 1 0"
        """
        
        dh = []
        
        # Joint 1: 肩部旋转 (绕Z轴)
        # 从base到link1: z=0.0625, 绕Z轴旋转
        dh.append(DHParameter(
            a=0.0,
            alpha=0.0,
            d=self.L0,  # 0.0625
            theta=0.0   # 变量
        ))
        
        # Joint 2: 肩部俯仰 (绕-X轴, 经过rpy=1.5708)
        # 从link1到link2: x=-0.0301, z=0.06, 坐标系旋转90度
        dh.append(DHParameter(
            a=self.L2,      # 0.0301 (沿新x轴)
            alpha=np.pi/2,  # 90度扭转
            d=self.L1,      # 0.06 (沿z轴)
            theta=0.0       # 变量 (但轴是-X,需要后续处理)
        ))
        
        # Joint 3: 肘部旋转 (绕Z轴)
        # 从link2到link3: x=0.0301, z=0.06625
        dh.append(DHParameter(
            a=self.L2,      # 0.0301
            alpha=0.0,
            d=self.L3,      # 0.06625
            theta=0.0       # 变量
        ))
        
        # Joint 4: 肘部俯仰 (绕Y轴)
        # 从link3到link4: y=0.0315, z=0.15375
        dh.append(DHParameter(
            a=self.L5,      # 0.0315 (在XY平面)
            alpha=-np.pi/2, # -90度 (Y轴旋转需要坐标系变换)
            d=self.L4,      # 0.15375
            theta=0.0       # 变量
        ))
        
        # Joint 5: 腕部旋转 (绕Z轴)
        # 从link4到link5: y=-0.0315, z=0.0955
        dh.append(DHParameter(
            a=0.0,
            alpha=np.pi/2,  # 90度扭转
            d=self.L6,      # 0.0955
            theta=0.0       # 变量
        ))
        
        # Joint 6: 腕部俯仰 (绕X轴)
        # 从link5到link6: x=0.0375, z=0.1205
        dh.append(DHParameter(
            a=self.L8,      # 0.0375
            alpha=np.pi/2,  # 90度扭转
            d=self.L7,      # 0.1205
            theta=0.0       # 变量
        ))
        
        # Joint 7: 腕部偏航 (绕Y轴)
        # 从link6到link7: x=-0.0375
        dh.append(DHParameter(
            a=self.L8,      # 0.0375
            alpha=-np.pi/2, # -90度
            d=0.0,
            theta=0.0       # 变量
        ))
        
        return dh
    
    def dh_transform(self, dh: DHParameter, theta: float) -> np.ndarray:
        """
        计算单个DH参数对应的齐次变换矩阵 (修正DH表示法)
        
        T = Rot(Z, theta) * Trans(0, 0, d) * Trans(a, 0, 0) * Rot(X, alpha)
        
        Args:
            dh: DH参数
            theta: 关节角度 (弧度)
            
        Returns:
            4x4齐次变换矩阵
        """
        ct = np.cos(theta + dh.theta)
        st = np.sin(theta + dh.theta)
        ca = np.cos(dh.alpha)
        sa = np.sin(dh.alpha)
        
        T = np.array([
            [ct, -st*ca,  st*sa, dh.a*ct],
            [st,  ct*ca, -ct*sa, dh.a*st],
            [0,   sa,     ca,    dh.d   ],
            [0,   0,      0,     1      ]
        ])
        
        return T
    
    def forward_kinematics(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """
        正运动学: 从关节角度计算末端执行器位姿
        
        Args:
            joint_angles: 7个关节角度 [q1, q2, q3, q4, q5, q6, q7] (弧度)
            
        Returns:
            T_end: 末端执行器相对于基座的4x4齐次变换矩阵
            T_list: 每个关节的累积变换矩阵列表 (用于可视化)
        """
        assert len(joint_angles) == 7, "需要7个关节角度"
        
        # 检查关节限位
        for i, angle in enumerate(joint_angles):
            lower, upper = self.joint_limits[i]
            if angle < lower or angle > upper:
                print(f"警告: Joint {i+1} 超出限位: {angle:.3f} (限位: [{lower:.3f}, {upper:.3f}])")
        
        # 累积变换
        T = np.eye(4)
        T_list = [T.copy()]
        
        for i, (dh, angle) in enumerate(zip(self.dh_params, joint_angles)):
            # Joint2的轴是-X, 需要反转角度
            if i == 1:
                angle = -angle
            
            T_i = self.dh_transform(dh, angle)
            T = T @ T_i
            T_list.append(T.copy())
        
        return T, T_list
    
    def get_position_orientation(self, T: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        从齐次变换矩阵提取位置和姿态
        
        Args:
            T: 4x4齐次变换矩阵
            
        Returns:
            position: [x, y, z] 位置 (米)
            euler_angles: [roll, pitch, yaw] 欧拉角 (弧度, ZYX顺序)
        """
        position = T[:3, 3]
        
        # 从旋转矩阵提取ZYX欧拉角
        R = T[:3, :3]
        
        # 防止万向锁
        sy = np.sqrt(R[0, 0]**2 + R[1, 0]**2)
        
        if sy > 1e-6:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0
        
        return position, np.array([roll, pitch, yaw])
    
    def inverse_kinematics(
        self, 
        target_pos: np.ndarray, 
        target_orient: Optional[np.ndarray] = None,
        q_init: Optional[np.ndarray] = None,
        max_iter: int = 100,
        tolerance: float = 1e-4
    ) -> Optional[np.ndarray]:
        """
        逆运动学: 从目标位姿计算关节角度 (数值解法 - 雅可比转置法)
        
        对于7-DOF冗余机械臂,使用迭代数值方法求解
        
        Args:
            target_pos: 目标位置 [x, y, z] (米)
            target_orient: 目标姿态 [roll, pitch, yaw] (弧度), None表示只考虑位置
            q_init: 初始关节角度猜测 (弧度), None使用零位
            max_iter: 最大迭代次数
            tolerance: 收敛容差 (米/弧度)
            
        Returns:
            关节角度 [q1, ..., q7] (弧度), 或None (无解)
        """
        # 初始化
        if q_init is None:
            q = np.zeros(7)
        else:
            q = q_init.copy()
        
        # 目标位姿
        target_pose = target_pos
        pose_dim = 3
        
        if target_orient is not None:
            # 如果指定姿态,扩展为6维任务空间
            target_pose = np.concatenate([target_pos, target_orient])
            pose_dim = 6
        
        # 迭代求解
        for iteration in range(max_iter):
            # 正运动学
            T_current, _ = self.forward_kinematics(q)
            current_pos, current_orient = self.get_position_orientation(T_current)
            
            # 计算误差
            if target_orient is None:
                error = target_pos - current_pos
            else:
                pos_error = target_pos - current_pos
                orient_error = target_orient - current_orient
                # 归一化角度到[-pi, pi]
                orient_error = np.arctan2(np.sin(orient_error), np.cos(orient_error))
                error = np.concatenate([pos_error, orient_error])
            
            # 检查收敛
            if np.linalg.norm(error) < tolerance:
                return q
            
            # 计算雅可比矩阵 (数值方法)
            J = self._compute_jacobian(q, pose_dim)
            
            # 使用阻尼最小二乘法 (Damped Least Squares)
            lambda_damping = 0.01
            delta_q = J.T @ np.linalg.inv(J @ J.T + lambda_damping**2 * np.eye(pose_dim)) @ error
            
            # 更新关节角度
            alpha = 0.5  # 步长因子
            q = q + alpha * delta_q
            
            # 关节限位投影
            for i in range(7):
                lower, upper = self.joint_limits[i]
                q[i] = np.clip(q[i], lower, upper)
        
        # 未收敛
        print(f"警告: 逆运动学未收敛, 最终误差: {np.linalg.norm(error):.6f}")
        return None
    
    def _compute_jacobian(self, q: np.ndarray, pose_dim: int = 3) -> np.ndarray:
        """
        数值计算雅可比矩阵 J = dX/dq
        
        Args:
            q: 当前关节角度
            pose_dim: 任务空间维度 (3=位置, 6=位置+姿态)
            
        Returns:
            雅可比矩阵 (pose_dim x 7)
        """
        epsilon = 1e-6
        J = np.zeros((pose_dim, 7))
        
        # 当前位姿
        T0, _ = self.forward_kinematics(q)
        pos0, orient0 = self.get_position_orientation(T0)
        
        if pose_dim == 3:
            x0 = pos0
        else:
            x0 = np.concatenate([pos0, orient0])
        
        # 对每个关节求偏导
        for i in range(7):
            q_delta = q.copy()
            q_delta[i] += epsilon
            
            T_delta, _ = self.forward_kinematics(q_delta)
            pos_delta, orient_delta = self.get_position_orientation(T_delta)
            
            if pose_dim == 3:
                x_delta = pos_delta
            else:
                x_delta = np.concatenate([pos_delta, orient_delta])
            
            J[:, i] = (x_delta - x0) / epsilon
        
        return J
    
    def print_dh_table(self):
        """打印DH参数表"""
        print("\n=== OpenArm Right Arm DH Parameters (Modified DH Convention) ===")
        print(f"{'Joint':<8} {'a (m)':<12} {'alpha (rad)':<15} {'d (m)':<12} {'theta (rad)':<15}")
        print("-" * 70)
        
        for i, dh in enumerate(self.dh_params, 1):
            print(f"Joint {i:<3} {dh.a:<12.6f} {dh.alpha:<15.6f} {dh.d:<12.6f} {dh.theta:<15.6f} (var)")
        
        print("\n关节限位 (弧度 / 度):")
        for i, (lower, upper) in enumerate(self.joint_limits, 1):
            print(f"  Joint {i}: [{lower:.6f}, {upper:.6f}] rad = [{np.degrees(lower):.2f}°, {np.degrees(upper):.2f}°]")


def test_kinematics():
    """测试运动学功能"""
    print("=" * 70)
    print("OpenArm Right Arm 7-DOF 运动学测试")
    print("=" * 70)
    
    # 创建运动学对象
    robot = OpenArmRightKinematics()
    
    # 打印DH参数表
    robot.print_dh_table()
    
    # 测试1: 正运动学 (零位)
    print("\n" + "=" * 70)
    print("测试1: 正运动学 (零位姿态)")
    print("=" * 70)
    q_zero = np.zeros(7)
    T_end, _ = robot.forward_kinematics(q_zero)
    pos, orient = robot.get_position_orientation(T_end)
    
    print(f"关节角度: {np.degrees(q_zero)} (度)")
    print(f"末端位置: {pos} (米)")
    print(f"末端姿态: {np.degrees(orient)} (度)")
    print(f"\n齐次变换矩阵:\n{T_end}")
    
    # 测试2: 正运动学 (随机姿态)
    print("\n" + "=" * 70)
    print("测试2: 正运动学 (示例姿态)")
    print("=" * 70)
    q_test = np.array([0.5, 0.3, 0.2, 0.8, 0.1, 0.2, 0.3])
    T_end, _ = robot.forward_kinematics(q_test)
    pos, orient = robot.get_position_orientation(T_end)
    
    print(f"关节角度: {np.degrees(q_test)} (度)")
    print(f"末端位置: {pos} (米)")
    print(f"末端姿态: {np.degrees(orient)} (度)")
    
    # 测试3: 逆运动学 (仅位置)
    print("\n" + "=" * 70)
    print("测试3: 逆运动学 (位置控制)")
    print("=" * 70)
    target_pos = np.array([0.3, 0.0, 0.5])
    print(f"目标位置: {target_pos} (米)")
    
    q_ik = robot.inverse_kinematics(target_pos, q_init=q_zero, max_iter=200)
    
    if q_ik is not None:
        print(f"求解成功!")
        print(f"关节角度: {np.degrees(q_ik)} (度)")
        
        # 验证
        T_verify, _ = robot.forward_kinematics(q_ik)
        pos_verify, _ = robot.get_position_orientation(T_verify)
        error = np.linalg.norm(pos_verify - target_pos)
        print(f"实际位置: {pos_verify} (米)")
        print(f"位置误差: {error:.6f} (米)")
    else:
        print("求解失败!")
    
    # 测试4: 逆运动学 (位置+姿态)
    print("\n" + "=" * 70)
    print("测试4: 逆运动学 (位置+姿态控制)")
    print("=" * 70)
    target_pos = np.array([0.25, 0.1, 0.45])
    target_orient = np.array([0.0, np.pi/6, 0.0])  # 俯仰30度
    print(f"目标位置: {target_pos} (米)")
    print(f"目标姿态: {np.degrees(target_orient)} (度)")
    
    q_ik = robot.inverse_kinematics(target_pos, target_orient, q_init=q_zero, max_iter=300)
    
    if q_ik is not None:
        print(f"求解成功!")
        print(f"关节角度: {np.degrees(q_ik)} (度)")
        
        # 验证
        T_verify, _ = robot.forward_kinematics(q_ik)
        pos_verify, orient_verify = robot.get_position_orientation(T_verify)
        pos_error = np.linalg.norm(pos_verify - target_pos)
        orient_error = np.linalg.norm(orient_verify - target_orient)
        print(f"实际位置: {pos_verify} (米)")
        print(f"实际姿态: {np.degrees(orient_verify)} (度)")
        print(f"位置误差: {pos_error:.6f} (米)")
        print(f"姿态误差: {orient_error:.6f} (弧度)")
    else:
        print("求解失败!")
    
    print("\n" + "=" * 70)
    print("测试完成!")
    print("=" * 70)


if __name__ == "__main__":
    test_kinematics()
