#!/usr/bin/env python3
"""测试双臂演示的初始位置是否可达"""

import numpy as np
from openarm_kinematics import OpenArmRightKinematics

def test_initial_positions():
    robot = OpenArmRightKinematics()
    
    print("=" * 70)
    print("测试双臂演示的初始位置")
    print("=" * 70)
    
    # 左臂圆心位置
    left_center = np.array([0.15, 0.0, 0.35])
    print(f"\n左臂圆心位置: {left_center}")
    q_left = robot.inverse_kinematics(left_center, max_iter=200)
    if q_left is not None:
        print(f"✓ 左臂初始位置可达")
        print(f"  关节角度: {np.degrees(q_left)}")
        T, _ = robot.forward_kinematics(q_left)
        pos, _ = robot.get_position_orientation(T)
        error = np.linalg.norm(pos - left_center)
        print(f"  验证位置: {pos}")
        print(f"  位置误差: {error:.6f} 米")
    else:
        print("✗ 左臂初始位置不可达，需要调整")
    
    # 右臂方形中心
    right_center = np.array([0.15, 0.0, 0.35])
    print(f"\n右臂方形中心: {right_center}")
    q_right = robot.inverse_kinematics(right_center, max_iter=200)
    if q_right is not None:
        print(f"✓ 右臂初始位置可达")
        print(f"  关节角度: {np.degrees(q_right)}")
        T, _ = robot.forward_kinematics(q_right)
        pos, _ = robot.get_position_orientation(T)
        error = np.linalg.norm(pos - right_center)
        print(f"  验证位置: {pos}")
        print(f"  位置误差: {error:.6f} 米")
    else:
        print("✗ 右臂初始位置不可达，需要调整")
    
    # 测试圆形轨迹的几个点
    print("\n" + "=" * 70)
    print("测试左臂圆形轨迹 (半径=0.06m, XZ平面)")
    print("=" * 70)
    radius = 0.06
    angles = [0, np.pi/2, np.pi, 3*np.pi/2]
    all_reachable = True
    
    for i, angle in enumerate(angles):
        x = left_center[0] + radius * np.cos(angle)
        y = left_center[1]
        z = left_center[2] + radius * np.sin(angle)
        target = np.array([x, y, z])
        
        q = robot.inverse_kinematics(target, q_init=q_left if q_left is not None else None, max_iter=100)
        if q is not None:
            print(f"✓ 点{i+1} [{x:.3f}, {y:.3f}, {z:.3f}] 可达")
        else:
            print(f"✗ 点{i+1} [{x:.3f}, {y:.3f}, {z:.3f}] 不可达")
            all_reachable = False
    
    # 测试方形轨迹的四个角
    print("\n" + "=" * 70)
    print("测试右臂方形轨迹 (边长=0.08m, XZ平面)")
    print("=" * 70)
    half_size = 0.04
    corners = [
        [right_center[0] - half_size, right_center[1], right_center[2] - half_size],
        [right_center[0] + half_size, right_center[1], right_center[2] - half_size],
        [right_center[0] + half_size, right_center[1], right_center[2] + half_size],
        [right_center[0] - half_size, right_center[1], right_center[2] + half_size],
    ]
    
    for i, corner in enumerate(corners):
        target = np.array(corner)
        q = robot.inverse_kinematics(target, q_init=q_right if q_right is not None else None, max_iter=100)
        if q is not None:
            print(f"✓ 角{i+1} [{corner[0]:.3f}, {corner[1]:.3f}, {corner[2]:.3f}] 可达")
        else:
            print(f"✗ 角{i+1} [{corner[0]:.3f}, {corner[1]:.3f}, {corner[2]:.3f}] 不可达")
            all_reachable = False
    
    print("\n" + "=" * 70)
    if all_reachable and q_left is not None and q_right is not None:
        print("✓ 所有轨迹点均可达！可以启动演示。")
    else:
        print("⚠ 部分点不可达，可能需要调整轨迹参数。")
    print("=" * 70)

if __name__ == "__main__":
    test_initial_positions()
