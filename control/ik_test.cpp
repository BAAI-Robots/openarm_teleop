// Copyright 2025 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file ik_test.cpp
 * @brief IK求解器测试程序
 * 
 * 测试IK求解器的功能：
 * 1. 正运动学：关节角度 -> 末端位姿
 * 2. 逆运动学：末端位姿 -> 关节角度
 * 3. 验证FK(IK(pose)) ≈ pose
 */

#include <controller/dynamics.hpp>
#include <iostream>
#include <iomanip>

void print_pose(const Eigen::Vector3d& pos, const Eigen::Matrix3d& rot) {
    std::cout << "Position: [" << pos.x() << ", " << pos.y() << ", " << pos.z() << "]" << std::endl;
    std::cout << "Rotation matrix:" << std::endl;
    std::cout << rot << std::endl;
}

void print_joints(const std::vector<double>& joints) {
    std::cout << "Joint angles: [";
    for (size_t i = 0; i < joints.size(); ++i) {
        std::cout << std::fixed << std::setprecision(4) << joints[i];
        if (i < joints.size() - 1) std::cout << ", ";
    }
    std::cout << "]" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "======================================" << std::endl;
    std::cout << "    IK Solver Test Program" << std::endl;
    std::cout << "======================================" << std::endl;

    // 1. 初始化Dynamics和IK求解器
    std::string urdf_path = "/tmp/openarm_urdf_gen/v10_leader.urdf";
    std::string root_link = "openarm_left_link0";
    std::string leaf_link = "openarm_left_link7";

    std::cout << "\n[1] Initializing Dynamics..." << std::endl;
    Dynamics dynamics(urdf_path, root_link, leaf_link);
    if (!dynamics.Init()) {
        std::cerr << "[ERROR] Failed to initialize Dynamics!" << std::endl;
        return 1;
    }
    std::cout << "[✓] Dynamics initialized" << std::endl;

    std::cout << "\n[2] Initializing IK Solver..." << std::endl;
    if (!dynamics.InitIKSolver(true)) {
        std::cerr << "[ERROR] Failed to initialize IK Solver!" << std::endl;
        return 1;
    }
    std::cout << "[✓] IK Solver initialized (using LMA algorithm)" << std::endl;

    // 2. 测试用例1：零位姿态的IK
    std::cout << "\n[3] Test Case 1: Zero Configuration" << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::vector<double> zero_joints = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // 7个关节
    
    // 正运动学
    Eigen::Matrix3d R_zero;
    Eigen::Vector3d p_zero;
    double* joint_array = zero_joints.data();
    dynamics.GetEECordinate(joint_array, R_zero, p_zero);
    
    std::cout << "Initial joint configuration:" << std::endl;
    print_joints(zero_joints);
    std::cout << "\nForward Kinematics (FK) result:" << std::endl;
    print_pose(p_zero, R_zero);

    // 逆运动学
    std::cout << "\nSolving Inverse Kinematics..." << std::endl;
    std::vector<double> ik_solution_zero;
    bool success = dynamics.SolveIK(p_zero, R_zero, zero_joints, ik_solution_zero);
    
    if (success) {
        std::cout << "[✓] IK solved successfully!" << std::endl;
        std::cout << "IK solution:" << std::endl;
        print_joints(ik_solution_zero);
        
        // 验证
        Eigen::Matrix3d R_verify;
        Eigen::Vector3d p_verify;
        dynamics.GetEECordinate(ik_solution_zero.data(), R_verify, p_verify);
        
        double pos_error = (p_verify - p_zero).norm();
        std::cout << "\nVerification: FK(IK(pose))" << std::endl;
        print_pose(p_verify, R_verify);
        std::cout << "Position error: " << pos_error << " m" << std::endl;
    } else {
        std::cout << "[✗] IK solving failed!" << std::endl;
    }

    // 3. 测试用例2：随机关节配置
    std::cout << "\n[4] Test Case 2: Random Configuration" << std::endl;
    std::cout << "========================================" << std::endl;
    
    std::vector<double> random_joints = {0.5, -0.3, 0.2, 0.1, -0.4, 0.6, 0.2};  // 7个关节
    
    // 正运动学
    Eigen::Matrix3d R_random;
    Eigen::Vector3d p_random;
    dynamics.GetEECordinate(random_joints.data(), R_random, p_random);
    
    std::cout << "Random joint configuration:" << std::endl;
    print_joints(random_joints);
    std::cout << "\nFK result:" << std::endl;
    print_pose(p_random, R_random);

    // 逆运动学（使用零位作为初值）
    std::cout << "\nSolving IK from zero initial guess..." << std::endl;
    std::vector<double> ik_solution_random;
    success = dynamics.SolveIK(p_random, R_random, zero_joints, ik_solution_random);
    
    if (success) {
        std::cout << "[✓] IK solved!" << std::endl;
        std::cout << "IK solution:" << std::endl;
        print_joints(ik_solution_random);
        
        // 验证
        Eigen::Matrix3d R_verify2;
        Eigen::Vector3d p_verify2;
        dynamics.GetEECordinate(ik_solution_random.data(), R_verify2, p_verify2);
        
        double pos_error2 = (p_verify2 - p_random).norm();
        std::cout << "\nVerification:" << std::endl;
        print_pose(p_verify2, R_verify2);
        std::cout << "Position error: " << pos_error2 << " m" << std::endl;
    } else {
        std::cout << "[✗] IK solving failed!" << std::endl;
    }

    // 4. 测试用例3：仅位置IK（忽略姿态）
    std::cout << "\n[5] Test Case 3: Position-Only IK" << std::endl;
    std::cout << "========================================" << std::endl;
    
    Eigen::Vector3d target_position(0.3, 0.2, 0.5);
    std::cout << "Target position: [" << target_position.x() << ", " 
              << target_position.y() << ", " << target_position.z() << "]" << std::endl;
    
    std::vector<double> ik_solution_pos;
    success = dynamics.SolveIKPositionOnly(target_position, zero_joints, ik_solution_pos);
    
    if (success) {
        std::cout << "[✓] Position-only IK solved!" << std::endl;
        print_joints(ik_solution_pos);
        
        // 验证位置
        Eigen::Matrix3d R_verify3;
        Eigen::Vector3d p_verify3;
        dynamics.GetEECordinate(ik_solution_pos.data(), R_verify3, p_verify3);
        
        double pos_error3 = (p_verify3 - target_position).norm();
        std::cout << "\nAchieved position: [" << p_verify3.x() << ", " 
                  << p_verify3.y() << ", " << p_verify3.z() << "]" << std::endl;
        std::cout << "Position error: " << pos_error3 << " m" << std::endl;
    } else {
        std::cout << "[✗] Position-only IK failed!" << std::endl;
    }

    // 5. 测试四元数接口
    std::cout << "\n[6] Test Case 4: Quaternion Interface" << std::endl;
    std::cout << "========================================" << std::endl;
    
    Eigen::Quaterniond target_quat(0.707, 0.0, 0.707, 0.0);  // 90度绕Y轴旋转
    Eigen::Vector3d target_pos_quat(0.4, 0.0, 0.3);
    
    std::cout << "Target pose (quaternion):" << std::endl;
    std::cout << "  Position: [" << target_pos_quat.x() << ", " 
              << target_pos_quat.y() << ", " << target_pos_quat.z() << "]" << std::endl;
    std::cout << "  Quaternion: [w=" << target_quat.w() << ", x=" << target_quat.x() 
              << ", y=" << target_quat.y() << ", z=" << target_quat.z() << "]" << std::endl;
    
    std::vector<double> ik_solution_quat;
    success = dynamics.SolveIKQuaternion(target_pos_quat, target_quat, zero_joints, ik_solution_quat);
    
    if (success) {
        std::cout << "[✓] Quaternion IK solved!" << std::endl;
        print_joints(ik_solution_quat);
        
        // 验证
        Eigen::Matrix3d R_verify4;
        Eigen::Vector3d p_verify4;
        dynamics.GetEECordinate(ik_solution_quat.data(), R_verify4, p_verify4);
        
        double pos_error4 = (p_verify4 - target_pos_quat).norm();
        std::cout << "\nVerification:" << std::endl;
        print_pose(p_verify4, R_verify4);
        std::cout << "Position error: " << pos_error4 << " m" << std::endl;
    } else {
        std::cout << "[✗] Quaternion IK failed!" << std::endl;
    }

    std::cout << "\n======================================" << std::endl;
    std::cout << "    Test Completed" << std::endl;
    std::cout << "======================================" << std::endl;

    return 0;
}
