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

#include "ik_solver.hpp"
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <iostream>

IKSolver::IKSolver(const KDL::Chain& kdl_chain, SolverType solver_type)
    : kdl_chain_(kdl_chain), solver_type_(solver_type) {
    
    switch (solver_type_) {
        case KDL_LMA:
            // Levenberg-Marquardt优化算法IK求解器（推荐）
            ik_solver_lma_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
            std::cout << "[IKSolver] Using KDL LMA solver" << std::endl;
            break;
            
        case KDL_NUMERICAL:
            // 数值迭代法IK求解器（需要速度求解器）
            ik_solver_vel_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_chain_);
            std::cout << "[IKSolver] Using KDL Numerical solver" << std::endl;
            break;
            
        case ANALYTICAL:
            std::cout << "[IKSolver] Analytical solver not implemented yet" << std::endl;
            break;
    }
}

IKSolver::~IKSolver() {}

void IKSolver::set_solver_params(int max_iterations, double tolerance) {
    max_iterations_ = max_iterations;
    tolerance_ = tolerance;
}

KDL::Rotation IKSolver::quaternion_to_kdl_rotation(const Eigen::Quaterniond& quat) {
    // 将Eigen四元数转换为KDL旋转矩阵
    Eigen::Matrix3d R = quat.toRotationMatrix();
    return eigen_to_kdl_rotation(R);
}

KDL::Rotation IKSolver::eigen_to_kdl_rotation(const Eigen::Matrix3d& R) {
    return KDL::Rotation(
        R(0, 0), R(0, 1), R(0, 2),
        R(1, 0), R(1, 1), R(1, 2),
        R(2, 0), R(2, 1), R(2, 2)
    );
}

bool IKSolver::solve(const Eigen::Vector3d& target_position,
                     const Eigen::Matrix3d& target_orientation,
                     const std::vector<double>& current_joint_angles,
                     std::vector<double>& solution) {
    
    const size_t n_joints = kdl_chain_.getNrOfJoints();
    
    if (current_joint_angles.size() != n_joints) {
        std::cerr << "[IKSolver] Error: current_joint_angles size mismatch! Expected "
                  << n_joints << ", got " << current_joint_angles.size() << std::endl;
        return false;
    }
    
    // 构造KDL目标位姿
    KDL::Frame target_frame;
    target_frame.p = KDL::Vector(target_position.x(), target_position.y(), target_position.z());
    target_frame.M = eigen_to_kdl_rotation(target_orientation);
    
    // 当前关节角度（作为初始猜测）
    KDL::JntArray q_init(n_joints);
    for (size_t i = 0; i < n_joints; ++i) {
        q_init(i) = current_joint_angles[i];
    }
    
    // 求解结果
    KDL::JntArray q_out(n_joints);
    
    int ret = -1;
    
    switch (solver_type_) {
        case KDL_LMA:
            if (ik_solver_lma_) {
                ret = ik_solver_lma_->CartToJnt(q_init, target_frame, q_out);
            }
            break;
            
        case KDL_NUMERICAL: {
            if (ik_solver_vel_) {
                // 使用Newton-Raphson迭代法
                KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
                KDL::ChainIkSolverPos_NR ik_solver_nr(kdl_chain_, fk_solver, *ik_solver_vel_, 
                                                       max_iterations_, tolerance_);
                ret = ik_solver_nr.CartToJnt(q_init, target_frame, q_out);
            }
            break;
        }
            
        case ANALYTICAL:
            std::cerr << "[IKSolver] Analytical solver not implemented!" << std::endl;
            return false;
    }
    
    if (ret < 0) {
        std::cerr << "[IKSolver] IK solving failed with error code: " << ret << std::endl;
        return false;
    }
    
    // 复制结果
    solution.resize(n_joints);
    for (size_t i = 0; i < n_joints; ++i) {
        solution[i] = q_out(i);
    }
    
    return true;
}

bool IKSolver::solve_quaternion(const Eigen::Vector3d& target_position,
                                 const Eigen::Quaterniond& target_quaternion,
                                 const std::vector<double>& current_joint_angles,
                                 std::vector<double>& solution) {
    
    // 将四元数转换为旋转矩阵
    Eigen::Matrix3d R = target_quaternion.toRotationMatrix();
    
    // 调用旋转矩阵版本的求解函数
    return solve(target_position, R, current_joint_angles, solution);
}

bool IKSolver::solve_position_only(const Eigen::Vector3d& target_position,
                                    const std::vector<double>& current_joint_angles,
                                    std::vector<double>& solution) {
    
    // 使用当前姿态作为目标姿态（仅关注位置）
    const size_t n_joints = kdl_chain_.getNrOfJoints();
    
    if (current_joint_angles.size() != n_joints) {
        std::cerr << "[IKSolver] Error: current_joint_angles size mismatch!" << std::endl;
        return false;
    }
    
    // 先正运动学获取当前末端姿态
    KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
    KDL::JntArray q_current(n_joints);
    for (size_t i = 0; i < n_joints; ++i) {
        q_current(i) = current_joint_angles[i];
    }
    
    KDL::Frame current_frame;
    if (fk_solver.JntToCart(q_current, current_frame) < 0) {
        std::cerr << "[IKSolver] FK failed in solve_position_only!" << std::endl;
        return false;
    }
    
    // 使用目标位置 + 当前姿态
    KDL::Frame target_frame;
    target_frame.p = KDL::Vector(target_position.x(), target_position.y(), target_position.z());
    target_frame.M = current_frame.M;  // 保持当前姿态
    
    // 求解IK
    KDL::JntArray q_out(n_joints);
    int ret = -1;
    
    switch (solver_type_) {
        case KDL_LMA:
            if (ik_solver_lma_) {
                ret = ik_solver_lma_->CartToJnt(q_current, target_frame, q_out);
            }
            break;
            
        case KDL_NUMERICAL: {
            if (ik_solver_vel_) {
                KDL::ChainIkSolverPos_NR ik_solver_nr(kdl_chain_, fk_solver, *ik_solver_vel_, 
                                                       max_iterations_, tolerance_);
                ret = ik_solver_nr.CartToJnt(q_current, target_frame, q_out);
            }
            break;
        }
            
        case ANALYTICAL:
            std::cerr << "[IKSolver] Analytical solver not implemented!" << std::endl;
            return false;
    }
    
    if (ret < 0) {
        std::cerr << "[IKSolver] IK solving failed in position-only mode!" << std::endl;
        return false;
    }
    
    // 复制结果
    solution.resize(n_joints);
    for (size_t i = 0; i < n_joints; ++i) {
        solution[i] = q_out(i);
    }
    
    return true;
}
