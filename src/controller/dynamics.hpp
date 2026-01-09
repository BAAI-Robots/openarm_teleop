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

#pragma once
#include <string.h>
#include <unistd.h>
#include <urdf_parser/urdf_parser.h>

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <sstream>
#include <vector>

// 前向声明IK求解器
class IKSolver;

/*
 * Compute gravity and inertia compensation using Orocos
 * Kinematics and Dynamics Library (KDL).
 */
class Dynamics {
private:
    std::shared_ptr<urdf::ModelInterface> urdf_model_interface;

    std::string urdf_path;
    std::string start_link;
    std::string end_link;

    KDL::JntSpaceInertiaMatrix inertia_matrix;
    KDL::JntArray q;
    KDL::JntArray q_d;
    KDL::JntArray coriolis_forces;
    KDL::JntArray gravity_forces;

    KDL::JntArray biasangle;

    KDL::Tree kdl_tree;
    KDL::Chain kdl_chain;
    std::unique_ptr<KDL::ChainDynParam> solver;
    
    // IK求解器（可选）
    std::unique_ptr<IKSolver> ik_solver_;

public:
    Dynamics(std::string urdf_path, std::string start_link, std::string end_link);
    ~Dynamics();

    bool Init();
    void GetGravity(const double *motor_position, double *gravity);
    void GetCoriolis(const double *motor_position, const double *motor_velocity, double *coriolis);
    void GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag);

    void GetJacobian(const double *motor_position, Eigen::MatrixXd &jacobian);

    void GetNullSpace(const double *motor_positon, Eigen::MatrixXd &nullspace);

    void GetNullSpaceTauSpace(const double *motor_position, Eigen::MatrixXd &nullspace_T);

    void GetEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);

    void GetPreEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);
    
    /**
     * @brief 初始化IK求解器
     * @param use_lma 是否使用LMA优化算法（true）还是数值迭代法（false）
     * @return 是否成功初始化
     */
    bool InitIKSolver(bool use_lma = true);
    
    /**
     * @brief 求解逆运动学（位置+姿态）
     * @param target_position 目标位置 [x, y, z]
     * @param target_orientation 目标姿态（旋转矩阵）
     * @param current_joint_angles 当前关节角度（作为初值）
     * @param solution 输出的关节角度解
     * @return 是否求解成功
     */
    bool SolveIK(const Eigen::Vector3d& target_position,
                 const Eigen::Matrix3d& target_orientation,
                 const std::vector<double>& current_joint_angles,
                 std::vector<double>& solution);
    
    /**
     * @brief 求解逆运动学（四元数版本）
     * @param target_position 目标位置 [x, y, z]
     * @param target_quat 目标姿态（四元数 [w, x, y, z]）
     * @param current_joint_angles 当前关节角度（作为初值）
     * @param solution 输出的关节角度解
     * @return 是否求解成功
     */
    bool SolveIKQuaternion(const Eigen::Vector3d& target_position,
                           const Eigen::Quaterniond& target_quat,
                           const std::vector<double>& current_joint_angles,
                           std::vector<double>& solution);
    
    /**
     * @brief 仅求解位置IK（忽略姿态）
     * @param target_position 目标位置 [x, y, z]
     * @param current_joint_angles 当前关节角度（作为初值）
     * @param solution 输出的关节角度解
     * @return 是否求解成功
     */
    bool SolveIKPositionOnly(const Eigen::Vector3d& target_position,
                             const std::vector<double>& current_joint_angles,
                             std::vector<double>& solution);
};
