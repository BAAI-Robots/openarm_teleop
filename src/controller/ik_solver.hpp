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

#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/frames.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>

/**
 * @brief IK求解器封装类
 * 
 * 提供三种IK求解方法：
 * 1. KDL数值迭代法（基础）
 * 2. KDL LMA优化法（推荐）
 * 3. 解析解法（待实现，需要机器人特定几何参数）
 */
class IKSolver {
public:
    enum SolverType {
        KDL_NUMERICAL,  // KDL数值迭代法
        KDL_LMA,        // KDL Levenberg-Marquardt优化
        ANALYTICAL      // 解析解（未实现）
    };

    /**
     * @brief 构造函数
     * @param kdl_chain KDL运动链
     * @param solver_type 求解器类型
     */
    IKSolver(const KDL::Chain& kdl_chain, SolverType solver_type = KDL_LMA);
    
    ~IKSolver();

    /**
     * @brief 求解逆运动学
     * @param target_position 目标位置 [x, y, z]
     * @param target_orientation 目标姿态（旋转矩阵）
     * @param current_joint_angles 当前关节角度（作为初值）
     * @param solution 输出的关节角度解
     * @return 是否求解成功
     */
    bool solve(const Eigen::Vector3d& target_position,
               const Eigen::Matrix3d& target_orientation,
               const std::vector<double>& current_joint_angles,
               std::vector<double>& solution);

    /**
     * @brief 求解逆运动学（四元数版本）
     * @param target_position 目标位置 [x, y, z]
     * @param target_quaternion 目标姿态（四元数 [w, x, y, z]）
     * @param current_joint_angles 当前关节角度（作为初值）
     * @param solution 输出的关节角度解
     * @return 是否求解成功
     */
    bool solve_quaternion(const Eigen::Vector3d& target_position,
                          const Eigen::Quaterniond& target_quaternion,
                          const std::vector<double>& current_joint_angles,
                          std::vector<double>& solution);

    /**
     * @brief 仅求解位置IK（忽略姿态）
     * @param target_position 目标位置 [x, y, z]
     * @param current_joint_angles 当前关节角度（作为初值）
     * @param solution 输出的关节角度解
     * @return 是否求解成功
     */
    bool solve_position_only(const Eigen::Vector3d& target_position,
                             const std::vector<double>& current_joint_angles,
                             std::vector<double>& solution);

    /**
     * @brief 设置求解器参数
     * @param max_iterations 最大迭代次数
     * @param tolerance 收敛容差
     */
    void set_solver_params(int max_iterations, double tolerance);

private:
    KDL::Chain kdl_chain_;
    SolverType solver_type_;
    
    // KDL IK求解器
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_lma_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel_;
    
    // 求解器参数
    int max_iterations_ = 100;
    double tolerance_ = 1e-5;

    /**
     * @brief 四元数转KDL旋转矩阵
     */
    KDL::Rotation quaternion_to_kdl_rotation(const Eigen::Quaterniond& quat);
    
    /**
     * @brief Eigen旋转矩阵转KDL旋转矩阵
     */
    KDL::Rotation eigen_to_kdl_rotation(const Eigen::Matrix3d& R);
};
