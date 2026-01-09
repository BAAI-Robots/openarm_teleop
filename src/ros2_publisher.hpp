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

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>

namespace openarm {
namespace ros2 {

/**
 * @brief ROS2发布器，用于发布机器人关节状态和末端执行器位姿
 * 
 * 该类封装了ROS2节点，负责将机器人状态发布到ROS2话题：
 * - /joint_states: 发布关节位置、速度、力矩
 * - /ee_pose: 发布末端执行器的位姿（位置和姿态）
 */
class RobotStatePublisher {
public:
    /**
     * @brief 构造函数
     * @param node_name ROS2节点名称
     * @param robot_name 机器人名称（如"leader", "follower"）
     */
    explicit RobotStatePublisher(const std::string &node_name, const std::string &robot_name);

    ~RobotStatePublisher();

    /**
     * @brief 初始化ROS2节点
     * @param argc 命令行参数数量
     * @param argv 命令行参数数组
     * @return 是否初始化成功
     */
    bool init(int argc, char **argv);

    /**
     * @brief 发布关节状态
     * @param robot_state 机器人系统状态，包含手臂和手部的关节信息
     * @param arm_joint_names 手臂关节名称列表
     * @param hand_joint_names 手部关节名称列表
     */
    void publish_joint_states(std::shared_ptr<RobotSystemState> robot_state,
                              const std::vector<std::string> &arm_joint_names,
                              const std::vector<std::string> &hand_joint_names);

    /**
     * @brief 发布末端执行器位姿
     * @param position 末端位置 (x, y, z)
     * @param orientation 末端姿态旋转矩阵 (3x3)
     */
    void publish_ee_pose(const Eigen::Vector3d &position, const Eigen::Matrix3d &orientation);

    /**
     * @brief 处理ROS2回调
     * 应该在主循环中定期调用此函数
     */
    void spin_some();

    /**
     * @brief 检查ROS2节点是否正常运行
     */
    bool is_ok() const;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string robot_name_;

    // 发布器
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_pub_;

    // 是否已初始化
    bool initialized_;

    /**
     * @brief 将旋转矩阵转换为四元数
     * @param R 3x3旋转矩阵
     * @return 四元数 [w, x, y, z]
     */
    Eigen::Quaterniond rotation_matrix_to_quaternion(const Eigen::Matrix3d &R);
};

}  // namespace ros2
}  // namespace openarm
