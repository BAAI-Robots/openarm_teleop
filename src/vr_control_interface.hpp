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
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <robot_state.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>

namespace openarm {
namespace ros2 {

/**
 * @brief VR控制接口 - 订阅关节命令和末端位姿命令
 * 
 * 该类提供ROS2订阅接口，用于接收来自VR控制器的命令：
 * - /{robot_name}/joint_command: 订阅关节位置命令
 * - /{robot_name}/ee_pose_command: 订阅末端执行器位姿命令
 * - /{robot_name}/gripper_command: 订阅夹爪命令
 * 
 * 使用回调函数将命令传递给控制系统
 */
class VRControlInterface {
public:
    // 回调函数类型定义
    using JointCommandCallback = std::function<void(const std::vector<double>&)>;
    using EEPoseCommandCallback = std::function<void(const Eigen::Vector3d&, const Eigen::Quaterniond&)>;
    using GripperCommandCallback = std::function<void(double)>;

    /**
     * @brief 构造函数
     * @param node_name ROS2节点名称
     * @param robot_name 机器人名称（如"robot", "follower"）
     */
    explicit VRControlInterface(const std::string& node_name, const std::string& robot_name);

    ~VRControlInterface();

    /**
     * @brief 初始化ROS2节点和订阅器
     * @param argc 命令行参数数量
     * @param argv 命令行参数数组
     * @return 是否初始化成功
     */
    bool init(int argc, char** argv);

    /**
     * @brief 设置关节命令回调函数
     * @param callback 当收到关节命令时调用的回调函数
     */
    void set_joint_command_callback(JointCommandCallback callback);

    /**
     * @brief 设置末端位姿命令回调函数
     * @param callback 当收到末端位姿命令时调用的回调函数
     */
    void set_ee_pose_command_callback(EEPoseCommandCallback callback);

    /**
     * @brief 设置夹爪命令回调函数
     * @param callback 当收到夹爪命令时调用的回调函数
     */
    void set_gripper_command_callback(GripperCommandCallback callback);

    /**
     * @brief 处理ROS2回调
     * 应该在主循环中定期调用此函数
     */
    void spin_some();

    /**
     * @brief 检查ROS2节点是否正常运行
     */
    bool is_ok() const;

    /**
     * @brief 获取节点指针（用于高级用法）
     */
    std::shared_ptr<rclcpp::Node> get_node() { return node_; }

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string robot_name_;

    // 订阅器
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_command_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ee_pose_command_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr gripper_command_sub_;

    // 回调函数
    JointCommandCallback joint_command_callback_;
    EEPoseCommandCallback ee_pose_command_callback_;
    GripperCommandCallback gripper_command_callback_;

    // 是否已初始化
    bool initialized_;

    /**
     * @brief 关节命令消息回调
     */
    void joint_command_msg_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    /**
     * @brief 末端位姿命令消息回调
     */
    void ee_pose_command_msg_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    /**
     * @brief 夹爪命令消息回调
     */
    void gripper_command_msg_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
};

}  // namespace ros2
}  // namespace openarm
