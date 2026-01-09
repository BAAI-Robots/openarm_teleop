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

#include "ros2_publisher.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace openarm {
namespace ros2 {

RobotStatePublisher::RobotStatePublisher(const std::string &node_name,
                                         const std::string &robot_name)
    : robot_name_(robot_name), initialized_(false) {
    // 构造函数中不初始化ROS2，等待init()调用
}

RobotStatePublisher::~RobotStatePublisher() {
    if (initialized_ && rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

bool RobotStatePublisher::init(int argc, char **argv) {
    if (initialized_) {
        return true;
    }

    try {
        // 初始化ROS2
        rclcpp::init(argc, argv);

        // 创建节点
        node_ = std::make_shared<rclcpp::Node>(robot_name_ + "_state_publisher");

        // 创建发布器
        joint_state_pub_ = node_->create_publisher<sensor_msgs::msg::JointState>(
            "/" + robot_name_ + "/joint_states", 10);

        ee_pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/" + robot_name_ + "/ee_pose", 10);

        initialized_ = true;
        RCLCPP_INFO(node_->get_logger(), "ROS2 Publisher initialized for robot: %s",
                    robot_name_.c_str());
        return true;
    } catch (const std::exception &e) {
        std::cerr << "Failed to initialize ROS2 publisher: " << e.what() << std::endl;
        return false;
    }
}

void RobotStatePublisher::publish_joint_states(
    std::shared_ptr<RobotSystemState> robot_state,
    const std::vector<std::string> &arm_joint_names,
    const std::vector<std::string> &hand_joint_names) {
    if (!initialized_ || !is_ok()) {
        return;
    }

    // 获取关节响应状态
    auto arm_states = robot_state->arm_state().get_all_responses();
    auto hand_states = robot_state->hand_state().get_all_responses();

    // 创建JointState消息
    auto joint_state_msg = sensor_msgs::msg::JointState();
    joint_state_msg.header.stamp = node_->now();
    joint_state_msg.header.frame_id = robot_name_ + "_base_link";

    // 添加手臂关节
    for (size_t i = 0; i < arm_states.size() && i < arm_joint_names.size(); ++i) {
        joint_state_msg.name.push_back(arm_joint_names[i]);
        joint_state_msg.position.push_back(arm_states[i].position);
        joint_state_msg.velocity.push_back(arm_states[i].velocity);
        joint_state_msg.effort.push_back(arm_states[i].effort);
    }

    // 添加手部关节
    for (size_t i = 0; i < hand_states.size() && i < hand_joint_names.size(); ++i) {
        joint_state_msg.name.push_back(hand_joint_names[i]);
        joint_state_msg.position.push_back(hand_states[i].position);
        joint_state_msg.velocity.push_back(hand_states[i].velocity);
        joint_state_msg.effort.push_back(hand_states[i].effort);
    }

    // 发布消息
    joint_state_pub_->publish(joint_state_msg);
}

void RobotStatePublisher::publish_ee_pose(const Eigen::Vector3d &position,
                                          const Eigen::Matrix3d &orientation) {
    if (!initialized_ || !is_ok()) {
        return;
    }

    // 创建PoseStamped消息
    auto pose_msg = geometry_msgs::msg::PoseStamped();
    pose_msg.header.stamp = node_->now();
    pose_msg.header.frame_id = robot_name_ + "_base_link";

    // 设置位置
    pose_msg.pose.position.x = position.x();
    pose_msg.pose.position.y = position.y();
    pose_msg.pose.position.z = position.z();

    // 将旋转矩阵转换为四元数
    Eigen::Quaterniond quat = rotation_matrix_to_quaternion(orientation);
    pose_msg.pose.orientation.w = quat.w();
    pose_msg.pose.orientation.x = quat.x();
    pose_msg.pose.orientation.y = quat.y();
    pose_msg.pose.orientation.z = quat.z();

    // 发布消息
    ee_pose_pub_->publish(pose_msg);
}

void RobotStatePublisher::spin_some() {
    if (initialized_ && is_ok()) {
        rclcpp::spin_some(node_);
    }
}

bool RobotStatePublisher::is_ok() const { return initialized_ && rclcpp::ok(); }

Eigen::Quaterniond RobotStatePublisher::rotation_matrix_to_quaternion(
    const Eigen::Matrix3d &R) {
    Eigen::Quaterniond quat(R);
    quat.normalize();
    return quat;
}

}  // namespace ros2
}  // namespace openarm
