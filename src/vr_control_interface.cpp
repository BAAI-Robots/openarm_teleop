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

#include "vr_control_interface.hpp"

namespace openarm {
namespace ros2 {

VRControlInterface::VRControlInterface(const std::string& node_name,
                                       const std::string& robot_name)
    : robot_name_(robot_name), initialized_(false) {}

VRControlInterface::~VRControlInterface() {
    if (initialized_ && rclcpp::ok()) {
        // 节点会在rclcpp::shutdown时自动清理
    }
}

bool VRControlInterface::init(int argc, char** argv) {
    if (initialized_) {
        return true;
    }

    try {
        // 如果rclcpp还未初始化，则初始化
        if (!rclcpp::ok()) {
            rclcpp::init(argc, argv);
        }

        // 创建节点
        node_ = std::make_shared<rclcpp::Node>(robot_name_ + "_vr_control_interface");

        // 创建订阅器 - 关节命令
        joint_command_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/" + robot_name_ + "/joint_command", 10,
            std::bind(&VRControlInterface::joint_command_msg_callback, this,
                      std::placeholders::_1));

        // 创建订阅器 - 末端位姿命令
        ee_pose_command_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/" + robot_name_ + "/ee_pose_command", 10,
            std::bind(&VRControlInterface::ee_pose_command_msg_callback, this,
                      std::placeholders::_1));

        // 创建订阅器 - 夹爪命令
        gripper_command_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/" + robot_name_ + "/gripper_command", 10,
            std::bind(&VRControlInterface::gripper_command_msg_callback, this,
                      std::placeholders::_1));

        initialized_ = true;
        RCLCPP_INFO(node_->get_logger(), "VR Control Interface initialized for robot: %s",
                    robot_name_.c_str());
        RCLCPP_INFO(node_->get_logger(), "Subscribing to topics:");
        RCLCPP_INFO(node_->get_logger(), "  - /%s/joint_command", robot_name_.c_str());
        RCLCPP_INFO(node_->get_logger(), "  - /%s/ee_pose_command", robot_name_.c_str());
        RCLCPP_INFO(node_->get_logger(), "  - /%s/gripper_command", robot_name_.c_str());

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize VR Control Interface: " << e.what() << std::endl;
        return false;
    }
}

void VRControlInterface::set_joint_command_callback(JointCommandCallback callback) {
    joint_command_callback_ = callback;
}

void VRControlInterface::set_ee_pose_command_callback(EEPoseCommandCallback callback) {
    ee_pose_command_callback_ = callback;
}

void VRControlInterface::set_gripper_command_callback(GripperCommandCallback callback) {
    gripper_command_callback_ = callback;
}

void VRControlInterface::spin_some() {
    if (initialized_ && is_ok()) {
        rclcpp::spin_some(node_);
    }
}

bool VRControlInterface::is_ok() const { return initialized_ && rclcpp::ok(); }

void VRControlInterface::joint_command_msg_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!joint_command_callback_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Joint command received but no callback is set!");
        return;
    }

    // 提取位置命令
    std::vector<double> joint_positions;
    if (!msg->position.empty()) {
        joint_positions.assign(msg->position.begin(), msg->position.end());
        
        RCLCPP_DEBUG(node_->get_logger(), 
                     "Received joint command: %zu joints", joint_positions.size());
        
        // 调用回调函数
        joint_command_callback_(joint_positions);
    } else {
        RCLCPP_WARN(node_->get_logger(), "Received empty joint command!");
    }
}

void VRControlInterface::ee_pose_command_msg_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!ee_pose_command_callback_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "EE pose command received but no callback is set!");
        return;
    }

    // 提取位置
    Eigen::Vector3d position(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    // 提取姿态（四元数）
    Eigen::Quaterniond orientation(msg->pose.orientation.w, msg->pose.orientation.x,
                                   msg->pose.orientation.y, msg->pose.orientation.z);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Received EE pose command: pos[%.3f, %.3f, %.3f], quat[%.3f, %.3f, %.3f, %.3f]",
                 position.x(), position.y(), position.z(), orientation.w(), orientation.x(),
                 orientation.y(), orientation.z());

    // 调用回调函数
    ee_pose_command_callback_(position, orientation);
}

void VRControlInterface::gripper_command_msg_callback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (!gripper_command_callback_) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                             "Gripper command received but no callback is set!");
        return;
    }

    if (msg->data.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Received empty gripper command!");
        return;
    }

    // 提取夹爪命令（通常是一个值，表示开合程度）
    double gripper_value = msg->data[0];

    RCLCPP_DEBUG(node_->get_logger(), "Received gripper command: %.3f", gripper_value);

    // 调用回调函数
    gripper_command_callback_(gripper_value);
}

}  // namespace ros2
}  // namespace openarm
