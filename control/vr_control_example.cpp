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
 * @file vr_control_example.cpp
 * @brief VR控制示例程序 - 展示如何使用VR控制接口
 * 
 * 该程序展示如何：
 * 1. 订阅VR控制器发送的关节命令
 * 2. 订阅VR控制器发送的末端位姿命令
 * 3. 订阅VR控制器发送的夹爪命令
 * 4. 将命令应用到机器人并发布状态
 */

#include <atomic>
#include <chrono>
#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm_port/openarm_init.hpp>
#include <periodic_timer_thread.hpp>
#include <robot_state.hpp>
#include <ros2_publisher.hpp>
#include <thread>
#include <vr_control_interface.hpp>
#include <yamlloader.hpp>

std::atomic<bool> keep_running(true);

void signal_handler(int signal) {
    if (signal == SIGINT) {
        std::cout << "\nCtrl+C detected. Exiting..." << std::endl;
        keep_running = false;
    }
}

/**
 * @brief VR控制线程 - 处理机器人控制和状态发布
 */
class VRControlThread : public PeriodicTimerThread {
public:
    VRControlThread(std::shared_ptr<RobotSystemState> robot_state, Control* control,
                    Dynamics* dynamics,
                    std::shared_ptr<openarm::ros2::RobotStatePublisher> ros2_pub,
                    double hz = 500.0)
        : PeriodicTimerThread(hz),
          robot_state_(robot_state),
          control_(control),
          dynamics_(dynamics),
          ros2_pub_(ros2_pub),
          publish_counter_(0) {}

protected:
    void before_start() override { std::cout << "[VR Control] Thread started" << std::endl; }

    void after_stop() override { std::cout << "[VR Control] Thread stopped" << std::endl; }

    void on_timer() override {
        // 执行控制步骤
        control_->unilateral_step();

        // 每10次发布一次ROS2消息（50Hz）
        publish_counter_++;
        if (publish_counter_ >= 10 && ros2_pub_) {
            publish_counter_ = 0;

            // 发布关节状态
            std::vector<std::string> arm_joint_names = {"joint1", "joint2", "joint3",
                                                        "joint4", "joint5", "joint6"};
            std::vector<std::string> hand_joint_names = {"gripper_joint"};
            ros2_pub_->publish_joint_states(robot_state_, arm_joint_names, hand_joint_names);

            // 获取并发布末端执行器位姿
            auto arm_responses = robot_state_->arm_state().get_all_responses();
            std::vector<double> joint_positions(arm_responses.size());
            for (size_t i = 0; i < arm_responses.size(); ++i) {
                joint_positions[i] = arm_responses[i].position;
            }

            Eigen::Matrix3d ee_rotation;
            Eigen::Vector3d ee_position;
            dynamics_->GetEECordinate(joint_positions.data(), ee_rotation, ee_position);
            ros2_pub_->publish_ee_pose(ee_position, ee_rotation);

            // 处理ROS2回调
            ros2_pub_->spin_some();
        }
    }

private:
    std::shared_ptr<RobotSystemState> robot_state_;
    Control* control_;
    Dynamics* dynamics_;
    std::shared_ptr<openarm::ros2::RobotStatePublisher> ros2_pub_;
    int publish_counter_;
};

int main(int argc, char** argv) {
    try {
        std::signal(SIGINT, signal_handler);

        std::cout << "=== OpenArm VR Control Example ===" << std::endl;

        // 配置参数
        std::string can_interface = (argc > 1) ? argv[1] : "can0";
        std::string urdf_path = (argc > 2) ? argv[2] : "/tmp/openarm_urdf_gen/v10_leader.urdf";
        std::string arm_side = (argc > 3) ? argv[3] : "right_arm";

        std::cout << "CAN Interface: " << can_interface << std::endl;
        std::cout << "URDF Path: " << urdf_path << std::endl;
        std::cout << "Arm Side: " << arm_side << std::endl;

        // 检查URDF文件
        if (!std::filesystem::exists(urdf_path)) {
            std::cerr << "[ERROR] URDF not found: " << urdf_path << std::endl;
            std::cerr << "Please generate URDF first or specify correct path" << std::endl;
            return 1;
        }

        // 设置动力学
        std::string root_link = "openarm_body_link0";
        std::string leaf_link =
            (arm_side == "left_arm") ? "openarm_left_hand" : "openarm_right_hand";

        Dynamics* arm_dynamics = new Dynamics(urdf_path, root_link, leaf_link);
        arm_dynamics->Init();
        
        // 初始化IK求解器（使用LMA优化算法）
        std::cout << "[INFO] Initializing IK solver..." << std::endl;
        if (!arm_dynamics->InitIKSolver(true)) {
            std::cerr << "[ERROR] Failed to initialize IK solver!" << std::endl;
            return 1;
        }

        // 初始化OpenArm硬件
        std::cout << "[INFO] Initializing OpenArm hardware..." << std::endl;
        openarm::can::socket::OpenArm* openarm =
            openarm_init::OpenArmInitializer::initialize_openarm(can_interface, false);

        size_t arm_motor_num = openarm->get_arm().get_motors().size();
        size_t hand_motor_num = openarm->get_gripper().get_motors().size();

        std::cout << "Arm motors: " << arm_motor_num << std::endl;
        std::cout << "Hand motors: " << hand_motor_num << std::endl;

        // 加载参数
        YamlLoader loader("config/leader.yaml");
        std::vector<double> kp = loader.get_vector("LeaderArmParam", "Kp");
        std::vector<double> kd = loader.get_vector("LeaderArmParam", "Kd");
        std::vector<double> Fc = loader.get_vector("LeaderArmParam", "Fc");
        std::vector<double> k = loader.get_vector("LeaderArmParam", "k");
        std::vector<double> Fv = loader.get_vector("LeaderArmParam", "Fv");
        std::vector<double> Fo = loader.get_vector("LeaderArmParam", "Fo");

        // 创建机器人状态
        std::shared_ptr<RobotSystemState> robot_state =
            std::make_shared<RobotSystemState>(arm_motor_num, hand_motor_num);

        // 创建控制器
        const double CONTROL_FREQUENCY = 500.0;
        Control* control =
            new Control(openarm, arm_dynamics, arm_dynamics, robot_state, 1.0 / CONTROL_FREQUENCY,
                        ROLE_LEADER, arm_side, arm_motor_num, hand_motor_num);
        control->SetParameter(kp, kd, Fc, k, Fv, Fo);

        // 初始化ROS2发布器
        std::cout << "[INFO] Initializing ROS2 Publisher..." << std::endl;
        std::shared_ptr<openarm::ros2::RobotStatePublisher> ros2_pub =
            std::make_shared<openarm::ros2::RobotStatePublisher>("robot_state_pub", "robot");
        if (!ros2_pub->init(argc, argv)) {
            std::cerr << "[ERROR] Failed to initialize ROS2 publisher!" << std::endl;
            return 1;
        }

        // 初始化VR控制接口
        std::cout << "[INFO] Initializing VR Control Interface..." << std::endl;
        std::shared_ptr<openarm::ros2::VRControlInterface> vr_interface =
            std::make_shared<openarm::ros2::VRControlInterface>("vr_control", "robot");
        if (!vr_interface->init(argc, argv)) {
            std::cerr << "[ERROR] Failed to initialize VR control interface!" << std::endl;
            return 1;
        }

        // 设置关节命令回调
        vr_interface->set_joint_command_callback(
            [robot_state](const std::vector<double>& joint_positions) {
                std::cout << "[VR] Received joint command: " << joint_positions.size()
                          << " joints" << std::endl;

                // 将关节命令设置为参考值
                size_t arm_dof = robot_state->arm_state().get_size();
                size_t hand_dof = robot_state->hand_state().get_size();

                if (joint_positions.size() == arm_dof + hand_dof) {
                    // 设置手臂关节参考
                    std::vector<JointState> arm_refs(arm_dof);
                    for (size_t i = 0; i < arm_dof; ++i) {
                        arm_refs[i].position = joint_positions[i];
                        arm_refs[i].velocity = 0.0;
                        arm_refs[i].effort = 0.0;
                    }
                    robot_state->arm_state().set_all_references(arm_refs);

                    // 设置手部关节参考
                    std::vector<JointState> hand_refs(hand_dof);
                    for (size_t i = 0; i < hand_dof; ++i) {
                        hand_refs[i].position = joint_positions[arm_dof + i];
                        hand_refs[i].velocity = 0.0;
                        hand_refs[i].effort = 0.0;
                    }
                    robot_state->hand_state().set_all_references(hand_refs);
                } else {
                    std::cerr << "[WARNING] Joint command size mismatch! Expected "
                              << (arm_dof + hand_dof) << ", got " << joint_positions.size()
                              << std::endl;
                }
            });

        // 设置末端位姿命令回调
        vr_interface->set_ee_pose_command_callback(
            [robot_state, arm_dynamics](const Eigen::Vector3d& position, 
                                        const Eigen::Quaterniond& orientation) {
                std::cout << "[VR] Received EE pose command: "
                          << "pos[" << position.x() << ", " << position.y() << ", "
                          << position.z() << "], "
                          << "quat[" << orientation.w() << ", " << orientation.x() << ", "
                          << orientation.y() << ", " << orientation.z() << "]" << std::endl;

                // 获取当前关节角度作为IK初值
                size_t arm_dof = robot_state->arm_state().get_size();
                std::vector<double> current_joints(arm_dof);
                
                // 获取当前关节响应状态
                std::vector<JointState> current_arm_states = robot_state->arm_state().get_all_responses();
                for (size_t i = 0; i < arm_dof; ++i) {
                    current_joints[i] = current_arm_states[i].position;
                }

                // 求解IK
                std::vector<double> ik_solution;
                bool ik_success = arm_dynamics->SolveIKQuaternion(position, orientation, 
                                                                   current_joints, ik_solution);
                
                if (ik_success) {
                    std::cout << "[VR] IK solved successfully! Setting joint references..." 
                              << std::endl;
                    
                    // 设置关节参考值
                    std::vector<JointState> arm_refs(arm_dof);
                    for (size_t i = 0; i < arm_dof; ++i) {
                        arm_refs[i].position = ik_solution[i];
                        arm_refs[i].velocity = 0.0;
                        arm_refs[i].effort = 0.0;
                    }
                    robot_state->arm_state().set_all_references(arm_refs);
                    
                    // 打印解算结果（前3个关节）
                    std::cout << "[VR] IK solution: [";
                    for (size_t i = 0; i < std::min(size_t(3), arm_dof); ++i) {
                        std::cout << ik_solution[i];
                        if (i < std::min(size_t(3), arm_dof) - 1) std::cout << ", ";
                    }
                    if (arm_dof > 3) std::cout << ", ...";
                    std::cout << "]" << std::endl;
                } else {
                    std::cerr << "[ERROR] IK solving failed for target pose!" << std::endl;
                }
            });

        // 设置夹爪命令回调
        vr_interface->set_gripper_command_callback([robot_state](double gripper_value) {
            std::cout << "[VR] Received gripper command: " << gripper_value << std::endl;

            // 将夹爪命令设置为手部参考值
            size_t hand_dof = robot_state->hand_state().get_size();
            std::vector<JointState> hand_refs(hand_dof);
            for (size_t i = 0; i < hand_dof; ++i) {
                hand_refs[i].position = gripper_value;
                hand_refs[i].velocity = 0.0;
                hand_refs[i].effort = 0.0;
            }
            robot_state->hand_state().set_all_references(hand_refs);
        });

        // 回到初始位置
        std::cout << "[INFO] Moving to home position..." << std::endl;
        control->AdjustPosition();

        // 启动控制线程
        std::cout << "[INFO] Starting control thread..." << std::endl;
        VRControlThread control_thread(robot_state, control, arm_dynamics, ros2_pub, CONTROL_FREQUENCY);
        control_thread.start_thread();

        std::cout << "\n=== VR Control Active ===" << std::endl;
        std::cout << "Waiting for VR commands on topics:" << std::endl;
        std::cout << "  - /robot/joint_command" << std::endl;
        std::cout << "  - /robot/ee_pose_command" << std::endl;
        std::cout << "  - /robot/gripper_command" << std::endl;
        std::cout << "\nPublishing robot state on:" << std::endl;
        std::cout << "  - /robot/joint_states" << std::endl;
        std::cout << "  - /robot/ee_pose" << std::endl;
        std::cout << "\nPress Ctrl+C to exit..." << std::endl;

        // 主循环 - 处理VR接口回调
        while (keep_running) {
            vr_interface->spin_some();
            ros2_pub->spin_some();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // 停止控制线程
        std::cout << "\n[INFO] Stopping control thread..." << std::endl;
        control_thread.stop_thread();

        // 关闭硬件
        std::cout << "[INFO] Disabling motors..." << std::endl;
        openarm->disable_all();

        std::cout << "[INFO] Shutdown complete" << std::endl;

        delete control;
        delete arm_dynamics;

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
