# OpenArm IK求解器使用指南

## 📚 概述

OpenArm项目现已集成**逆运动学（IK）求解器**，支持从末端位姿计算关节角度。基于KDL库的Levenberg-Marquardt优化算法实现。

---

## ✨ 可用的IK求解方案

### 1. **KDL IK求解器（已集成）** ✅

**优点**：
- 已经集成在项目中，无需额外安装
- 基于成熟的KDL库
- 提供两种算法：
  - `KDL_LMA`: Levenberg-Marquardt优化（推荐，速度快）
  - `KDL_NUMERICAL`: Newton-Raphson数值迭代

**适用场景**：
- 实时VR控制
- 末端位姿跟踪
- 轨迹规划

### 2. **MoveIt（已安装）** ✅

**优点**：
- 功能最全面
- 支持碰撞检测、轨迹规划
- 集成RViz可视化
- 支持多种IK求解器（KDL、TRAC-IK等）

**适用场景**：
- 复杂运动规划
- 需要碰撞检测
- RViz可视化调试

**使用方法**：参考[MoveIt官方文档](https://moveit.ros.org/)

### 3. **TRAC-IK（未安装，可选）**

**安装方法**：
```bash
sudo apt install ros-humble-trac-ik-kinematics-plugin
```

**优点**：
- 比KDL更快
- 更高的求解成功率
- 可直接替换KDL作为MoveIt的求解器

---

## 🚀 快速开始

### 方案A：在C++代码中直接使用

#### 1. 包含头文件
```cpp
#include <controller/dynamics.hpp>
```

#### 2. 初始化Dynamics和IK求解器
```cpp
// 创建Dynamics对象
Dynamics dynamics(urdf_path, root_link, leaf_link);
dynamics.Init();

// 初始化IK求解器（使用LMA算法）
dynamics.InitIKSolver(true);  // true表示使用LMA，false使用数值迭代
```

#### 3. 调用IK求解

**方法1：使用旋转矩阵**
```cpp
Eigen::Vector3d target_position(0.3, 0.2, 0.5);  // 目标位置 [x, y, z]
Eigen::Matrix3d target_orientation;              // 目标姿态（旋转矩阵）
std::vector<double> current_joints = {0, 0, 0, 0, 0, 0};  // 当前关节角度（作为初值）

std::vector<double> solution;
bool success = dynamics.SolveIK(target_position, target_orientation, 
                                current_joints, solution);
```

**方法2：使用四元数（推荐）**
```cpp
Eigen::Vector3d target_position(0.3, 0.2, 0.5);
Eigen::Quaterniond target_quat(0.707, 0.0, 0.707, 0.0);  // [w, x, y, z]
std::vector<double> current_joints = {0, 0, 0, 0, 0, 0};

std::vector<double> solution;
bool success = dynamics.SolveIKQuaternion(target_position, target_quat, 
                                          current_joints, solution);
```

**方法3：仅位置IK（忽略姿态）**
```cpp
Eigen::Vector3d target_position(0.3, 0.2, 0.5);
std::vector<double> current_joints = {0, 0, 0, 0, 0, 0};

std::vector<double> solution;
bool success = dynamics.SolveIKPositionOnly(target_position, current_joints, solution);
```

#### 4. 使用求解结果
```cpp
if (success) {
    std::cout << "IK solved! Joint angles: ";
    for (auto angle : solution) {
        std::cout << angle << " ";
    }
    std::cout << std::endl;
    
    // 设置为机器人参考值
    // robot_state->arm_state().set_all_references(solution);
} else {
    std::cerr << "IK solving failed!" << std::endl;
}
```

---

## 🧪 测试IK求解器

### 运行测试程序
```bash
cd /home/robot/openarm_teleop/build

# 测试IK求解器功能
./ik_test
```

**测试内容**：
1. 零位姿态的FK和IK
2. 随机关节配置的IK
3. 仅位置IK测试
4. 四元数接口测试
5. 验证 FK(IK(pose)) ≈ pose

---

## 🎮 在VR控制中使用IK

VR控制示例程序（`vr_control_example`）已经集成了IK求解器。

### 工作流程

1. **VR端发布末端位姿命令**（geometry_msgs/PoseStamped）到`/robot/ee_pose_command`
2. **机器人端接收位姿命令**，回调函数触发
3. **IK求解器计算关节角度**
4. **设置关节参考值**到控制器
5. **控制器执行运动**

### 关键代码

在[vr_control_example.cpp](vr_control_example.cpp#L234-L284)中：

```cpp
// 设置末端位姿命令回调
vr_interface->set_ee_pose_command_callback(
    [robot_state, arm_dynamics](const Eigen::Vector3d& position, 
                                const Eigen::Quaterniond& orientation) {
        // 获取当前关节角度作为IK初值
        std::vector<double> current_joints(arm_dof);
        std::vector<JointState> current_arm_states = robot_state->arm_state().get_all_responses();
        for (size_t i = 0; i < arm_dof; ++i) {
            current_joints[i] = current_arm_states[i].position;
        }

        // 求解IK
        std::vector<double> ik_solution;
        bool ik_success = arm_dynamics->SolveIKQuaternion(
            position, orientation, current_joints, ik_solution);
        
        if (ik_success) {
            // 设置关节参考值
            std::vector<JointState> arm_refs(arm_dof);
            for (size_t i = 0; i < arm_dof; ++i) {
                arm_refs[i].position = ik_solution[i];
                arm_refs[i].velocity = 0.0;
                arm_refs[i].effort = 0.0;
            }
            robot_state->arm_state().set_all_references(arm_refs);
        }
    });
```

---

## 📊 API参考

### Dynamics类的IK接口

```cpp
class Dynamics {
public:
    /**
     * @brief 初始化IK求解器
     * @param use_lma true=使用LMA优化算法（推荐），false=使用数值迭代法
     * @return 是否成功初始化
     */
    bool InitIKSolver(bool use_lma = true);
    
    /**
     * @brief 求解逆运动学（旋转矩阵版本）
     * @param target_position 目标位置 [x, y, z]
     * @param target_orientation 目标姿态（3x3旋转矩阵）
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
```

---

## ⚠️ 注意事项

### 1. 初始化顺序
**必须**按以下顺序初始化：
```cpp
dynamics.Init();          // 1. 先初始化Dynamics
dynamics.InitIKSolver();  // 2. 再初始化IK求解器
```

### 2. 初值的重要性
IK求解器是**迭代算法**，需要一个初始猜测：
- ✅ **推荐**：使用当前关节角度作为初值（最接近解）
- ⚠️ **可用**：使用零位或其他已知配置
- ❌ **避免**：使用随机值（可能导致求解失败）

### 3. 求解失败的原因
- 目标位姿超出工作空间
- 目标位姿处于奇异点附近
- 初值离解太远
- 姿态约束过严（考虑使用`SolveIKPositionOnly`）

### 4. 性能考虑
- LMA算法通常在**10-50次迭代**内收敛
- 求解时间约**1-5ms**（取决于配置）
- 适合**实时控制**（500Hz控制循环可用）

---

## 📦 相关文件

| 文件 | 描述 |
|------|------|
| [src/controller/ik_solver.hpp](../src/controller/ik_solver.hpp) | IK求解器类定义 |
| [src/controller/ik_solver.cpp](../src/controller/ik_solver.cpp) | IK求解器实现 |
| [src/controller/dynamics.hpp](../src/controller/dynamics.hpp) | Dynamics类（含IK接口） |
| [src/controller/dynamics.cpp](../src/controller/dynamics.cpp) | Dynamics实现 |
| [control/ik_test.cpp](../control/ik_test.cpp) | IK测试程序 |
| [control/vr_control_example.cpp](../control/vr_control_example.cpp) | VR控制示例（集成IK） |

---

## 🔧 高级用法

### 使用MoveIt进行IK求解

如果需要更强大的功能（碰撞检测、轨迹规划等），可以使用MoveIt：

```bash
# 安装MoveIt（已安装）
sudo apt install ros-humble-moveit

# 配置机器人描述包
# 参考: https://moveit.ros.org/install-moveit2/source/
```

**MoveIt的优势**：
- 多种IK插件（KDL、TRAC-IK、IKFast等）
- 碰撞检测
- 运动规划
- RViz可视化

### 切换IK算法

修改`Dynamics::InitIKSolver()`参数：
```cpp
// 使用LMA优化算法（推荐）
dynamics.InitIKSolver(true);

// 使用Newton-Raphson数值迭代
dynamics.InitIKSolver(false);
```

---

## 🐛 常见问题

### Q1: IK求解失败怎么办？
**A**: 
1. 检查目标位姿是否在工作空间内
2. 尝试不同的初值
3. 使用`SolveIKPositionOnly`仅约束位置
4. 增加求解器迭代次数（修改`ik_solver.cpp`中的参数）

### Q2: 如何提高求解成功率？
**A**:
1. 使用当前关节角度作为初值
2. 考虑安装TRAC-IK（成功率更高）
3. 放宽姿态约束（position-only IK）

### Q3: 如何可视化IK结果？
**A**:
- 使用RViz订阅`/robot/joint_states`和`/robot/ee_pose`
- 或使用MoveIt的可视化工具

---

## 📚 参考资源

- [KDL官方文档](https://www.orocos.org/kdl.html)
- [MoveIt文档](https://moveit.ros.org/)
- [TRAC-IK论文](https://humanoids.ieee-ras.org/wp-content/uploads/2017/05/beeson-humanoids-15-fast-ik.pdf)
- [ROS2 Humble文档](https://docs.ros.org/en/humble/)

---

## ✅ 总结

| 功能 | 状态 |
|------|------|
| KDL IK求解器 | ✅ 已集成 |
| 旋转矩阵接口 | ✅ 可用 |
| 四元数接口 | ✅ 可用 |
| 仅位置IK | ✅ 可用 |
| VR控制集成 | ✅ 已实现 |
| 测试程序 | ✅ 可运行 |
| MoveIt支持 | ✅ 已安装 |

---

**需要帮助？** 参考以上文档或运行测试程序 `./ik_test` 查看示例。
