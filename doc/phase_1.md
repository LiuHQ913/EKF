# ekf_localization 开发指南 phase-1

## 🎯 目标
在 ROS 2（Humble / Rolling）中实现最小化 EKF 节点 `ekf_node`，融合：
- `/odom` (`nav_msgs::msg::Odometry`) —— **预测 (Predict)**
- `/imu` (`sensor_msgs::msg::Imu`) —— **更新 (Update)**

最终输出更精确的 `/odometry/filtered` (`nav_msgs::msg::Odometry`) 位姿估计 `[x, y, θ]`。

> 🎯 **总体愿景**
> 本项目将分阶段（Phase-1 / Phase-2 / Phase-3 等阶段）逐步演进，最终目标是构建一个对标 `robot_localization` （reference目录下）的通用 EKF/UKF 融合框架，具备以下能力：
> 1. **多传感器融合**：兼容轮速计、IMU、视觉里程计、激光里程计等多源数据；
> 2. **全 3D 状态估计**：支持 6-DoF `[x, y, z, roll, pitch, yaw]` 与速度、加速度的完整状态向量；
> 3. **双滤波架构**：类似 `robot_localization` 的定位 EKF + 定向 EKF 组合，提高姿态与位置解耦稳定性；
> 4. **参数化与动态重配置**：所有噪声协方差、传感器开关、参考坐标系均可通过 YAML/ROS 参数动态调整；
> 5. **高性能 & 可扩展**：基于现代 C++14 与模板策略模式，便于插拔不同运动/观测模型，并可在嵌入式平台运行；
> 6. **完善的测试与文档**：包含单元测试、集成仿真、Doxygen 文档与实践示例，降低集成成本。

---
## 1. 创建 ROS 2 包
```bash
cd ~/ekf_ws/src
ros2 pkg create ekf_localization --build-type ament_cmake \
  --dependencies rclcpp sensor_msgs nav_msgs geometry_msgs Eigen3
```
生成的目录包含 `CMakeLists.txt` 与 `package.xml`。

---
## 2. 目录结构
```
ekf_localization/
├─ include/
│   └─ ekf_core.hpp
├─ src/
│   ├─ ekf_core.cpp
│   └─ ekf_node.cpp
├─ launch/
│   └─ ekf.launch.py  # 可选
├─ CMakeLists.txt
└─ package.xml
```
仅三个核心源码文件即可跑通 MVP。

---
## 3. 关键文件实现
### 3.1 `include/ekf_core.hpp`
```cpp
#pragma once
#include <Eigen/Dense>

class EkfCore {
public:
    EkfCore();
    void predict(double vx, double wz, double dt);   // 预测
    void updateImu(double yaw);                      // 更新
    Eigen::Vector3d getState() const;                // [x, y, theta]
    Eigen::Matrix3d getCovariance() const;           // 协方差
private:
    Eigen::Vector3d x_;   // 状态向量
    Eigen::Matrix3d P_;   // 协方差
    Eigen::Matrix3d Q_;   // 过程噪声
    Eigen::Matrix<double,1,1> R_imu_; // IMU 噪声
};
```

### 3.2 `src/ekf_core.cpp`
1. 初始化 `x_` 为零，`P_` 为单位矩阵。
2. **预测**：
   ```cpp
   x_(0) += vx * std::cos(x_(2)) * dt;
   x_(1) += vx * std::sin(x_(2)) * dt;
   x_(2) += wz * dt;
   // 更新协方差 P_ = F P F^T + Q
   ```
3. **更新**：
   ```cpp
   double y = yaw - x_(2);           // 创新
   double S = P_(2,2) + R_imu_(0,0); // 创新协方差
   double K = P_(2,2) / S;           // 卡尔曼增益（标量）
   x_(2) += K * y;
   P_.row(2) *= (1 - K);
   P_.col(2) = P_.row(2).transpose();
   ```

### 3.3 `src/ekf_node.cpp`
- 创建 `rclcpp::Node`，订阅 `/odom` 与 `/imu`。
- 在 `odomCallback()` 中调用 `predict()`，并发布融合结果。
- 在 `imuCallback()` 中调用 `updateImu()`。
- 发布器：`/odometry/filtered`。

### 3.4 `CMakeLists.txt`
```cmake
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(ekf_core src/ekf_core.cpp)
target_include_directories(ekf_core PUBLIC include ${EIGEN3_INCLUDE_DIRS})

add_executable(ekf_node src/ekf_node.cpp)
ament_target_dependencies(ekf_node rclcpp sensor_msgs nav_msgs geometry_msgs)
target_link_libraries(ekf_node ekf_core)

install(TARGETS ekf_core ekf_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
```

---
## 4. 构建与运行
```bash
cd ~/ekf_ws
colcon build --packages-select ekf_localization
source install/setup.bash
ros2 run ekf_localization ekf_node
```
查看输出：
```bash
ros2 topic echo /odometry/filtered
```

---
## 5. 后续扩展
1. 将 `Q`、`R` 与初始 `P0` 参数化。
2. 加入 TF 发布、扩展到 3D。
3. 对标 `robot_localization` 的完整参数与双 EKF 架构。

---
## 6. 参考
- robot_localization Wiki: <https://github.com/cra-ros-pkg/robot_localization/wiki>
- Probabilistic Robotics, Ch.3 (Extended Kalman Filter)
- ROS REP-105 坐标系规范
