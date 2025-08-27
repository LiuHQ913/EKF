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

## phase-1 2D平面下 差速底盘与IMU的融合

差速底盘
linear : 包含机器人在 x、y方向上的线速度。  
angular : 包含机器人在 z 方向上的角速度。  

惯性测量单元
Linear Acceleration 传感器在 x、y、z 轴上的线性加速度，单位是 m/s²（米/秒²）。
Angular Velocity 传感器绕其 x、y、z 轴的角速度，单位是 rad/s（弧度/秒）。