# 算法理论

1. 预测步骤 (Predict)

  状态预测: x̂⁻ = f(x̂⁺, u)
  协方差预测: P⁻ = F·P⁺·Fᵀ + Q

2. 更新步骤 (Update)

  创新: y = z - h(x̂⁻)
  创新协方差: S = H·P⁻·Hᵀ + R
  卡尔曼增益: K = P⁻·Hᵀ·S⁻¹
  状态更新: x̂⁺ = x̂⁻ + K·y
  协方差更新: P⁺ = (I - K·H)·P⁻


EKF更新步骤的理论基础

  1. 贝叶斯推理的核心思想

  EKF更新本质上是贝叶斯推理：
  P(状态|观测) ∝ P(观测|状态) × P(状态)

  翻译成工程语言：
  - P(状态)：我们的预测信心（来自predict步骤）
  - P(观测|状态)：传感器的可靠性
  - P(状态|观测)：融合后的最优估计

  2. 数学推导的逻辑链条

  步骤A：观测模型建立

  z = h(x) + v    其中 v ~ N(0, R)

  IMU情况：
  - z：IMU测量的yaw角度
  - h(x) = x(2)：观测函数（直接读取状态中的角度）
  - v：测量噪声
  - R：测量噪声方差

  步骤B：线性化处理

  H = ∂h/∂x|_{x=x⁻} = [0, 0, 1]

  物理含义：观测值只依赖于状态向量的第3个分量（角度）

  步骤C：创新（Innovation）计算

  y = z - h(x⁻)

  关键洞察：创新y表示"观测与预测的差异"

  步骤D：创新协方差

  S = H·P⁻·H^T + R

  物理含义：S表示"创新的不确定性"，结合了：
  - 预测不确定性（H·P⁻·H^T）
  - 观测不确定性（R）

  步骤E：卡尔曼增益

  K = P⁻·H^T·S^(-1)

  关键理解：K是一个"信任权重矩阵"

  步骤F：状态更新

  x⁺ = x⁻ + K·y

  数学解释：新状态 = 预测 + 加权修正

  步骤G：协方差更新

  P⁺ = (I - K·H)·P⁻

  物理含义：观测总是减少不确定性



  非常好的问题！这涉及到EKF中时间标记的核心概念。

  ⏰ EKF中的时间标记系统

  符号约定解释

  x⁻（x minus）：
  - 时刻：k时刻
  - 含义：先验估计（prior estimate）
  - 来源：由k-1时刻的后验估计通过运动模型预测得到
  - 对应代码：state_pre_

  x⁺（x plus）：
  - 时刻：k时刻
  - 含义：后验估计（posterior estimate）
  - 来源：融合k时刻观测后的最优估计
  - 对应代码：state_

  时间线分析

  时刻:     k-1        |    k时刻predict    |    k时刻update    |    k+1
  状态:   x⁺_{k-1}    |      x⁻_k         |      x⁺_k        |    ...
  代码:    state_      |    state_pre_      |     state_       |    ...

  在你的代码架构中

  predict()函数：
  // 输入：state_（这是x⁺_{k-1}）
  // 输出：state_pre_（这是x⁻_k）
  state_pre_(0) = pose_x + linear_velocity * cos(pose_yaw) * dt;

  updateImu()函数：
  // 输入：state_pre_（这是x⁻_k）和观测z_k
  // 输出：state_（这是x⁺_k）

  协方差的时间标记

  P⁻：对应covariance_在predict步骤结束后
  P⁺：对应covariance_在update步骤结束后