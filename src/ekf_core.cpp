#include "ekf_localization/ekf_core.h"
#include <iostream>

namespace ekf_localization {

EkfCore::EkfCore() : initialized_(false) {
    // Initialize process noise covariance Q
    process_noise_ = Eigen::Matrix3d::Identity() * 0.0025;
    process_noise_(2,2) = 0.0036; // Higher uncertainty for angle
}

void EkfCore::initialize(const Eigen::Vector3d& initial_state, 
                         const Eigen::Matrix3d& initial_covariance) {
    state_ = initial_state;
    // Normalize initial angle
    state_(2) = normalizeAngle(state_(2));
    
    covariance_ = initial_covariance;
    initialized_ = true;
    
    LOG(INFO) << "EKF initialized with state: \n"
              << "  state_: \n" << state_.transpose()
              << "  covariance_: \n" << covariance_;
}

void EkfCore::predict(double linear_velocity, double angular_velocity, double dt) 
{
    if (!initialized_) {
        LOG(ERROR) << "EKF not initialized!";
        return;
    }

    // TODO(human): 实现预测步骤
    // 1. 状态预测：根据运动模型更新状态向量
    // 2. 协方差预测：使用雅可比矩阵F更新协方差矩阵
    
    double pose_x   = state_(0);
    double pose_y   = state_(1);
    double pose_yaw = state_(2);
    
    // 状态预测 - 运动模型
    // x_{k+1} = x_k + vx * cos(θ) * dt
    // y_{k+1} = y_k + vx * sin(θ) * dt  
    // θ_{k+1} = θ_k + ω * dt
    state_pre_(0) = pose_x + linear_velocity * cos(pose_yaw) * dt;
    state_pre_(1) = pose_y + linear_velocity * sin(pose_yaw) * dt;
    state_pre_(2) = normalizeAngle(pose_yaw + angular_velocity * dt);
    
    // 协方差预测 P⁻ = F·P⁺·Fᵀ + Q
    // 需要计算雅可比矩阵F
    jacobian_F_ = computeStateJacobian(linear_velocity, pose_yaw, dt);
    covariance_ = jacobian_F_ * covariance_ * jacobian_F_.transpose() + process_noise_;

    LOG(INFO) << "Predict step \n"
              << "  state_pre_: "    << state_pre_.transpose()
              << "  jacobian_F_: \n" << jacobian_F_
              << "  covariance_: \n" << covariance_;
}

void EkfCore::updateImu(double measured_yaw, double measurement_noise) {
    if (!initialized_) {
        LOG(ERROR) << "EKF not initialized!";
        return;
    }   
    
    // 步骤1: 角度归一化
    measured_yaw = normalizeAngle(measured_yaw);
    
    // 步骤2: 设置观测雅可比矩阵 H = [0, 0, 1]
    Eigen::Matrix<double, 1, 3> H;
    H << 0.0, 0.0, 1.0;
    
    // 步骤3: 观测噪声协方差 R
    R_imu_ = measurement_noise;
    
    // TODO(human): 
    // 步骤4: 残差计算 innovation = measured_yaw - state_pre_(2)
    double diff_imu_yaw = angleDifference(measured_yaw, state_pre_(2)); // 观测与预测的残差

    // 步骤5: 残差协方差 S = H * covariance_ * H.transpose() + R
    S_imu_ = H * covariance_ * H.transpose() + R_imu_;
    
    // 步骤6: 卡尔曼增益 K = covariance_ * H.transpose() / S
    //        注意：S是标量，可以直接除法
    K_imu_ = covariance_ * H.transpose() / S_imu_;
    
    // 步骤7: 状态更新 state_ = state_pre_ + K * innovation
    //        记得对state_(2)进行角度归一化！
    /* Insight
        EKF的核心思想是状态向量中的所有分量都可能受到任何观测的影响，
        这通过协方差矩阵P的非对角元素体现。即使只观测角度，K矩阵也会
        修正x和y位置，因为它们之间存在相关性。
    */
    state_imu_ = state_pre_ + K_imu_ * diff_imu_yaw;
    state_imu_(2) = normalizeAngle(state_imu_(2));
    updateState(state_imu_);
    
    // 步骤8: 协方差更新 covariance_ = (I - K * H) * covariance_
    covariance_imu_ = (Eigen::Matrix3d::Identity() - K_imu_ * H) * covariance_;
    updateCovariance(covariance_imu_);
    
    LOG(INFO) << "Update IMU Report :"
              << "  观测值 (from IMU integration): " << measured_yaw << "\n"
              << "  观测噪声: " << measurement_noise << "\n"
              << "  残差(观测-预测): " << diff_imu_yaw << "\n"
              << "  残差协方差: " << S_imu_ << "\n"
              << "  卡尔曼增益: " << K_imu_.transpose() << "\n"
              << "  状态更新(后验, 只有yaw值): " << state_imu_ << "\n"
              << "  状态协方差(后验): \n" << covariance_imu_;
}

void EkfCore::updateOdometry(double measured_x, 
                             double measured_y, 
                             double position_noise_x, 
                             double position_noise_y) 
{
    if (!initialized_) {
        LOG(ERROR) << "EKF not initialized!";
        return;
    }

    // 步骤1: 设置观测雅可比矩阵 H = [1, 0, 0; 0, 1, 0]
    Eigen::Matrix<double, 2, 3> H;
    H << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0;
         
    // 步骤2: 设置观测噪声协方差矩阵 R (独立模型)
    Eigen::Matrix2d R_odom = Eigen::Matrix2d::Zero();
    R_odom(0,0) = position_noise_x;  // x方向噪声方差
    R_odom(1,1) = position_noise_y;  // y方向噪声方差
    
    // TODO(human): 
    // 步骤3: 残差计算 innovation = [measured_x - state_pre_(0), measured_y - state_pre_(1)]
    Eigen::Vector2d diff_odom_xy = Eigen::Vector2d(measured_x - state_pre_(0), 
                                                   measured_y - state_pre_(1));

    // 步骤4: 残差协方差 S = H * covariance_ * H.transpose() + R
    //        注意：S是2×2矩阵
    S_odom_ = H * covariance_ * H.transpose() + R_odom;

    // 步骤5: 卡尔曼增益 K = covariance_ * H.transpose() * S.inverse()
    //        注意：必须使用矩阵求逆，不能直接除法
    Eigen::Matrix2d S_inverse = S_odom_.inverse(); // 小型矩阵(< 4x4) 直接显示求逆
    K_odom_ = covariance_ * H.transpose() * S_inverse;
    
    // 步骤6: 状态更新 state_ = state_pre_ + K * innovation
    //        注意：角度不需要特殊处理，因为没有观测角度
    state_odom_ = state_pre_ + K_odom_ * diff_odom_xy;
    updateState(state_odom_);

    // 步骤7: 协方差更新 covariance_ = (I - K * H) * covariance_
    covariance_odom_ = (Eigen::Matrix3d::Identity() - K_odom_ * H) * covariance_;
    updateCovariance(covariance_odom_);

    LOG(INFO) << "Update Odometry Report :"
              << "  观测值 (from Odometry): " << measured_x << ", " << measured_y << "\n"
              << "  观测噪声: " << position_noise_x << ", " << position_noise_y << "\n"
              << "  残差(观测-预测): " << diff_odom_xy.transpose() << "\n"
              << "  残差协方差: \n" << S_odom_ << "\n"
              << "  卡尔曼增益: " << K_odom_.transpose() << "\n"
              << "  状态更新(后验, 只有xy值): " << state_odom_ << "\n"
              << "  状态协方差(后验): \n" << covariance_odom_;
}

void EkfCore::updateState(const Eigen::Vector3d& state_posterior) {
    state_ = state_posterior;
}

void EkfCore::updateCovariance(const Eigen::Matrix3d& covariance_posterior) {
    covariance_ = covariance_posterior;
}

Eigen::Matrix3d EkfCore::computeStateJacobian(double vx, double theta, double dt) {
    // TODO(human): 计算状态转移雅可比矩阵F
    // F = [1,  0,  -vx*sin(θ)*dt]
    //     [0,  1,   vx*cos(θ)*dt]  
    //     [0,  0,   1           ]
    
    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -vx * sin(theta) * dt;
    F(1, 2) =  vx * cos(theta) * dt;

    return F;
}

} // namespace ekf_localization