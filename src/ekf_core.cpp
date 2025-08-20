#include "ekf_localization/ekf_core.hpp"
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
    
    LOG(INFO) << "EKF initialized with state: [" 
              << state_(0) << ", " << state_(1) << ", " << state_(2) << "]";
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

    // TODO(human): 实现IMU更新步骤
    // 观测模型: z = θ (直接观测角度)

    // 观测雅可比矩阵 H = [0, 0, 1]
    
    measured_yaw = normalizeAngle(measured_yaw);
    
    // 创新计算
    // 卡尔曼增益计算
    // 状态更新
    // 协方差更新
    
    std::cout << "IMU update: measured_yaw=" << measured_yaw << std::endl;
}

void EkfCore::updateOdometry(double measured_x, double measured_y, 
                            double position_noise_x, double position_noise_y) {
    if (!initialized_) {
        std::cerr << "EKF not initialized!" << std::endl;
        return;
    }

    // TODO(human): 实现里程计更新步骤  
    // 观测模型: z = [x, y]ᵀ (直接观测位置)
    // 观测雅可比矩阵 H = [1, 0, 0]
    //                     [0, 1, 0]
    
    std::cout << "Odometry update: x=" << measured_x << ", y=" << measured_y << std::endl;
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