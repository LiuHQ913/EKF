#pragma once

#include <Eigen/Dense>

#include <glog/logging.h>

#include "ekf_localization/utils.hpp"

namespace ekf_localization {

/**
 * @brief 2D Extended Kalman Filter for robot localization
 * 
 * State vector: [x, y, θ]ᵀ where:
 * - x, y: position in 2D space (meters)  
 * - θ: orientation angle (radians)
 */
class EkfCore {
public:
    /**
     * @brief Constructor with default initialization
     */
    EkfCore();

    /**
     * @brief Initialize the filter with initial state and covariance
     * @param initial_state Initial state vector [x, y, θ]ᵀ
     * @param initial_covariance Initial state covariance matrix (3x3)
     */
    void initialize(const Eigen::Vector3d& initial_state, 
                    const Eigen::Matrix3d& initial_covariance);

    /**
     * @brief Prediction step using odometry motion model
     * @param linear_velocity Linear velocity in x direction (m/s)
     * @param angular_velocity Angular velocity (rad/s) 
     * @param dt Time step (seconds)
     */
    void predict(double linear_velocity, double angular_velocity, double dt);

    /**
     * @brief Update step using IMU yaw measurement
     * @param measured_yaw Measured yaw angle from IMU (radians)
     * @param measurement_noise Measurement noise variance
     */
    void updateImu(double measured_yaw, double measurement_noise);

    /**
     * @brief Update step using odometry position measurement  
     * @param measured_x Measured x position (meters)
     * @param measured_y Measured y position (meters)
     * @param position_noise_x Position noise variance in x direction
     * @param position_noise_y Position noise variance in y direction
     */
    void updateOdometry(double measured_x, double measured_y, 
                       double position_noise_x, double position_noise_y);

    /**
     * @brief Get current state estimate
     * @return Current state vector [x, y, θ]ᵀ
     */
    const Eigen::Vector3d& getState() const { return state_; }

    /**
     * @brief Get current state covariance  
     * @return Current state covariance matrix (3x3)
     */
    const Eigen::Matrix3d& getCovariance() const { return covariance_; }

    /**
     * @brief Check if filter is initialized
     * @return True if initialized, false otherwise
     */
    bool isInitialized() const { return initialized_; }

private:
    // Process noise covariance  
    Eigen::Matrix3d process_noise_;   // Q matrix

    // State and covariance
    Eigen::Vector3d state_;           // [x, y, θ]ᵀ
    Eigen::Vector3d state_pre_;       // [x, y, θ]ᵀ
    Eigen::Matrix3d covariance_;      // State covariance matrix P

    // measurement
    Eigen::Vector3d measurement_;       // [x, y, θ]ᵀ

    // Jacobian state matrix
    Eigen::Matrix3d jacobian_F_; // F matrix
    // Jacobian measurement matrix
    Eigen::Matrix3d jacobian_H_; // H matrix

    // update IMU
    double S_imu_;
    double R_imu_;
    Eigen::Vector3d K_imu_;
    Eigen::Vector3d state_imu_; // posterior
    Eigen::Matrix3d covariance_imu_;

    // update Odometry
    Eigen::Matrix2d S_odom_;
    Eigen::Matrix2d R_odom_;
    Eigen::Matrix<double, 3, 2> K_odom_;  // 卡尔曼增益矩阵 (3x2)
    Eigen::Vector3d state_odom_; // posterior
    Eigen::Matrix3d covariance_odom_;
    
    // Filter status
    bool initialized_;


private:   
    /**
     * @brief Compute state transition Jacobian matrix F
     * @param vx Linear velocity in x direction
     * @param theta Current orientation angle
     * @param dt Time step
     * @return Jacobian matrix F (3x3)
     */
    Eigen::Matrix3d computeStateJacobian(double vx, double theta, double dt);

    /**
     * @brief Update the state vector
     * @param state_posterior Posterior state vector
     */
    void updateState(const Eigen::Vector3d& state_posterior);

    /**
     * @brief Update the covariance matrix
     * @param covariance_posterior Posterior covariance matrix
     */
    void updateCovariance(const Eigen::Matrix3d& covariance_posterior);
};

} // namespace ekf_localization