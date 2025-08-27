#include "ekf_localization/ekf_node.h"

#include <glog/logging.h>

namespace ekf_localization {

EkfNode::EkfNode() : Node("ekf_localization_node"), 
                     ekf_core_ptr_(std::make_shared<EkfCore>()),
                     has_first_odom_(false),
                     isInitialized_(false) {
    
    // 创建订阅者，使用默认QoS设置
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&EkfNode::odomCallback, this, std::placeholders::_1));
    
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu", 10, 
        std::bind(&EkfNode::imuCallback, this, std::placeholders::_1));
    
    // 创建发布者
    filtered_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
    
    if(!isInitialized_){
        Eigen::Vector3d initial_state(0.0, 0.0, 0.0);
        Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity() * 0.1;
        ekf_core_ptr_->initialize(initial_state, initial_covariance);
        isInitialized_ = true;
    }
    
    LOG(INFO) << "EKF Localization Node initialized";
    LOG(INFO) << "Subscribing to: /odom, /imu";
    LOG(INFO) << "Publishing  to: /odometry/filtered";
}

void EkfNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if(!isInitialized_){
        LOG(ERROR) << "EKF not initialized!";
        return;
    }

    rclcpp::Time current_time = msg->header.stamp;
    
    // 1. 处理首次里程计数据 - 更新EKF状态为真实位置
    if(!has_first_odom_) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y; 
        double yaw = extractYawFromQuaternion(msg->pose.pose.orientation);
        
        // 用实际里程计数据重新初始化EKF状态
        Eigen::Vector3d initial_state(x, y, yaw);
        Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity() * 0.1;
        ekf_core_ptr_->initialize(initial_state, initial_covariance);
        
        has_first_odom_ = true;
        last_prediction_time_ = current_time;
        
        LOG(INFO) << "EKF state initialized with first odometry data: "
                  << "x=" << x << ", y=" << y << ", yaw=" << yaw;
        
        // 发布初始状态
        publishFilteredOdometry(ekf_core_ptr_->getState(), 
                                ekf_core_ptr_->getCovariance(), 
                                current_time);
        return;
    }
    
    // 2. 计算时间间隔用于预测步骤
    double dt = (current_time - last_prediction_time_).seconds();
    if (dt <= 0) {
        LOG(WARNING) << "Non-positive time step: " << dt;
        return;
    }
    
    // 3. 从里程计消息提取速度信息
    double linear_velocity = msg->twist.twist.linear.x;
    double angular_velocity = msg->twist.twist.angular.z;
    
    // 4. 数据验证和安全检查
    // 限制时间步长在合理范围内
    if (dt > 1.0) {
        LOG(WARNING) << "Time step too large: " << dt << "s, skipping update";
        return;
    }
    if (dt < 0.001) {
        LOG(WARNING) << "Time step too small: " << dt << "s, skipping update";
        return;
    }
    
    // 验证速度数据的合理性
    if (std::abs(linear_velocity) > 10.0) {
        LOG(WARNING) << "Abnormal linear velocity: " << linear_velocity << " m/s";
        linear_velocity = std::copysign(10.0, linear_velocity);  // 限幅
    }
    if (std::abs(angular_velocity) > 5.0) {
        LOG(WARNING) << "Abnormal angular velocity: " << angular_velocity << " rad/s";
        angular_velocity = std::copysign(5.0, angular_velocity);  // 限幅
    }
    
    // 5. EKF预测步骤 - 使用运动模型
    ekf_core_ptr_->predict(linear_velocity, angular_velocity, dt);
    
    // 6. EKF更新步骤 - 使用里程计位置观测
    double measured_x = msg->pose.pose.position.x;
    double measured_y = msg->pose.pose.position.y;
    ekf_core_ptr_->updateOdometry(measured_x, measured_y, ODOM_NOISE_X, ODOM_NOISE_Y);
    
    // 7. 发布融合后的结果
    publishFilteredOdometry(ekf_core_ptr_->getState(), 
                            ekf_core_ptr_->getCovariance(), 
                            current_time);
    
    last_prediction_time_ = current_time;
}

void EkfNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    // TODO(human): 实现IMU回调函数逻辑
    // 1. 检查EKF是否已初始化
    // 2. 从四元数中提取yaw角度
    // 3. 调用EKF的updateImu步骤
    // 4. 发布融合后的结果
}

void EkfNode::publishFilteredOdometry(const Eigen::Vector3d& state,
                                    const Eigen::Matrix3d& covariance,
                                    const rclcpp::Time& timestamp) {
    // 1. 创建nav_msgs::msg::Odometry消息
    auto filtered_msg = std::make_unique<nav_msgs::msg::Odometry>();
    
    // 2. 设置消息头
    filtered_msg->header.stamp = timestamp;
    filtered_msg->header.frame_id = "odom";
    filtered_msg->child_frame_id = "base_link";
    
    // 3. 设置位置信息 (x, y, z=0)
    filtered_msg->pose.pose.position.x = state[0];
    filtered_msg->pose.pose.position.y = state[1]; 
    filtered_msg->pose.pose.position.z = 0.0;
    
    // 4. 将yaw角度转换为四元数并设置方向
    double yaw = state[2];
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    filtered_msg->pose.pose.orientation.x = q.x();
    filtered_msg->pose.pose.orientation.y = q.y();
    filtered_msg->pose.pose.orientation.z = q.z();
    filtered_msg->pose.pose.orientation.w = q.w();
    
    // 5. 设置协方差矩阵 (6x6矩阵，只填充x,y,yaw相关部分)
    std::fill(filtered_msg->pose.covariance.begin(), filtered_msg->pose.covariance.end(), 0.0);
    filtered_msg->pose.covariance[0] = covariance(0,0);   // x-x
    filtered_msg->pose.covariance[1] = covariance(0,1);   // x-y  
    filtered_msg->pose.covariance[6] = covariance(1,0);   // y-x
    filtered_msg->pose.covariance[7] = covariance(1,1);   // y-y
    filtered_msg->pose.covariance[35] = covariance(2,2);  // yaw-yaw
    
    // 速度信息设为0（简化实现）
    filtered_msg->twist.twist.linear.x = 0.0;
    filtered_msg->twist.twist.linear.y = 0.0;
    filtered_msg->twist.twist.angular.z = 0.0;
    std::fill(filtered_msg->twist.covariance.begin(), filtered_msg->twist.covariance.end(), 0.0);
    
    // 6. 发布消息
    filtered_pub_->publish(std::move(filtered_msg));
    
    LOG(INFO) << "Published filtered odometry: x=" << state[0] 
              << ", y=" << state[1] << ", yaw=" << state[2];
}

double EkfNode::extractYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

} // namespace ekf_localization

// Main函数
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ekf_localization::EkfNode>();
    
    RCLCPP_INFO(node->get_logger(), "Starting EKF Localization Node...");
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}