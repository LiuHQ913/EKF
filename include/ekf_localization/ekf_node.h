#include <memory>
#include <chrono>
#include <deque>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "ekf_localization/ekf_core.h"

namespace ekf_localization {

/**
 * @brief EKF定位节点，融合里程计和IMU数据进行机器人定位
 * 
 * 订阅话题:
 * - /odom (nav_msgs::msg::Odometry): 里程计数据
 * - /imu (sensor_msgs::msg::Imu): IMU数据
 * 
 * 发布话题:
 * - /odometry/filtered (nav_msgs::msg::Odometry): 滤波后的定位结果
 */
class EkfNode : public rclcpp::Node 
{
public:
    /**
     * @brief 构造函数，初始化ROS 2节点和EKF滤波器
     */
    EkfNode();

private:
    // EKF算法核心
    std::shared_ptr<EkfCore> ekf_core_ptr_;
    
    // ROS 2 订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
    
    // ROS 2 发布者
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr filtered_pub_;
    
    // 时间管理
    rclcpp::Time last_prediction_time_;
    bool has_first_odom_;
    
    // 噪声参数(后续可从参数服务器读取)
    static constexpr double ODOM_NOISE_X  = 0.01;     // 里程计x方向噪声方差
    static constexpr double ODOM_NOISE_Y  = 0.01;     // 里程计y方向噪声方差  
    static constexpr double IMU_NOISE_YAW = 0.001;    // IMU角度噪声方差   

private:
    // TODO(human): 核心成员变量声明

    // 初始化
    std::atomic<bool> isInitialized_;

    // 缓存
    std::deque<std::pair<rclcpp::Time, Eigen::VectorXd>> imu_buf_;
    std::deque<std::pair<rclcpp::Time, Eigen::VectorXd>> odom_buf_;

private:
    /**
     * @brief 里程计数据回调函数
     * @param msg 里程计消息
     */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    
    /**
     * @brief IMU数据回调函数  
     * @param msg IMU消息
     */
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    
    /**
     * @brief 发布融合后的里程计数据
     * @param state 当前状态向量 [x, y, θ]
     * @param covariance 状态协方差矩阵
     * @param timestamp 时间戳
     */
    void publishFilteredOdometry(const Eigen::Vector3d& state,
                                const Eigen::Matrix3d& covariance,
                                const rclcpp::Time& timestamp);
    
    /**
     * @brief 从四元数中提取yaw角度
     * @param q 四元数
     * @return yaw角度(弧度)
     */
    double extractYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

};

} // namespace ekf_localization