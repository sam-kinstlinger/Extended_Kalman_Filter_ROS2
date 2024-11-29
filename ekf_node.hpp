#ifndef EKF_NODE_HPP
#define EKF_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "ekf.hpp"  // Include the header for your ExtendedKalmanFilter class
#include "yaml-cpp/yaml.h"  // For loading parameters from a YAML file

using namespace std::chrono_literals;

class EKFNode : public rclcpp::Node
{
public:
    EKFNode();  // Constructor to initialize the node and parameters

private:
    // Callback functions for sensor and control inputs
    void sensor1_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void sensor2_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
    void control_input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

    // Timer callback to trigger the EKF prediction step
    void prediction_step();

    // Function to publish the fused state
    void publish_fused_state();

    // ROS2 subscription objects
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensor1_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sensor2_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr control_input_sub_;

    // ROS2 publisher object
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr fused_state_pub_;

    // ROS2 timer for periodic callbacks
    rclcpp::TimerBase::SharedPtr timer_;

    // EKF instance
    std::shared_ptr<ExtendedKalmanFilter> ekf_;

    // Parameters for EKF node loaded from YAML configuration
    double dt_;  // Time step for prediction
    int state_dim_;  // State dimension
    int control_dim_;  // Control input dimension
    double sensor1_noise_cov_;  // Sensor 1 noise covariance
    double sensor2_noise_cov_;  // Sensor 2 noise covariance
    double process_noise_cov_;  // Process noise covariance
};

#endif  // EKF_NODE_HPP
