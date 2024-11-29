#include "ekf_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "yaml-cpp/yaml.h"  // For reading parameters from a YAML file
#include "ekf.hpp"  // Include your ExtendedKalmanFilter class

EKFNode::EKFNode()
    : Node("ekf_node")
{
    // Load parameters from the YAML file
    this->declare_parameter("dt", 0.1);  // Default time step
    this->declare_parameter("state_dim", 2);  // Default state dimension
    this->declare_parameter("control_dim", 1);  // Default control dimension
    this->declare_parameter("sensor1_noise_cov", 0.1);  // Default sensor noise covariance
    this->declare_parameter("sensor2_noise_cov", 0.1);  // Default sensor noise covariance
    this->declare_parameter("process_noise_cov", 0.01);  // Default process noise covariance

    // Retrieve parameters from the YAML file
    this->get_parameter("dt", dt_);
    this->get_parameter("state_dim", state_dim_);
    this->get_parameter("control_dim", control_dim_);
    this->get_parameter("sensor1_noise_cov", sensor1_noise_cov_);
    this->get_parameter("sensor2_noise_cov", sensor2_noise_cov_);
    this->get_parameter("process_noise_cov", process_noise_cov_);

    // Initialize the EKF instance
    ekf_ = std::make_shared<ExtendedKalmanFilter>(state_dim_, control_dim_, dt_);

    // Set up subscriptions
    sensor1_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "sensor1_data", 10, std::bind(&EKFNode::sensor1_callback, this, std::placeholders::_1));
    sensor2_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "sensor2_data", 10, std::bind(&EKFNode::sensor2_callback, this, std::placeholders::_1));
    control_input_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "control_input", 10, std::bind(&EKFNode::control_input_callback, this, std::placeholders::_1));

    // Set up publisher
    fused_state_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("fused_data", 10);

    // Create a timer for periodic prediction steps
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
        std::bind(&EKFNode::prediction_step, this));

    // Log to confirm initialization
    RCLCPP_INFO(this->get_logger(), "EKF Node Initialized");
}

void EKFNode::sensor1_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Sensor 1 callback: updates the EKF with new sensor data
    auto z = Eigen::VectorXd(1);
    z(0) = msg->data[0];  // Convert sensor data to a measurement vector
    ekf_->update(z);  // Update the EKF with the measurement
    publish_fused_state();  // Publish the updated state
}

void EKFNode::sensor2_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Sensor 2 callback: updates the EKF with new sensor data
    auto z = Eigen::VectorXd(1);
    z(0) = msg->data[0];  // Convert sensor data to a measurement vector
    ekf_->update(z);  // Update the EKF with the measurement
    publish_fused_state();  // Publish the updated state
}

void EKFNode::control_input_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
    // Control input callback: updates the EKF with control inputs
    auto u = Eigen::VectorXd(1);
    u(0) = msg->data[0];  // Convert control input to a vector
    ekf_->set_control_input(u);  // Update the EKF with the control input
}

void EKFNode::prediction_step()
{
    // Prediction step callback: runs the EKF prediction step
    ekf_->predict();  // Perform the EKF prediction step
}

void EKFNode::publish_fused_state()
{
    // Publish the fused state as a message
    std_msgs::msg::Float64MultiArray fused_msg;
    fused_msg.data.clear();
    fused_msg.data.push_back(ekf_->get_state()(0));  // Example: publish the first state variable
    fused_msg.data.push_back(ekf_->get_state()(1));  // Example: publish the second state variable
    fused_state_pub_->publish(fused_msg);  // Publish the fused state message
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);  // Initialize ROS2 communication
    rclcpp::spin(std::make_shared<EKFNode>());  // Spin the node to process callbacks
    rclcpp::shutdown();  // Shutdown ROS2 communication
    return 0;
}
