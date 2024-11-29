#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "ekf_node.hpp"
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class TestEKFNode : public ::testing::Test
{
protected:
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor exec;

  // Setup function runs before each test
  void SetUp() override
  {
    // Initialize the ROS 2 node
    rclcpp::init(0, nullptr);
    node = std::make_shared<ekf_ros2::EKFNode>();

    // Spin the node in a separate thread for testing purposes
    exec.add_node(node);
  }

  // Teardown function runs after each test
  void TearDown() override
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestEKFNode, TestSensorFusion)
{
  // Create sample sensor data (simulate sensor 1 and 2 data)
  std_msgs::msg::Float64MultiArray sensor1_data;
  sensor1_data.data = {1.0};  // Simulated measurement from sensor 1

  std_msgs::msg::Float64MultiArray sensor2_data;
  sensor2_data.data = {2.0};  // Simulated measurement from sensor 2

  // Create a publisher to send the simulated data
  auto sensor1_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("sensor1_data", 10);
  auto sensor2_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("sensor2_data", 10);

  // Publish the sensor data
  sensor1_pub->publish(sensor1_data);
  sensor2_pub->publish(sensor2_data);

  // Allow some time for the EKF to process the data
  std::this_thread::sleep_for(1s);

  // Check that the fused state is updated after sensor input
  // Create a subscriber to check the fused data
  auto fused_state_received = false;
  auto fused_state_sub = node->create_subscription<std_msgs::msg::Float64MultiArray>(
    "fused_data", 10, [&fused_state_received](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      // Check if the received fused data matches the expected output
      EXPECT_GT(msg->data.size(), 0);  // Check if the message contains data
      fused_state_received = true;     // Set flag once data is received
    });

  // Wait for the fused data to be published
  rclcpp::spin_some(node);  // Process the data (simulate callback execution)
  EXPECT_TRUE(fused_state_received);  // Ensure that we received the fused data
}

TEST_F(TestEKFNode, TestControlInput)
{
  // Simulate control input message
  std_msgs::msg::Float64MultiArray control_input_data;
  control_input_data.data = {0.5};  // Simulated control input

  // Create a publisher to send the control input data
  auto control_input_pub = node->create_publisher<std_msgs::msg::Float64MultiArray>("control_input", 10);

  // Publish control input data
  control_input_pub->publish(control_input_data);

  // Allow time for the EKF to process
  std::this_thread::sleep_for(1s);

}

int main(int argc, char **argv)
{
  // Run all the tests that were declared with TEST()
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
