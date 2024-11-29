# Extended_Kalman_Filter_ROS2

### Overview
Implemented a dynamic Extended Kalman Filter (EKF) ROS2 package for real-time sensor fusion, leveraging C++, Eigen, and ROS2 libraries to enhance state estimation and control accuracy

### Project Structure
- **ekf.cpp** - Contains the implementation of the EKF algorithm, including prediction and update steps.
-  **ekf.hpp** - Header file for ekf.cpp, containing class definitions and method declarations.
-  **ekf_node.cpp** - Implements the ROS2 node responsible for handling sensor fusion. It subscribes to sensor data topics and control inputs, applies the EKF, and publishes the fused state estimates.
- **ekf_node.hpp** - Header file for ekf_node.cpp, containing ROS2 node class definitions.
- **ekf_params.yaml** - YAML configuration file that defines parameters for the EKF algorithm, such as sensor noise and initial state values.
- **ekf_test.cpp** - Unit tests for the EKF algorithm to verify functionality.
- **ekf_launch.py** - Launch file to start the EKF node with ROS2, including necessary parameters and configurations.
- **CMakeLists.txt** - Build configuration file for the ROS2 package.

### Use
1. Clone the repository into your ROS2 workspace
2. Install dependencies using rosdep
3. Build the project with colcon
4. Source the workspace
5. Launch the EKF node using the launch file
