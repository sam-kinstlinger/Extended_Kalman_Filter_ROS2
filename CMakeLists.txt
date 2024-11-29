cmake_minimum_required(VERSION 3.5)
project(ekf_ros2)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

# Declare the executable for your EKF node
add_executable(ekf_node src/ekf_node.cpp)

# Link your executable to the necessary dependencies
ament_target_dependencies(
  ekf_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  Eigen3
  ekf  # Assuming you have a library target for EKF
)

# Install the executable
install(TARGETS ekf_node DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install parameters file
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install header and source files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY src/
  DESTINATION src/${PROJECT_NAME}
)

# Install the libraries if you have them
install(TARGETS ekf
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
