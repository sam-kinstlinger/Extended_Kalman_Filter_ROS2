import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable, PushRosNamespace, TimerAction
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node

def generate_launch_description():
    # Define the path to the ekf_params.yaml file
    pkg_share = os.path.join(os.getenv('AMENT_PREFIX_PATH').split(':')[0], 'share', 'ekf_ros2', 'config')
    ekf_params_yaml = os.path.join(pkg_share, 'ekf_params.yaml')

    # Create the LaunchDescription object
    return LaunchDescription([
        # Declare arguments for flexibility in parameterization
        DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation time'
        ),

        # Node that runs the EKF node
        Node(
            package='ekf_ros2',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_params_yaml],
            remappings=[('/sensor1_data', '/sensor1/data'),  # Optional topic remapping
                        ('/sensor2_data', '/sensor2/data'),
                        ('/control_input', '/control/input'),
                        ('/fused_data', '/fused/estimates')],
            # Add use_sim_time argument if desired
            arguments=[LaunchConfiguration('use_sim_time')],
        ),

        # Add any other actions or nodes if necessary (e.g., logging, namespaces, etc.)
        LogInfo(
            condition=LaunchConfiguration('use_sim_time'),
            msg="Launching EKF Node with simulation time enabled."
        ),
    ])
