from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define the path to your URDF file
    urdf_file_path = '/home/andresarias23gmailcom/ros2_ws/src/tractor_description/urdf/tractor_test.urdf'

    return LaunchDescription([
        # Launch argument for use_sim_time (enables simulation time)
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Robot State Publisher Node to broadcast state based on URDF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[urdf_file_path]
        ),
        
        # Gazebo Spawn Entity node to spawn the robot model into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=['-entity', 'tractor', '-file', urdf_file_path]
        ),
    ])

