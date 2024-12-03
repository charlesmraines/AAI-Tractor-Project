from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Start Ignition Gazebo with a custom world file
        ExecuteProcess(
            cmd=['ign', 'gazebo', '/home/andresarias23gmailcom/ros2_ws/gazebo_worlds/car_world.sdf'],
            output='screen'
        ),
        # Spawn the URDF model
        Node(
            package='ros_gz_sim', executable='create',
            arguments=[
                '-file', '/home/andresarias23gmailcom/ros2_ws/src/tractor_description/urdf/tractor_test.urdf',
                '-name', 'small_tractor'
            ],
            output='screen'
        ),
    ])

