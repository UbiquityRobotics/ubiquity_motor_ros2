from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',  # Correct executable for spawning controllers
            arguments=['diff_drive_controller'],  # Controller name
            parameters=['/home/ubuntu/ros2_ws/src/ubiquity_motor_ros2/cfg/test.yaml'],  # Path to your YAML config file
            output='screen',
        ),
    ])
