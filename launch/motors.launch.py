import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the path to the xacro file
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('magni_description'), 'urdf', 'magni.urdf.xacro']
    )

    # Define the command to run xacro as a subprocess
    xacro_command = Command(
        [
            'xacro ', xacro_file,
        ]
    )

    # Use ExecuteProcess to run the xacro command
    run_xacro = ExecuteProcess(
        cmd=['xacro', xacro_file],
        output='screen',
        shell=True
    )

    # Use the robot_description generated from the xacro command
    robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': xacro_command}]
    )

    # ros2_control_node
    # Step 1: Run the stty command to configure the serial port
    serial_config = ExecuteProcess(
        cmd=['sudo', 'stty', '-F', '/dev/ttyS0', 'sane'],
        shell=True
    )

        # Path to the test.yaml configuration file
    config_file = PathJoinSubstitution(
        [FindPackageShare('ubiquity_motor_ros2'), 'cfg', 'conf.yaml']
    )

    # Step 2: Run the ros2_control_node with parameters
    controller_node = launch_ros.actions.Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[config_file],
    )
    
    # Spawning the controller using spawner command
    spawn_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'controller_manager', 'spawner', 'ubiquity_velocity_controller'
        ],
        output='screen'
    )

    # Return the launch description without ros2_control_node
    return launch.LaunchDescription([
        run_xacro,  # Runs the xacro process
        robot_state_publisher,  # Starts the robot_state_publisher with xacro output
        serial_config,
        controller_node,
        spawn_controller
    ])
