import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def try_run_motor_node(context, *args, **kwargs):
    generation_value = LaunchConfiguration('generation').perform(context)

    to_launch = []

    if generation_value == "gen5":
        # ros2_control_node
        # Step 1: Run the stty command to configure the serial port
        to_launch.append(ExecuteProcess(
            cmd=['sudo', 'stty', '-F', '/dev/robot_serial', 'sane'],
            shell=True
        ))

        # Path to the conf.yaml configuration file
        config_file = PathJoinSubstitution(
            [FindPackageShare('ubiquity_motor_ros2'), 'cfg', 'conf.yaml']
        )

        # Step 2: Run the ros2_control_node with parameters
        to_launch.append(launch_ros.actions.Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/ubiquity_velocity_controller/odom', '/odom')         # Remap odom
            ]
        ))

        spawn_cmd = ['ros2', 'run', 'controller_manager', 'spawner', 'ubiquity_velocity_controller']

        param_file_path = os.path.expanduser("~/.ros/params/ubiquity_velocity_controller.yaml")
        if os.path.isfile(param_file_path):
            spawn_cmd.append('--param-file')
            spawn_cmd.append(param_file_path)

        # Spawning the controller using spawner command
        to_launch.append(TimerAction(
            period=3.0,  # delay in seconds
            actions=[
                ExecuteProcess(
                    cmd=spawn_cmd,
                    output='screen'
                )
            ]
        ))
    
    # Add support for gen6 or other generations if needed in the future
    
    return to_launch


def generate_launch_description():
    # Declare the launch arguments
    generation_arg = DeclareLaunchArgument('generation', default_value='gen5')
    
    # Restore arguments for robot description
    camera_arg = DeclareLaunchArgument('camera_extrinsics_file', default_value='')
    lidar_arg = DeclareLaunchArgument('lidar_extrinsics_file', default_value='')
    sonars_arg = DeclareLaunchArgument('sonars_installed', default_value='false')
    shell_arg = DeclareLaunchArgument('shell_installed', default_value='false')
    tower_arg = DeclareLaunchArgument('tower_installed', default_value='false')

    # Path to the magni_description launch file
    magni_description_launch = PathJoinSubstitution([
        FindPackageShare('magni_description'),
        'launch',
        'magni_description.launch.py'
    ])

    # Include the magni_description launch file to publish robot_description
    magni_description_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(magni_description_launch),
        launch_arguments={
            'camera_extrinsics_file': LaunchConfiguration('camera_extrinsics_file'),
            'lidar_extrinsics_file': LaunchConfiguration('lidar_extrinsics_file'),
            'sonars_installed': LaunchConfiguration('sonars_installed'),
            'shell_installed': LaunchConfiguration('shell_installed'),
            'tower_installed': LaunchConfiguration('tower_installed'),
        }.items()
    )

    return launch.LaunchDescription([
        generation_arg,
        camera_arg,
        lidar_arg,
        sonars_arg,
        shell_arg,
        tower_arg,
        magni_description_include,
        OpaqueFunction(function=try_run_motor_node)
    ])
