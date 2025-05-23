import os
import launch
import launch_ros.actions
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, IncludeLaunchDescription, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    description_share = os.path.join(get_package_share_directory('magni_description'))

    # Declare the launch arguments
    camera_arg = DeclareLaunchArgument('camera_extrinsics_file', default_value=os.path.join(description_share, 'urdf/extrinsics/camera_extrinsics_forward.yaml'))
    lidar_arg = DeclareLaunchArgument('lidar_extrinsics_file', default_value=os.path.join(description_share, 'urdf/extrinsics/lidar_extrinsics_top_plate_center.yaml'))
    sonars_arg = DeclareLaunchArgument('sonars_installed', default_value='false')
    shell_arg = DeclareLaunchArgument('shell_installed', default_value='false')
    tower_arg = DeclareLaunchArgument('tower_installed', default_value='false')

    # Use LaunchConfiguration to get the values
    camera_file = LaunchConfiguration('camera_extrinsics_file')
    lidar_file = LaunchConfiguration('lidar_extrinsics_file')
    sonars = LaunchConfiguration('sonars_installed')
    shell = LaunchConfiguration('shell_installed')
    tower = LaunchConfiguration('tower_installed')

    # Path to the magni_description launch file
    magni_description_launch = os.path.join(
        get_package_share_directory('magni_description'),
        'launch',
        'description.launch.py'
    )

    # Include the magni_description launch file
    magni_description_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(magni_description_launch),
        launch_arguments={
            'camera_extrinsics_file': camera_file,
            'lidar_extrinsics_file': lidar_file,
            'sonars_installed': sonars,
            'shell_installed': shell,
            'tower_installed': tower,
        }.items()
    )

    # ros2_control_node
    # Step 1: Run the stty command to configure the serial port
    serial_config = ExecuteProcess(
        # cmd=['sudo', 'stty', '-F', '/dev/ttyS0', 'sane'],
        cmd=['sudo', 'stty', '-F', '/dev/ttyAMA0', 'sane'],
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
        remappings=[
            ('/ubiquity_velocity_controller/cmd_vel', '/cmd_vel'),  # Remap the cmd_vel topic
            ('/ubiquity_velocity_controller/odom', '/odom')         # Remap odom
        ]
    )
    
    # Spawning the controller using spawner command
    spawn_controller = TimerAction(
        period=3.0,  # delay in seconds
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'controller_manager', 'spawner', 'ubiquity_velocity_controller'
                ],
                output='screen'
            )
        ]
    )

    # Return the launch description without ros2_control_node
    return launch.LaunchDescription([
        # run_xacro,  # Runs the xacro process
        # robot_state_publisher,  # Starts the robot_state_publisher with xacro output
        camera_arg,
        lidar_arg,
        sonars_arg,
        shell_arg,
        tower_arg,
        magni_description_include,
        serial_config,
        controller_node,
        spawn_controller
    ])
