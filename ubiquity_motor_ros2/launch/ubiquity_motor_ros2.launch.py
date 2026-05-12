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
        # Step 1: Run the stty command to configure the serial port
        to_launch.append(ExecuteProcess(
            cmd=['sudo', 'stty', '-F', '/dev/robot_serial', 'sane'],
            shell=True
        ))

        # Path to the conf.yaml configuration file
        config_file = PathJoinSubstitution(
            [FindPackageShare('ubiquity_motor_ros2'), 'cfg', 'conf.yaml']
        )

        # Step 2: Run the ros2_control_node
        to_launch.append(launch_ros.actions.Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('/ubiquity_velocity_controller/odom', '/odom')
            ]
        ))

        # Step 3: Spawn Joint State Broadcaster
        to_launch.append(TimerAction(
            period=2.0,
            actions=[
                launch_ros.actions.Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen',
                )
            ]
        ))

        # Step 4: Spawn Velocity Controller
        to_launch.append(TimerAction(
            period=4.0,
            actions=[
                launch_ros.actions.Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['ubiquity_velocity_controller', '--controller-manager', '/controller_manager'],
                    output='screen',
                )
            ]
        ))

    return to_launch


def generate_launch_description():
    description_share = os.path.join(get_package_share_directory('magni_description'))

    # Declare the launch arguments
    camera_arg = DeclareLaunchArgument('camera_extrinsics_file', default_value=os.path.join(description_share, 'urdf/extrinsics/camera_extrinsics_forward.yaml'))
    lidar_arg = DeclareLaunchArgument('lidar_extrinsics_file', default_value=os.path.join(description_share, 'urdf/extrinsics/lidar_extrinsics_top_plate_center.yaml'))
    sonars_arg = DeclareLaunchArgument('sonars_installed', default_value='false')
    shell_arg = DeclareLaunchArgument('shell_installed', default_value='false')
    tower_arg = DeclareLaunchArgument('tower_installed', default_value='false')
    generation_arg = DeclareLaunchArgument('generation', default_value='gen5')

    # Path to the magni_description launch file
    magni_description_launch = os.path.join(
        get_package_share_directory('magni_description'),
        'launch',
        'magni_description.launch.py'
    )

    # Include the magni_description launch file
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
        camera_arg,
        lidar_arg,
        sonars_arg,
        shell_arg,
        tower_arg,
        generation_arg,
        magni_description_include,
        OpaqueFunction(function=try_run_motor_node)
    ])
