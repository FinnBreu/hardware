from hippo_common.launch_helper import (
    LaunchArgsDict,
    config_file_path,
    declare_vehicle_name_and_sim_time,
    launch_file_source,
)
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression


def declare_launch_args(launch_description: LaunchDescription):
    declare_vehicle_name_and_sim_time(
        launch_description=launch_description, use_sim_time_default='false'
    )
    pkg = 'hippo_control'
    config_file = config_file_path(
        pkg, 'actuator_mixer/hippocampus_normalized_default.yaml'
    )
    action = DeclareLaunchArgument('mixer_path', default_value=config_file)
    launch_description.add_action(action)
    action = DeclareLaunchArgument(
        'imu_source',
        default_value='ism330',
        choices=['ism330', 'pixhawk', 'none'],
        description='IMU source to launch.',
    )
    launch_description.add_action(action)
    action = DeclareLaunchArgument(
        'use_pixhawk',
        default_value='false',
        choices=['true', 'false'],
        description='Launch Pixhawk/FCU serial bridge processes.',
    )
    launch_description.add_action(action)


def add_mixer_node():
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    return Node(
        package='hippo_control',
        executable='actuator_mixer_node',
        parameters=[
            args,
            LaunchConfiguration('mixer_path'),
        ],
        output='screen',
    )


def include_vertical_camera_node():
    pkg = 'mjpeg_cam'
    source = launch_file_source(pkg, 'ov9281.launch.py')
    args = LaunchArgsDict()
    args.add_vehicle_name_and_sim_time()
    args['camera_name'] = 'vertical_camera'
    return IncludeLaunchDescription(source, launch_arguments=args.items())


def add_micro_xrce_agent():
    action = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent',
            'serial',
            '--dev',
            '/dev/fcu_data',
            '-b',
            '921600',
        ],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('use_pixhawk')),
    )
    return action


def add_nsh_node():
    return Node(
        executable='nsh_node',
        package='hardware',
        condition=IfCondition(LaunchConfiguration('use_pixhawk')),
    )


def add_barometer_node():
    config_file = PathJoinSubstitution(
        [
            FindPackageShare('hardware'),
            'config',
            'barometer_default.yaml',
        ]
    )
    return Node(
        executable='barometer',
        package='hardware',
        parameters=[config_file],
        output='screen',
    )


def add_ism330_imu_node():
    config_file = PathJoinSubstitution(
        [
            FindPackageShare('hardware'),
            'config',
            'imu_default.yaml',
        ]
    )
    return Node(
        executable='imu_node',
        package='hardware',
        name='imu',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('imu_source'), "' == 'ism330'"]
            )
        ),
    )


def add_pixhawk_imu_node():
    config_file = PathJoinSubstitution(
        [
            FindPackageShare('hardware'),
            'config',
            'pixhawk_imu_default.yaml',
        ]
    )
    return Node(
        executable='pixhawk_imu_node',
        package='hardware',
        name='pixhawk_imu',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(
                ["'", LaunchConfiguration('imu_source'), "' == 'pixhawk'"]
            )
        ),
    )


def add_mavlink_routerd():
    action = ExecuteProcess(
        cmd=['mavlink-routerd'],
        output='screen',
        emulate_tty=True,
        # respan required because a FCU reboot will kill mavlink-routerd
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(LaunchConfiguration('use_pixhawk')),
    )
    return action


def add_esc_commander():
    return Node(
        executable='esc_commander_node',
        package='esc',
    )


def generate_launch_description():
    launch_description = LaunchDescription()
    declare_launch_args(launch_description=launch_description)
    actions = [
        include_vertical_camera_node(),
        GroupAction(
            [
                PushROSNamespace(LaunchConfiguration('vehicle_name')),
                add_esc_commander(),
                add_mixer_node(),
                add_micro_xrce_agent(),
                add_mavlink_routerd(),
                add_nsh_node(),
                add_barometer_node(),
                add_ism330_imu_node(),
                add_pixhawk_imu_node(),
            ],
            launch_configurations={'camera_name': 'vertical_camera'},
        ),
    ]
    for action in actions:
        launch_description.add_action(action)
    return launch_description
