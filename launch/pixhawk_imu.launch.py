from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [
            FindPackageShare("hardware"),
            "config",
            "pixhawk_imu_default.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="hardware",
                executable="pixhawk_imu_node",
                name="pixhawk_imu",
                output="screen",
                parameters=[config_file],
            )
        ]
    )
