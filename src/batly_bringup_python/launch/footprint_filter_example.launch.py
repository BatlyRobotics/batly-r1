from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [
            get_package_share_directory("roboverii_params"),
            "config",
            "footprint_filter_example.yaml",
        ]
    )

    return LaunchDescription(
        [
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                parameters=[config_file],
            )
        ]
    )
