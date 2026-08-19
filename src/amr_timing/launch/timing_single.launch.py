"""
launch/timing_single.launch.py
Examples:
    # idle test (motors enabled, 0 RPM, no rotation)
    ros2 launch amr_timing timing_single.launch.py frequency_hz:=20

    # loaded test (wheels spin at 200 RPM — keep wheels off the floor!)
    ros2 launch amr_timing timing_single.launch.py frequency_hz:=20 test_rpm:=200
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("frequency_hz",  default_value="50"),
        DeclareLaunchArgument("output_dir",    default_value="/tmp/timing_results"),
        DeclareLaunchArgument("mode",          default_value="rs485"),
        DeclareLaunchArgument("port",          default_value="/dev/rs485"),
        DeclareLaunchArgument("baudrate",      default_value="115200"),
        DeclareLaunchArgument("slave_id",      default_value="1"),
        DeclareLaunchArgument("mb_timeout_ms", default_value="150"),
        DeclareLaunchArgument("test_rpm",      default_value="0",
            description="0 = idle test, non-zero = wheels actually spin"),
        DeclareLaunchArgument("run_tag",       default_value="",
            description="Filename suffix; auto = 'idle' or 'rpm<N>'"),

        Node(
            package="amr_timing",
            executable="timing_node",
            name="amr_timing_node",
            output="screen",
            parameters=[{
                "auto_run_all":  False,
                "frequency_hz":  LaunchConfiguration("frequency_hz"),
                "output_dir":    LaunchConfiguration("output_dir"),
                "mode":          LaunchConfiguration("mode"),
                "port":          LaunchConfiguration("port"),
                "baudrate":      LaunchConfiguration("baudrate"),
                "slave_id":      LaunchConfiguration("slave_id"),
                "mb_timeout_ms": LaunchConfiguration("mb_timeout_ms"),
                "test_rpm":      LaunchConfiguration("test_rpm"),
                "run_tag":       LaunchConfiguration("run_tag"),
            }],
        ),
    ])
