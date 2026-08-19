"""
launch/timing_all.launch.py — full matrix sweep.

Runs the complete matrix {0, 200} RPM × {20, 50, 100, 200} Hz = 8 runs,
sequentially, in one launch. Each run is TOTAL_CYCLES (10,000) cycles.
In auto mode the test_rpm and run_tag arguments are IGNORED — the node
iterates the matrix internally and names files idle_* / rpm200_*.

  ros2 launch amr_timing timing_all.launch.py

Approx wall-clock: idle block ≈ (500+200+100+50) s, loaded block same,
so the full sweep is ~28 min. For the 200 RPM runs keep the wheels off
the floor (robot on blocks/stand).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("output_dir",    default_value="/tmp/timing_results"),
        DeclareLaunchArgument("mode",          default_value="rs485"),
        DeclareLaunchArgument("port",          default_value="/dev/rs485"),
        DeclareLaunchArgument("baudrate",      default_value="115200"),
        DeclareLaunchArgument("slave_id",      default_value="1"),
        DeclareLaunchArgument("mb_timeout_ms", default_value="150"),
        DeclareLaunchArgument("test_rpm",      default_value="0",
            description="0 = idle test, non-zero = wheels actually spin"),
        DeclareLaunchArgument("run_tag",       default_value=""),

        Node(
            package="amr_timing",
            executable="timing_node",
            name="amr_timing_node",
            output="screen",
            parameters=[{
                "auto_run_all":  True,
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
