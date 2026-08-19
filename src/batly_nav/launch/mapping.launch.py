# file: amr_system.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    batly_params_dir   = FindPackageShare("batly_params")
    batly_bringup_dir  = FindPackageShare("batly_bringup")
    slam_tb_share          = FindPackageShare("slam_toolbox")
    nav2_bringup_share     = FindPackageShare("nav2_bringup")


    slam_launch_path   = PathJoinSubstitution([slam_tb_share, 'launch', 'online_async_launch.py'])
    slam_config_path   = PathJoinSubstitution([batly_params_dir, 'config', 'slam.yaml'])

    navigation_launch_path = PathJoinSubstitution([nav2_bringup_share, 'launch', 'navigation_launch.py'])
    nav2_config_path       = PathJoinSubstitution([batly_params_dir, 'config', 'nav.yaml'])

    rviz_config_path = PathJoinSubstitution([batly_params_dir, 'rviz', 'slam.rviz'])
    bringup_launch   = PathJoinSubstitution([batly_bringup_dir, 'launch', 'bringup.launch.py'])

    # --- Launch arguments ---
    rviz_arg  = DeclareLaunchArgument('rviz',             default_value='false', description='Run RViz')
    teleop_arg= DeclareLaunchArgument('teleop_keyboard',  default_value='false', description='Run teleop_keyboard')
    sim_arg   = DeclareLaunchArgument('sim',              default_value='false', description='Enable use_sim_time')

    # --- Env: suppress logs (optional) ---
    env_actions = [
        SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', '50'),  # FATAL
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', ''),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    ]

    # --- Nodes / includes ---
    includes = [
        # Your sensor/driver bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch)
        ),

        # SLAM Toolbox
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('sim'),
                'slam_params_file': slam_config_path
            }.items()
        ),

        # Nav2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch_path),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('sim'),
                'params_file': nav2_config_path
            }.items()
        ),
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('sim')}],
    )

    teleop_proc = ExecuteProcess(
        cmd=['gnome-terminal', '--wait', '--', 'ros2', 'run', 'batly_bringup', 'teleop_keyboard'],
        condition=IfCondition(LaunchConfiguration('teleop_keyboard')),
        output='screen'
    )

    return LaunchDescription(
        env_actions + [rviz_arg, teleop_arg, sim_arg] + includes + [rviz_node, teleop_proc]
    )
