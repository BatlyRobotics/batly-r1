from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

MAP_NAME = 'map_aun'

def generate_launch_description():
    # --- Package share dirs ---
    batly_params_dir  = FindPackageShare("batly_params")
    batly_bringup_dir = FindPackageShare("batly_bringup")
    # laser_filters_dir      = FindPackageShare("laser_filters")

    # --- Paths ---
    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    rviz_config_path = PathJoinSubstitution(
        [batly_params_dir, 'rviz', 'navigation.rviz']
    )
    default_map_path = PathJoinSubstitution(
        [batly_params_dir, 'maps', f'{MAP_NAME}.yaml']
    )
    nav2_config_path = PathJoinSubstitution(
        [batly_params_dir, 'config', 'nav.yaml']
    )
    bringup_launch = PathJoinSubstitution(
        [batly_bringup_dir, 'launch', 'bringup.launch.py']
    )

    # --- Environment (reduce log verbosity) ---
    env_actions = [
        SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', '50'),  # FATAL
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', ''),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    ]

    # --- Launch args ---
    sim_arg = DeclareLaunchArgument(
        name='sim',
        default_value='false',
        description='Enable use_sim_time to true'
    )
    map_arg = DeclareLaunchArgument(
        name='map',
        default_value=default_map_path,
        description='Navigation map path'
    )
    rviz_arg = DeclareLaunchArgument(
        name='rviz',
        default_value='false',
        description='Run RViz'
    )

    # --- Includes / Nodes ---
    includes = [

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch)
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            condition=UnlessCondition(LaunchConfiguration("sim")),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items(),
        ),
    ]

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{'use_sim_time': LaunchConfiguration("sim")}],
    )

    return LaunchDescription(
        env_actions + [sim_arg, map_arg, rviz_arg] + includes + [rviz_node]
    )

