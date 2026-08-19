import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the shared directory paths
    batly_params_dir = get_package_share_directory("batly_params")
    batly_bringup_dir = get_package_share_directory("batly_bringup")

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_SEVERITY_THRESHOLD', '30'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_OUTPUT_FORMAT', ''),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        Node(
            package='batly_bringup',
            executable='zlac_node',
            name='zlac_node',
            arguments=['--ros-args', '--log-level', 'warn'],
            output='screen',
        ),


        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            remappings=[("odometry/filtered", "odom")],
            parameters=[os.path.join(batly_params_dir, 'config', 'ekf.yaml')],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),


        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link',
            arguments=['0.0', '0', '0.175', '0', '0', '0', 'base_footprint', 'base_link'],
            output='log',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['0.25', '0', '0.13', '3.14', '0', '0', 'base_link', 'laser'],
            output='log',
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_imu_link',
            arguments=['0', '0', '0.04', '0', '0', '0', 'base_link', 'imu_link'],
            output='log',
        ),


        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(batly_bringup_dir, 'launch', 'laser.launch.py')
           )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(batly_bringup_dir, 'launch', 'footprint_filter_example.launch.py')
           )
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='microros_agent',
            output='screen',
            arguments=['serial', '--dev', '/dev/uros', '-b', '115200'],
        ),



    ])
