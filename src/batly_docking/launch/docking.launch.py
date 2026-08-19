from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    parent_frame = LaunchConfiguration("parent_frame")
    child_frame  = LaunchConfiguration("child_frame")
    output_frame = LaunchConfiguration("output_frame")
    tag_yaml     = LaunchConfiguration("tag_yaml")
    view         = LaunchConfiguration("view")

    # Path to the stereo camera launch file
    stereo_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("elp_stereo_camera"),
                "launch",
                "stereo_camera_launch.py"
            ])
        ]),
        launch_arguments={
            "video_device": "/dev/elp_camera",
            "fps": "30",
            "image_width": "1280",
            "image_height": "480",
            "view": "false",
        }.items()
    )

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag_node",
        output="screen",
        parameters=[tag_yaml, {"use_sim_time": use_sim_time}],
        remappings=[
            ("image_rect",  "/stereo/full/image_raw"),
            ("camera_info", "/stereo/full/camera_info"),
            ("detections",  "/apriltag/detections"),
        ],
    )

    detected_dock_pose_publisher = Node(
        package="batly_docking",
        executable="detected_dock_pose_publisher",
        name="detected_dock_pose_publisher",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("parent_frame", default_value="camera_optical_link"),
        DeclareLaunchArgument("child_frame", default_value="tag0"),
        DeclareLaunchArgument("output_frame", default_value="odom"),
        DeclareLaunchArgument(
            "tag_yaml",
            default_value=PathJoinSubstitution([
                FindPackageShare("batly_params"), "config", "tag.yaml"
            ])
        ),
        stereo_camera_launch,
        apriltag_node,
        detected_dock_pose_publisher,
    ])
