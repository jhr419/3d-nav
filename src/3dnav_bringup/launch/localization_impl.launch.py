import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file, ws_path


def default_visualization_map() -> str:
    visualization_map = Path(ws_path("maps", "map_visualization.pcd"))
    if visualization_map.exists():
        return str(visualization_map)
    return ws_path("maps", "map_preprocessed.pcd")


def generate_launch_description():
    visualization_map = Path(ws_path("maps", "map_visualization.pcd"))
    fallback_message = []
    if not visualization_map.exists():
        fallback_message.append(
            LogInfo(
                msg=[
                    "[WARN] maps/map_visualization.pcd not found; localization visualization falls back to maps/map_preprocessed.pcd"
                ]
            )
        )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument("start_fastlio_rviz", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("map_pcd_path", default_value=ws_path("maps", "map_preprocessed.pcd")),
        DeclareLaunchArgument("icp_map_pcd_path", default_value=ws_path("maps", "map_preprocessed.pcd")),
        DeclareLaunchArgument("visualization_map_pcd_path", default_value=default_visualization_map()),
        DeclareLaunchArgument("fastlio_config_file", default_value="mid360_localization.yaml"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=package_file("localization_adapter", "rviz", "icp_localization.rviz"),
        ),
        *fallback_message,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                package_file("livox_ros_driver2", "launch_ROS2", "msg_MID360_launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("start_livox_driver")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("fast_lio", "launch", "mapping.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "config_path": package_file("fast_lio", "config"),
                "config_file": LaunchConfiguration("fastlio_config_file"),
                "rviz": LaunchConfiguration("start_fastlio_rviz"),
                "lid_topic": "/livox/lidar",
                "imu_topic": "/livox/imu",
                "lidar_type": "1",
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_fastlio")),
        ),
        Node(
            package="localization_adapter",
            executable="icp_localization_node",
            name="icp_localization_node",
            output="screen",
            parameters=[
                package_file("localization_adapter", "config", "fastlio_mid360_icp_localization.yaml"),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "map_pcd_path": LaunchConfiguration("map_pcd_path"),
                    "icp_map_pcd_path": LaunchConfiguration("icp_map_pcd_path"),
                    "visualization_map_pcd_path": LaunchConfiguration("visualization_map_pcd_path"),
                    "odom_topic": "/Odometry",
                    "scan_topic": "/cloud_registered_body",
                },
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="baselink2footprint",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "-0.24",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "base_footprint",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="baselink2mid360",
            arguments=[
                "--x", "0.3", "--y", "0.0", "--z", "0.38",
                "--roll", "-0.034101", "--pitch", "0.566395", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "livox_frame",
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("start_rviz")),
        ),
    ])
