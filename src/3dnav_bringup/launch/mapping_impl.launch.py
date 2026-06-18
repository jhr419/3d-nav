import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file, ws_path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument("start_mapping_adapter", default_value="true"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("mapping_dir", default_value=ws_path("maps", "fastlio_dddmr")),
        DeclareLaunchArgument("fastlio_config_file", default_value="mid360.yaml"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=package_file("mapping_adapter", "rviz", "livox_mid360_fastlio_dddmr_mapping.rviz"),
        ),
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
                "rviz": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("start_fastlio")),
        ),
        Node(
            package="mapping_adapter",
            executable="mapping_adapter_node",
            name="fastlio_dddmr_adapter",
            output="screen",
            parameters=[
                package_file("mapping_adapter", "config", "fastlio_mid360_dddmr.yaml"),
                {
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                    "mapping_dir": LaunchConfiguration("mapping_dir"),
                },
            ],
            condition=IfCondition(LaunchConfiguration("start_mapping_adapter")),
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
