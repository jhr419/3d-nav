import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_control", default_value="true"),
        DeclareLaunchArgument("launch_twist_bridge", default_value="true"),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(package_file("nav3d_control", "launch", "control.launch")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "config_file": package_file("nav3d_control", "config", "nav_execution_controller.yaml"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_control")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("go2_twist_bridge", "launch", "twist_bridge.launch.py")),
            launch_arguments={
                "config_file": package_file("go2_twist_bridge", "config", "twist_bridge.yaml"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("launch_twist_bridge")),
        ),
    ])
