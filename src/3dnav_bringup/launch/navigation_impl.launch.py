import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("localization", default_value="true"),
        DeclareLaunchArgument("global_planning", default_value="true"),
        DeclareLaunchArgument("local_planning", default_value="true"),
        DeclareLaunchArgument("robot_api", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument("global_planner_algorithm", default_value=""),
        DeclareLaunchArgument("require_fresh_cloud", default_value="true"),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=package_file("nav3d_bringup", "rviz", "nav3d_debug.rviz"),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "localization_impl.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                "start_fastlio": LaunchConfiguration("start_fastlio"),
                "start_rviz": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("localization")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "global_planning_impl.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "launch_rviz": "false",
                "algorithm": LaunchConfiguration("global_planner_algorithm"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("global_planning")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "local_planning_impl.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "require_fresh_cloud": LaunchConfiguration("require_fresh_cloud"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("local_planning")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "robot_api_impl.launch.py")),
            launch_arguments={"use_sim_time": LaunchConfiguration("use_sim_time")}.items(),
            condition=IfCondition(LaunchConfiguration("robot_api")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
