import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("mapping", default_value="false"),
        DeclareLaunchArgument("navigation", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument("global_planner_algorithm", default_value=""),
        DeclareLaunchArgument("require_fresh_cloud", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "mapping_impl.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                "start_fastlio": LaunchConfiguration("start_fastlio"),
                "start_rviz": LaunchConfiguration("rviz"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("mapping")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "navigation_impl.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "rviz": LaunchConfiguration("rviz"),
                "start_livox_driver": LaunchConfiguration("start_livox_driver"),
                "start_fastlio": LaunchConfiguration("start_fastlio"),
                "global_planner_algorithm": LaunchConfiguration("global_planner_algorithm"),
                "require_fresh_cloud": LaunchConfiguration("require_fresh_cloud"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("navigation")),
        ),
    ])
