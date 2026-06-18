import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import ensure_dir, package_file, ws_path


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("pct_planner_ros2", "launch", "planner.launch.py")),
            launch_arguments={
                "planner_config": package_file("pct_planner_ros2", "config", "planner.yaml"),
                "pcd_dir": ws_path("maps"),
                "pcd_file": "map_preprocessed.pcd",
                "tomogram_dir": ensure_dir("maps", "tomogram"),
                "tomogram_file": "map_preprocessed",
                "planner_lib_dir": ws_path("src", "PCT_planner", "planner", "lib"),
                "show_rviz": LaunchConfiguration("rviz"),
            }.items(),
        ),
    ])
