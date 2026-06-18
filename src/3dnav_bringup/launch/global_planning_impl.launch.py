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
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("pct_global_planner", "launch", "pct_global_planner.launch.py")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "config_file": package_file("pct_global_planner", "config", "pct_global_planner.yaml"),
                "map_file": ws_path("maps", "map_preprocessed.pcd"),
                "pcd_file": ws_path("maps", "map_preprocessed.pcd"),
                "tomogram_file": "map_preprocessed",
                "tomogram_dir": ensure_dir("maps", "tomogram"),
                "planner_lib_dir": ws_path("src", "PCT_planner", "planner", "lib"),
                "launch_rviz": LaunchConfiguration("launch_rviz"),
            }.items(),
        ),
    ])
