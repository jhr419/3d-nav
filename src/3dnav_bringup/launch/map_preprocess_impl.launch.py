import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import ensure_dir, package_file, ws_path


def generate_launch_description():
    tomogram_dir = ensure_dir("maps", "tomogram")
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("preprocess_overwrite", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("pct_planner_ros2", "launch", "tomography.launch.py")),
            launch_arguments={
                "tomography_config": package_file("pct_planner_ros2", "config", "tomography.yaml"),
                "pcd_dir": ws_path("maps"),
                "pcd_file": "map_preprocessed.pcd",
                "preprocess_input_pcd": ws_path("maps", "map_origin.pcd"),
                "preprocess_output_pcd": ws_path("maps", "map_preprocessed.pcd"),
                "tomogram_dir": tomogram_dir,
                "rviz": LaunchConfiguration("rviz"),
                "preprocess_overwrite": LaunchConfiguration("preprocess_overwrite"),
            }.items(),
        ),
    ])
