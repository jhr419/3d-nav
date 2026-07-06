import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    offline_test = _as_bool(LaunchConfiguration("offline_test").perform(context))

    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    mapping = "false" if offline_test else arg("mapping")
    start_livox_driver = "false" if offline_test else arg("start_livox_driver")
    start_fastlio = "false" if offline_test else arg("start_fastlio")
    require_fresh_cloud = "false" if offline_test else arg("require_fresh_cloud")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "mapping_impl.launch.py")),
            launch_arguments={
                "use_sim_time": arg("use_sim_time"),
                "start_livox_driver": start_livox_driver,
                "start_fastlio": start_fastlio,
                "start_rviz": arg("rviz"),
            }.items(),
            condition=IfCondition(mapping),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "navigation_impl.launch.py")),
            launch_arguments={
                "use_sim_time": arg("use_sim_time"),
                "rviz": arg("rviz"),
                "start_livox_driver": start_livox_driver,
                "start_fastlio": start_fastlio,
                "global_planner_algorithm": arg("global_planner_algorithm"),
                "require_fresh_cloud": require_fresh_cloud,
                "diagnostics": arg("diagnostics"),
                "offline_test": arg("offline_test"),
                "offline_start_x": arg("offline_start_x"),
                "offline_start_y": arg("offline_start_y"),
                "offline_start_z": arg("offline_start_z"),
                "offline_start_yaw": arg("offline_start_yaw"),
            }.items(),
            condition=IfCondition(arg("navigation")),
        ),
    ]


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
        DeclareLaunchArgument("diagnostics", default_value="true"),
        DeclareLaunchArgument("offline_test", default_value="false"),
        DeclareLaunchArgument("offline_start_x", default_value="-1.5"),
        DeclareLaunchArgument("offline_start_y", default_value="3.2"),
        DeclareLaunchArgument("offline_start_z", default_value="0.0"),
        DeclareLaunchArgument("offline_start_yaw", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
