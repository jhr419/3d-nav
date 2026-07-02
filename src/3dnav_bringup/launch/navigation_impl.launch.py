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

    localization = "false" if offline_test else arg("localization")
    robot_api = "false" if offline_test else arg("robot_api")
    start_livox_driver = "false" if offline_test else arg("start_livox_driver")
    start_fastlio = "false" if offline_test else arg("start_fastlio")
    require_fresh_cloud = "false" if offline_test else arg("require_fresh_cloud")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "localization_impl.launch.py")),
            launch_arguments={
                "use_sim_time": arg("use_sim_time"),
                "start_livox_driver": start_livox_driver,
                "start_fastlio": start_fastlio,
                "start_rviz": arg("rviz"),
            }.items(),
            condition=IfCondition(localization),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "global_planning_impl.launch.py")),
            launch_arguments={
                "use_sim_time": arg("use_sim_time"),
                "launch_rviz": arg("rviz"),
                "algorithm": arg("global_planner_algorithm"),
            }.items(),
            condition=IfCondition(arg("global_planning")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "local_planning_impl.launch.py")),
            launch_arguments={
                "use_sim_time": arg("use_sim_time"),
                "require_fresh_cloud": require_fresh_cloud,
            }.items(),
            condition=IfCondition(arg("local_planning")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "robot_api_impl.launch.py")),
            launch_arguments={"use_sim_time": arg("use_sim_time")}.items(),
            condition=IfCondition(robot_api),
        ),
    ]


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
        DeclareLaunchArgument("offline_test", default_value="false"),
        OpaqueFunction(function=_launch_setup),
    ])
