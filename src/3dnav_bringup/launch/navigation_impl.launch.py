import os
import signal
import sys
import time
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file


STALE_NAVIGATION_PATTERNS = (
    ("ego_local_planner_node", "__node:=ego_local_planner_node"),
    ("astar_global_planner_node", "__node:=astar_global_planner_node"),
    ("pct_global_map_publisher_node", "__node:=pct_global_map_publisher"),
    ("pct_goal_marker_node", "__node:=pct_start_goal_marker"),
)

STALE_RVIZ_PATTERNS = (
    ("rviz2", "__node:=planning_rviz2"),
    ("rviz2", "__node:=localization_rviz2"),
)


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _is_stale_offline_tf(cmdline: str) -> bool:
    if "static_transform_publisher" not in cmdline:
        return False
    if "__node:=offline_map_to_base_link" in cmdline:
        return True
    compact = " ".join(part for part in cmdline.split("\0") if part)
    return (
        " map base_link" in compact
        or "--frame-id map --child-frame-id base_link" in compact
    )


def _matching_stale_pids(include_offline_tf: bool, include_rviz: bool) -> list[int]:
    current_pid = os.getpid()
    pids = []
    proc_root = Path("/proc")
    if not proc_root.exists():
        return pids

    for proc_dir in proc_root.iterdir():
        if not proc_dir.name.isdigit():
            continue
        pid = int(proc_dir.name)
        if pid == current_pid:
            continue
        try:
            cmdline = proc_dir.joinpath("cmdline").read_bytes().decode(
                "utf-8",
                errors="ignore",
            )
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue

        matched = any(
            executable in cmdline and node_remap in cmdline
            for executable, node_remap in STALE_NAVIGATION_PATTERNS
        )
        if include_rviz:
            matched = matched or any(
                executable in cmdline and node_remap in cmdline
                for executable, node_remap in STALE_RVIZ_PATTERNS
            )
        if include_offline_tf:
            matched = matched or _is_stale_offline_tf(cmdline)
        if matched:
            pids.append(pid)
    return pids


def _cleanup_stale_navigation_processes(include_offline_tf: bool, include_rviz: bool) -> list[int]:
    pids = _matching_stale_pids(include_offline_tf, include_rviz)
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass

    if pids:
        time.sleep(0.3)

    for pid in pids:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            continue
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    return pids


def _launch_setup(context, *args, **kwargs):
    offline_test = _as_bool(LaunchConfiguration("offline_test").perform(context))
    cleanup_stale_nodes = _as_bool(
        LaunchConfiguration("cleanup_stale_navigation_nodes").perform(context)
    )

    def arg(name: str) -> str:
        return LaunchConfiguration(name).perform(context)

    localization = "false" if offline_test else arg("localization")
    robot_api = "false" if offline_test else arg("robot_api")
    start_livox_driver = "false" if offline_test else arg("start_livox_driver")
    start_fastlio = "false" if offline_test else arg("start_fastlio")
    require_fresh_cloud = "false" if offline_test else arg("require_fresh_cloud")

    actions = []
    if cleanup_stale_nodes:
        cleaned_pids = _cleanup_stale_navigation_processes(
            include_offline_tf=offline_test,
            include_rviz=_as_bool(arg("rviz")),
        )
        if cleaned_pids:
            actions.append(
                LogInfo(
                    msg=(
                        "[3dnav_bringup] Cleaned stale navigation/planning "
                        f"processes before launch: {cleaned_pids}"
                    )
                )
            )

    if offline_test:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="offline_map_to_base_link",
                arguments=[
                    "--x", arg("offline_start_x"),
                    "--y", arg("offline_start_y"),
                    "--z", arg("offline_start_z"),
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", arg("offline_start_yaw"),
                    "--frame-id", "map",
                    "--child-frame-id", "base_link",
                ],
            )
        )

    actions.extend([
        LogInfo(
            msg="[3dnav_bringup] diagnostics enabled",
            condition=IfCondition(arg("diagnostics")),
        ),
        LogInfo(
            msg="[3dnav_bringup] local planner performance monitor enabled",
            condition=IfCondition(arg("local_planning")),
        ),
        Node(
            package="nav3d_diagnostics",
            executable="nav3d_system_diagnostics_node",
            name="nav3d_system_diagnostics_node",
            output="screen",
            parameters=[
                {
                    "config_file": package_file(
                        "nav3d_bringup", "config", "nav3d_diagnostics.yaml"
                    ),
                    "use_sim_time": _as_bool(arg("use_sim_time")),
                }
            ],
            condition=IfCondition(arg("diagnostics")),
        ),
    ])

    actions.extend([
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
                "offline_test": "false",
            }.items(),
            condition=IfCondition(arg("global_planning")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "local_planning_impl.launch.py")),
            launch_arguments={
                "use_sim_time": arg("use_sim_time"),
                "config_file": package_file("ego_local_planner", "config", "ego_local_planner.yaml"),
                "require_fresh_cloud": require_fresh_cloud,
            }.items(),
            condition=IfCondition(arg("local_planning")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file("nav3d_bringup", "launch", "robot_api_impl.launch.py")),
            launch_arguments={"use_sim_time": arg("use_sim_time")}.items(),
            condition=IfCondition(robot_api),
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("localization", default_value="true"),
        DeclareLaunchArgument("global_planning", default_value="true"),
        DeclareLaunchArgument("local_planning", default_value="true"),
        DeclareLaunchArgument("robot_api", default_value="true"),
        DeclareLaunchArgument("diagnostics", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("start_livox_driver", default_value="true"),
        DeclareLaunchArgument("start_fastlio", default_value="true"),
        DeclareLaunchArgument("global_planner_algorithm", default_value=""),
        DeclareLaunchArgument("require_fresh_cloud", default_value="true"),
        DeclareLaunchArgument("offline_test", default_value="false"),
        DeclareLaunchArgument("cleanup_stale_navigation_nodes", default_value="true"),
        DeclareLaunchArgument("offline_start_x", default_value="-1.5"),
        DeclareLaunchArgument("offline_start_y", default_value="3.2"),
        DeclareLaunchArgument("offline_start_z", default_value="0.0"),
        DeclareLaunchArgument("offline_start_yaw", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
