import os
import signal
import sys
import time
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file, ws_path


GOAL_MARKER_EXECUTABLE = "pct_goal_marker_node"
GOAL_MARKER_NODE_REMAP = "__node:=pct_start_goal_marker"


DEFAULT_ALGORITHMS = {
    "pct": {
        "package": "pct_global_planner",
        "launch": "pct_global_planner.launch.py",
        "config": "src/global_planning/pct_global_planner/config/pct_global_planner.yaml",
        "launch_arguments": {
            "map_file": "maps/map_preprocessed.pcd",
            "pcd_file": "maps/map_preprocessed.pcd",
            "tomogram_file": "map_preprocessed",
            "tomogram_dir": "maps/tomogram",
            "planner_lib_dir": "src/map_process/PCT_planner/planner/lib",
            "rviz_config": "src/global_planning/pct_global_planner/rviz/pct_global_planner.rviz",
        },
    },
    "jie_octomap": {
        "package": "octo_planner",
        "launch": "jie_3d_global_planner.launch.py",
        "config": "src/global_planning/jie_3d_nav/octo_planner/config/jie_3d_global_planner.yaml",
        "launch_arguments": {
            "rviz_config": "src/global_planning/jie_3d_nav/jie_octomap/rviz/octomap_test.rviz",
        },
    },
    "astar": {
        "package": "nav3d_global_planning",
        "launch": "astar_global_planner.launch.py",
        "config": "src/global_planning/3dnav_global_planning/config/astar_global_planner.yaml",
        "launch_arguments": {
            "pcd_file": "maps/map_preprocessed.pcd",
            "tomogram_file": "map_preprocessed",
            "tomogram_dir": "maps/tomogram",
            "rviz_config": "src/global_planning/pct_global_planner/rviz/pct_global_planner.rviz",
        },
    },
}


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _matching_goal_marker_pids() -> list[int]:
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
        if GOAL_MARKER_EXECUTABLE in cmdline and GOAL_MARKER_NODE_REMAP in cmdline:
            pids.append(pid)
    return pids


def _cleanup_stale_goal_marker_processes() -> list[int]:
    pids = _matching_goal_marker_pids()
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


def _normalize_algorithm(raw: str) -> str:
    value = str(raw or "").strip()
    if not value:
        return "pct"
    lowered = value.lower().replace(" ", "_").replace("-", "_")
    aliases = {
        "pct_planner": "pct",
        "pct": "pct",
        "jie_3d_nav": "jie_octomap",
        "jie_octomap": "jie_octomap",
        "octomap": "jie_octomap",
        "octomap_planner": "jie_octomap",
        "a_star": "astar",
        "astar": "astar",
        "astar_global_planner": "astar",
    }
    return aliases.get(lowered, lowered)


def _resolve_workspace_file(path: str) -> str:
    value = str(path or "").strip()
    if not value:
        return value
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return str(candidate)
    return ws_path(*candidate.parts)


def _looks_like_workspace_path(key: str, value: str) -> bool:
    if not isinstance(value, str):
        return False
    if key == "tomogram_file":
        return False
    if key == "rviz_config":
        return "/" in value or value.startswith(".") or value.startswith("~")
    lowered = key.lower()
    if not any(token in lowered for token in ("file", "dir", "path")):
        return False
    return "/" in value or value.startswith(".") or value.startswith("~")


def _resolve_launch_argument(key: str, value):
    if value == "{use_sim_time}":
        return LaunchConfiguration("use_sim_time")
    if value == "{launch_rviz}":
        return LaunchConfiguration("launch_rviz")
    if isinstance(value, bool):
        return "true" if value else "false"
    if _looks_like_workspace_path(key, value):
        return _resolve_workspace_file(value)
    return str(value)


def _global_planning_config(system_config: dict) -> dict:
    if "global_planning" in system_config:
        return dict(system_config.get("global_planning") or {})

    legacy = dict(system_config.get("modules", {}).get("global_planning", {}) or {})
    if not legacy:
        return {"algorithm": "pct", "algorithms": DEFAULT_ALGORITHMS}

    package = legacy.get("package", "pct_global_planner")
    launch = legacy.get("launch", "pct_global_planner.launch.py")
    return {
        "enabled": True,
        "algorithm": _normalize_algorithm(legacy.get("algorithm", "pct")),
        "algorithms": {
            "pct": {
                **DEFAULT_ALGORITHMS["pct"],
                "package": package,
                "launch": launch,
            }
        },
    }


def _launch_setup(context, *args, **kwargs):
    system_config_path = LaunchConfiguration("system_config").perform(context)
    system_config = _load_yaml(system_config_path)
    global_config = _global_planning_config(system_config)

    requested_algorithm = LaunchConfiguration("algorithm").perform(context)
    algorithm = _normalize_algorithm(requested_algorithm or global_config.get("algorithm", "pct"))

    algorithms = dict(DEFAULT_ALGORITHMS)
    algorithms.update(global_config.get("algorithms", {}) or {})
    if algorithm not in algorithms:
        known = ", ".join(sorted(algorithms.keys()))
        raise RuntimeError(
            f"Unknown global planner algorithm '{algorithm}'. Known algorithms: {known}"
        )

    selected = dict(algorithms[algorithm])
    package = selected["package"]
    launch_file = selected["launch"]
    config_file = _resolve_workspace_file(selected["config"])
    cleanup_goal_marker = _as_bool(
        LaunchConfiguration("cleanup_stale_goal_marker").perform(context)
    )

    launch_arguments = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "config_file": config_file,
        "launch_rviz": LaunchConfiguration("launch_rviz"),
    }
    for key, value in dict(selected.get("launch_arguments", {}) or {}).items():
        launch_arguments[key] = _resolve_launch_argument(key, value)

    actions = []
    if cleanup_goal_marker and algorithm in ("astar", "pct"):
        cleaned_pids = _cleanup_stale_goal_marker_processes()
        if cleaned_pids:
            actions.append(
                LogInfo(
                    msg=(
                        "[3dnav_bringup] Cleaned stale pct_start_goal_marker "
                        f"processes before launch: {cleaned_pids}"
                    )
                )
            )

    if _as_bool(LaunchConfiguration("offline_test").perform(context)):
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="offline_map_to_base_link",
                arguments=[
                    "--x", LaunchConfiguration("offline_start_x"),
                    "--y", LaunchConfiguration("offline_start_y"),
                    "--z", LaunchConfiguration("offline_start_z"),
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", LaunchConfiguration("offline_start_yaw"),
                    "--frame-id", "map",
                    "--child-frame-id", "base_link",
                ],
            )
        )

    actions.extend([
        LogInfo(msg=f"[3dnav_bringup] Global planner algorithm: {algorithm}"),
        LogInfo(
            msg=(
                f"[3dnav_bringup] Including {package}/launch/{launch_file} "
                f"with config {config_file}"
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(package_file(package, "launch", launch_file)),
            launch_arguments=launch_arguments.items(),
        ),
    ])
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("algorithm", default_value=""),
            DeclareLaunchArgument("offline_test", default_value="false"),
            DeclareLaunchArgument(
                "cleanup_stale_goal_marker",
                default_value="true",
                description=(
                    "Terminate stale pct_start_goal_marker processes before "
                    "starting a new PCT/A* goal marker server."
                ),
            ),
            DeclareLaunchArgument("offline_start_x", default_value="-1.5"),
            DeclareLaunchArgument("offline_start_y", default_value="3.2"),
            DeclareLaunchArgument("offline_start_z", default_value="0.0"),
            DeclareLaunchArgument("offline_start_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "system_config",
                default_value=package_file("nav3d_bringup", "config", "nav3d_system.yaml"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
