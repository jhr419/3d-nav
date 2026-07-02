import sys
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription

sys.path.insert(0, str(Path(__file__).resolve().parent))
from _utils import package_file, ws_path


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
        },
    },
    "jie_octomap": {
        "package": "octo_planner",
        "launch": "jie_3d_global_planner.launch.py",
        "config": "src/global_planning/jie_3d_nav/octo_planner/config/jie_3d_global_planner.yaml",
    },
    "astar": {
        "package": "nav3d_global_planning",
        "launch": "astar_global_planner.launch.py",
        "config": "src/global_planning/3dnav_global_planning/config/astar_global_planner.yaml",
        "launch_arguments": {
            "pcd_file": "maps/map_preprocessed.pcd",
            "tomogram_file": "map_preprocessed",
            "tomogram_dir": "maps/tomogram",
        },
    },
}


def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


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

    launch_arguments = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "config_file": config_file,
        "launch_rviz": LaunchConfiguration("launch_rviz"),
    }
    for key, value in dict(selected.get("launch_arguments", {}) or {}).items():
        launch_arguments[key] = _resolve_launch_argument(key, value)

    return [
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
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("algorithm", default_value=""),
            DeclareLaunchArgument(
                "system_config",
                default_value=package_file("nav3d_bringup", "config", "nav3d_system.yaml"),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
