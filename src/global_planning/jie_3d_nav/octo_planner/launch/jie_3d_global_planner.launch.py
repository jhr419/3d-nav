import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _valid_workspace(path: Path) -> bool:
    return (path / "src").is_dir() and (path / "maps").is_dir()


def _find_workspace_root() -> Path:
    env_value = os.environ.get("NAV3D_WS", "")
    if env_value:
        env_path = Path(env_value).expanduser()
        if _valid_workspace(env_path):
            return env_path

    cwd = Path.cwd()
    for candidate in (cwd, *cwd.parents):
        if _valid_workspace(candidate):
            return candidate

    return cwd


def _resolve_workspace_path(raw_path: str) -> str:
    path = Path(os.path.expanduser(str(raw_path).strip()))
    if not str(path):
        return ""
    if path.is_absolute():
        return str(path)
    return str((_find_workspace_root() / path).resolve())


def _load_params(config_file: str, node_name: str) -> dict:
    with open(config_file, "r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    return dict(data.get(node_name, {}).get("ros__parameters", {}))


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    adapter_params = _load_params(config_file, "jie_3d_global_planner_adapter")
    pcd_params = _load_params(config_file, "jie_3d_pcd_to_octomap")

    map_frame = str(adapter_params.get("map_frame", "map"))
    octomap_topic = str(adapter_params.get("octomap_topic", "/octomap"))
    octomap_file = _resolve_workspace_path(
        adapter_params.get("octomap_file", "maps/map_preprocessed.bt")
    )
    pcd_file = _resolve_workspace_path(
        adapter_params.get(
            "map_pcd_file",
            pcd_params.get("pcd_file", "maps/map_preprocessed.pcd"),
        )
    )
    resolution = float(adapter_params.get("resolution", pcd_params.get("resolution", 0.2)))

    actions = [
        LogInfo(
            msg=[
                "[octo_planner] jie_3d_nav global planner config: ",
                config_file,
            ]
        ),
        LogInfo(msg=["[octo_planner] OctoMap topic: ", octomap_topic]),
    ]

    if os.path.isfile(octomap_file):
        actions.append(LogInfo(msg=["[octo_planner] Using OctoMap file: ", octomap_file]))
    elif os.path.isfile(pcd_file):
        actions.extend(
            [
                LogInfo(
                    msg=[
                        "[octo_planner][WARN] OctoMap file is missing; building OctoMap from PCD: ",
                        pcd_file,
                    ]
                ),
                Node(
                    package="jie_octomap",
                    executable="pcd_to_octomap_node",
                    name="jie_3d_pcd_to_octomap",
                    output="screen",
                    parameters=[
                        config_file,
                        {
                            "use_sim_time": ParameterValue(
                                LaunchConfiguration("use_sim_time"), value_type=bool
                            ),
                            "pcd_file": pcd_file,
                            "octomap_topic": octomap_topic,
                            "frame_id": map_frame,
                            "resolution": resolution,
                        },
                    ],
                ),
            ]
        )
    else:
        actions.append(
            LogInfo(
                msg=[
                    "[octo_planner][ERROR] No map found. Expected ",
                    octomap_file,
                    " or ",
                    pcd_file,
                ]
            )
        )

    actions.extend(
        [
            Node(
                package="octo_planner",
                executable="jie_path_node",
                name="jie_path_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "octomap_topic": octomap_topic,
                        "frame_id": map_frame,
                    },
                ],
            ),
            Node(
                package="octo_planner",
                executable="jie_3d_global_planner_adapter_node",
                name="jie_3d_global_planner_adapter",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "octomap_file": octomap_file,
                        "map_pcd_file": pcd_file,
                        "octomap_topic": octomap_topic,
                        "map_frame": map_frame,
                        "resolution": resolution,
                    },
                ],
            ),
            Node(
                package="jie_octomap",
                executable="octomap_to_occupied_markers_node",
                name="jie_octomap_occupied_markers",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "octomap_topic": octomap_topic,
                        "frame_id": map_frame,
                    },
                ],
                condition=IfCondition(LaunchConfiguration("launch_map_markers")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="planning_rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
            ),
        ]
    )

    return actions


def generate_launch_description():
    planner_share = get_package_share_directory("octo_planner")
    jie_octomap_share = get_package_share_directory("jie_octomap")
    default_config_file = os.path.join(
        planner_share,
        "config",
        "jie_3d_global_planner.yaml",
    )
    default_rviz_config_file = os.path.join(
        jie_octomap_share,
        "rviz",
        "octomap_test.rviz",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file,
                description="YAML parameter file for jie_3d_nav global planner adapter",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Launch RViz for OctoMap and path visualization",
            ),
            DeclareLaunchArgument(
                "launch_map_markers",
                default_value="true",
                description="Publish OctoMap occupied-cell visualization markers",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config_file,
                description="RViz config file",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
