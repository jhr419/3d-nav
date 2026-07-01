from dataclasses import dataclass
from pathlib import Path


def default_data_dir(name):
    return str(Path.home() / '.ros' / 'pct_planner' / name)


@dataclass
class ROSConfig:
    map_frame: str = 'map'
    pointcloud_topic: str = '/global_points'
    layer_g_topic_prefix: str = '/layer_G_'
    layer_c_topic_prefix: str = '/layer_C_'
    tomogram_topic: str = '/tomogram'
    path_topic: str = '/pct_path'


@dataclass
class TomographyConfig:
    pcd_dir: str = default_data_dir('pcd')
    tomogram_dir: str = default_data_dir('tomogram')
    benchmark_repeats: int = 10


@dataclass
class PlannerConfig:
    tomogram_dir: str = default_data_dir('tomogram')
    planner_lib_dir: str = ''
    use_quintic: bool = True
    max_heading_rate: float = 6.0
    path_z_offset: float = 0.0
    astar_cost_threshold: float = 20.0
    astar_step_cost_weight: float = 0.5
    optimizer_safe_cost_threshold: float = 8.0
    max_path_z_jump: float = 0.8
