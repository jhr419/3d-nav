from copy import deepcopy
from dataclasses import dataclass

import numpy as np


@dataclass
class ScenePCD:
    file_name: str


@dataclass
class SceneMap:
    resolution: float = 0.10
    ground_h: float = 0.0
    slice_dh: float = 0.5


@dataclass
class SceneTrav:
    kernel_size: int = 7
    interval_min: float = 0.50
    interval_free: float = 0.65
    slope_max: float = 0.36
    step_max: float = 0.20
    standable_ratio: float = 0.20
    cost_barrier: float = 50.0
    safe_margin: float = 0.4
    inflation: float = 0.2


@dataclass
class SceneConfig:
    pcd: ScenePCD
    map: SceneMap
    trav: SceneTrav


@dataclass
class PlanSceneDefaults:
    tomogram_file: str
    start_pos: np.ndarray
    goal_pos: np.ndarray


SCENES = {
    'Spiral': SceneConfig(
        pcd=ScenePCD('spiral0.3_2.pcd'),
        map=SceneMap(resolution=0.20, ground_h=0.0, slice_dh=0.5),
        trav=SceneTrav(
            kernel_size=7,
            interval_min=0.50,
            interval_free=0.65,
            slope_max=0.40,
            step_max=0.30,
            standable_ratio=0.40,
            cost_barrier=50.0,
            safe_margin=1.2,
            inflation=0.2,
        ),
    ),
    'Building': SceneConfig(
        pcd=ScenePCD('building2_9.pcd'),
        map=SceneMap(resolution=0.10, ground_h=0.0, slice_dh=0.5),
        trav=SceneTrav(
            kernel_size=7,
            interval_min=0.50,
            interval_free=0.65,
            slope_max=0.40,
            step_max=0.17,
            standable_ratio=0.20,
            cost_barrier=50.0,
            safe_margin=0.4,
            inflation=0.2,
        ),
    ),
    'Plaza': SceneConfig(
        pcd=ScenePCD('plaza3_10.pcd'),
        map=SceneMap(resolution=0.10, ground_h=0.0, slice_dh=0.5),
        trav=SceneTrav(
            kernel_size=7,
            interval_min=0.50,
            interval_free=0.65,
            slope_max=0.36,
            step_max=0.17,
            standable_ratio=0.20,
            cost_barrier=50.0,
            safe_margin=0.4,
            inflation=0.2,
        ),
    ),
}


PLAN_DEFAULTS = {
    'Spiral': PlanSceneDefaults(
        tomogram_file='spiral0.3_2',
        start_pos=np.array([-16.0, -6.0], dtype=np.float32),
        goal_pos=np.array([-26.0, -5.0], dtype=np.float32),
    ),
    'Building': PlanSceneDefaults(
        tomogram_file='building2_9',
        start_pos=np.array([5.0, 5.0], dtype=np.float32),
        goal_pos=np.array([-6.0, -1.0], dtype=np.float32),
    ),
    'Plaza': PlanSceneDefaults(
        tomogram_file='plaza3_10',
        start_pos=np.array([0.0, 0.0], dtype=np.float32),
        goal_pos=np.array([23.0, 10.0], dtype=np.float32),
    ),
}


def get_scene_config(scene_name):
    try:
        return deepcopy(SCENES[scene_name])
    except KeyError as exc:
        names = ', '.join(sorted(SCENES))
        raise ValueError(f'Unknown scene "{scene_name}". Available scenes: {names}') from exc


def get_plan_defaults(scene_name):
    try:
        return deepcopy(PLAN_DEFAULTS[scene_name])
    except KeyError as exc:
        names = ', '.join(sorted(PLAN_DEFAULTS))
        raise ValueError(f'Unknown scene "{scene_name}". Available scenes: {names}') from exc
