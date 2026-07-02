import os
import sys
import math
from pathlib import Path

import numpy as np


Point3 = tuple[float, float, float]


class PCTPlannerAdapter:
    def __init__(self):
        self.node = None
        self.logger = None
        self.planner = None
        self.initialized = False
        self.last_error = ""
        self.tomogram_file = ""
        self.tomogram_dir = ""
        self.min_direct_path_distance = 0.35

    def initialize(self, node) -> bool:
        self.node = node
        self.logger = node.get_logger()
        self.initialized = False
        self.last_error = ""

        try:
            self._prepare_import_path()
            from pct_planner_ros2.config import PlannerConfig
            from pct_planner_ros2.planner_wrapper import TomogramPlanner
            from pct_planner_ros2.scenes import get_plan_defaults

            scene = self._param("scene", "Building")
            tomogram_file = str(self._param("tomogram_file", "")).strip()
            tomogram_dir = str(self._param("tomogram_dir", str(Path.home() / ".ros" / "pct_planner" / "tomogram"))).strip()

            if not tomogram_file:
                tomogram_file = get_plan_defaults(scene).tomogram_file

            tomogram_dir, tomogram_file = self._normalize_tomogram_path(tomogram_dir, tomogram_file)
            tomogram_path = Path(tomogram_dir) / f"{tomogram_file}.pickle"
            if not tomogram_path.is_file():
                self.last_error = f"PCT tomogram file not found: {tomogram_path}"
                self.logger.error(self.last_error)
                return False

            map_source = str(self._param("map_source", "tomogram")).strip().lower()
            if map_source not in ("tomogram", "pct_tomogram"):
                self.logger.warning(
                    "PCT Planner ROS2 wrapper plans from tomogram pickle files; "
                    "map_source='%s' is kept as metadata and tomogram_file='%s' is used."
                    % (map_source, tomogram_path)
                )

            planner_cfg = PlannerConfig(
                tomogram_dir=tomogram_dir,
                planner_lib_dir=str(self._param("planner_lib_dir", "")),
                use_quintic=self._as_bool(self._param("use_quintic", True)),
                max_heading_rate=float(self._param("max_heading_rate", 6.0)),
                path_z_offset=float(self._param("path_z_offset", 0.0)),
                astar_cost_threshold=float(self._param("astar_cost_threshold", 20.0)),
                astar_step_cost_weight=float(self._param("astar_step_cost_weight", 0.5)),
                optimizer_safe_cost_threshold=float(self._param("optimizer_safe_cost_threshold", 8.0)),
                max_path_z_jump=float(self._param("max_path_z_jump", 0.8)),
                optimized_path_collision_cost_threshold=float(
                    self._param("optimized_path_collision_cost_threshold", 20.0)
                ),
                optimized_path_collision_check_resolution=float(
                    self._param("optimized_path_collision_check_resolution", 0.10)
                ),
            )

            self.planner = TomogramPlanner(planner_cfg, logger=self.logger)
            self.logger.info(f"Loading PCT tomogram: {tomogram_path}")
            self.planner.load_tomogram(tomogram_file)
            self._apply_clearance_cost_layer_to_pct_search()
            self.min_direct_path_distance = float(self._param("min_direct_path_distance", 0.35))

            self.tomogram_file = tomogram_file
            self.tomogram_dir = tomogram_dir
            self.initialized = True
            self.logger.info("PCT Planner adapter initialized.")
            return True
        except Exception as exc:  # noqa: BLE001 - ROS node reports the exact initialization failure.
            self.last_error = f"Failed to initialize PCT Planner adapter: {exc}"
            self.logger.error(self.last_error)
            return False

    def _apply_clearance_cost_layer_to_pct_search(self) -> None:
        if not self._as_bool(self._param("global_search_clearance_enabled", True)):
            return
        if self.planner is None or not hasattr(self.planner, "apply_traversability_cost_overlay"):
            return

        safety_map = getattr(self.node, "safety_map", None)
        if safety_map is None or not getattr(safety_map, "loaded", False):
            self.logger.warning("PCT A* clearance layer disabled because safety_map is not loaded.")
            return
        if getattr(safety_map, "obstacle_points", np.empty((0, 3))).size == 0:
            self.logger.warning("PCT A* clearance layer disabled because no obstacle points are available.")
            return

        overlay = self._build_clearance_overlay(safety_map)
        if overlay is None:
            return

        max_total_cost = float(self._param("global_search_clearance_max_total_cost", 19.5))
        self.planner.apply_traversability_cost_overlay(
            overlay,
            max_total_cost=max_total_cost,
            preserve_untraversable=True,
            recompute_gradients=True,
        )

        finite = overlay[np.isfinite(overlay)]
        active = finite[finite > 1.0e-6]
        if active.size == 0:
            self.logger.info("PCT A* clearance layer built but has no active cells.")
            return
        self.logger.info(
            "Applied PCT A* clearance layer: active_cells=%d mean_extra=%.3f max_extra=%.3f max_total_cost=%.3f"
            % (active.size, float(np.mean(active)), float(np.max(active)), max_total_cost)
        )

    def _build_clearance_overlay(self, safety_map):
        resolution = float(getattr(self.planner, "resolution", 0.0) or 0.0)
        center = np.asarray(getattr(self.planner, "center", [0.0, 0.0]), dtype=np.float64)
        map_dim = list(getattr(self.planner, "map_dim", []) or [])
        if resolution <= 0.0 or len(map_dim) != 2:
            self.logger.warning("Cannot build PCT A* clearance layer: invalid tomogram geometry.")
            return None

        rows = np.arange(map_dim[0], dtype=np.float64)
        cols = np.arange(map_dim[1], dtype=np.float64)
        xs = (rows - 0.5 * map_dim[0]) * resolution + center[0]
        ys = (cols - 0.5 * map_dim[1]) * resolution + center[1]
        xx, yy = np.meshgrid(xs, ys, indexing="ij")
        query = np.column_stack((xx.reshape(-1), yy.reshape(-1)))

        clearance = self._query_clearance_batch(safety_map, query).reshape(map_dim[0], map_dim[1])
        preferred = float(self._param("global_search_preferred_clearance", self._param("preferred_clearance", 0.50)))
        hard = float(self._param("global_search_hard_min_clearance", self._param("hard_min_clearance", 0.25)))
        hard = min(hard, preferred - 1.0e-3)
        span = max(1.0e-3, preferred - hard)

        ratio = np.clip((preferred - clearance) / span, 0.0, 1.0)
        weight = float(self._param("global_search_clearance_weight", 45.0))
        power = float(self._param("global_search_clearance_power", 2.0))
        max_extra = float(self._param("global_search_clearance_max_extra_cost", 18.0))
        overlay_2d = weight * np.power(ratio, power)
        if max_extra > 0.0:
            overlay_2d = np.minimum(overlay_2d, max_extra)
        overlay_2d[~np.isfinite(overlay_2d)] = 0.0

        # Do not spend search effort shaping cells that PCT already treats as obstacles.
        trav = np.asarray(getattr(self.planner, "trav", np.empty((0,))), dtype=np.float32)
        if trav.ndim == 3 and trav.shape[1:] == overlay_2d.shape:
            traversable_any_layer = np.any(trav <= float(self._param("astar_cost_threshold", 20.0)), axis=0)
            overlay_2d = np.where(traversable_any_layer, overlay_2d, 0.0)

        return overlay_2d.astype(np.float32, copy=False)

    @staticmethod
    def _query_clearance_batch(safety_map, query: np.ndarray) -> np.ndarray:
        tree = getattr(safety_map, "obstacle_tree", None)
        if tree is not None:
            distances, _ = tree.query(query, k=1)
            return np.asarray(distances, dtype=np.float32)

        points = np.asarray(getattr(safety_map, "obstacle_points", np.empty((0, 3))), dtype=np.float32)
        if points.size == 0:
            return np.full(query.shape[0], np.inf, dtype=np.float32)

        out = np.empty(query.shape[0], dtype=np.float32)
        chunk_size = 4096
        obstacles_xy = points[:, :2].astype(np.float32, copy=False)
        for start in range(0, query.shape[0], chunk_size):
            stop = min(query.shape[0], start + chunk_size)
            diff = query[start:stop, np.newaxis, :].astype(np.float32) - obstacles_xy[np.newaxis, :, :]
            out[start:stop] = np.sqrt(np.min(np.sum(diff * diff, axis=2), axis=1))
        return out

    def plan(self, start: Point3, goal: Point3) -> tuple[bool, list[Point3]]:
        if not self.initialized or self.planner is None:
            self.last_error = "PCT Planner adapter is not initialized."
            return False, []

        try:
            import numpy as np

            start_np = np.asarray(start, dtype=np.float32)
            goal_np = np.asarray(goal, dtype=np.float32)
            if self._distance(start, goal) <= self.min_direct_path_distance:
                self.logger.warning(
                    "Start and goal are %.3fm apart; publishing a direct short path instead of calling PCT core."
                    % self._distance(start, goal)
                )
                return True, [start, goal]

            start_idx = self.planner.pose_to_idx(start_np)
            goal_idx = self.planner.pose_to_idx(goal_np)
            if not self.planner._is_valid_idx(start_idx, "start"):
                self.last_error = "Start pose is invalid in the PCT tomogram."
                return False, []
            if not self.planner._is_valid_idx(goal_idx, "goal"):
                self.last_error = "Goal pose is invalid in the PCT tomogram."
                return False, []
            if np.array_equal(start_idx, goal_idx):
                self.logger.warning(
                    "Start and goal map to the same PCT grid cell %s; publishing a direct short path."
                    % start_idx.tolist()
                )
                return True, [start, goal]

            traj = self.planner.plan(start_np, goal_np)
            if traj is None:
                self.last_error = "PCT Planner returned no path."
                return False, []

            traj = np.asarray(traj, dtype=np.float64)
            if traj.ndim != 2 or traj.shape[1] < 3 or traj.shape[0] == 0:
                self.last_error = f"PCT Planner returned an invalid trajectory shape: {traj.shape}"
                return False, []

            path = [
                (float(row[0]), float(row[1]), float(row[2]))
                for row in traj[:, :3]
            ]
            path = self._maybe_use_raw_astar_fallback(path)
            return True, path
        except Exception as exc:  # noqa: BLE001 - planning failures should surface in ROS logs/status.
            self.last_error = f"PCT planning failed: {exc}"
            return False, []

    def _maybe_use_raw_astar_fallback(self, path: list[Point3]) -> list[Point3]:
        safety_map = getattr(self.node, "safety_map", None)
        if safety_map is None or not getattr(safety_map, "loaded", False):
            return path
        if self._path_clearance_valid(path, safety_map):
            return path

        raw_traj = getattr(self.planner, "last_raw_traj_3d", None)
        if raw_traj is None:
            self.logger.warning("PCT optimized path failed height-aware clearance validation; no raw A* fallback available.")
            return path

        raw_traj = np.asarray(raw_traj, dtype=np.float64)
        if raw_traj.ndim != 2 or raw_traj.shape[1] < 3 or raw_traj.shape[0] == 0:
            self.logger.warning("PCT optimized path failed height-aware clearance validation; raw A* fallback is invalid.")
            return path

        raw_path = [
            (float(row[0]), float(row[1]), float(row[2]))
            for row in raw_traj[:, :3]
        ]
        if self._path_clearance_valid(raw_path, safety_map):
            self.logger.warning(
                "PCT optimized path failed height-aware clearance validation; using raw A* fallback to avoid railings."
            )
            return raw_path

        self.logger.warning(
            "PCT optimized path failed height-aware clearance validation, but raw A* fallback is also too close to obstacles."
        )
        return path

    def _path_clearance_valid(self, path: list[Point3], safety_map) -> bool:
        if len(path) < 2:
            return False
        sample_resolution = max(0.05, float(self._param("path_resample_resolution", 0.2)) * 0.5)
        for index, point in enumerate(path):
            if safety_map.query_obstacle_distance(point) < safety_map.required_clearance(point):
                return False
            if index + 1 >= len(path):
                continue
            segment = self._distance(point, path[index + 1])
            steps = max(1, int(math.ceil(segment / sample_resolution)))
            for step in range(1, steps):
                ratio = step / float(steps)
                sample = (
                    point[0] + (path[index + 1][0] - point[0]) * ratio,
                    point[1] + (path[index + 1][1] - point[1]) * ratio,
                    point[2] + (path[index + 1][2] - point[2]) * ratio,
                )
                if safety_map.query_obstacle_distance(sample) < safety_map.required_clearance(sample):
                    return False
        return True

    @staticmethod
    def _distance(a: Point3, b: Point3) -> float:
        return math.sqrt(
            (a[0] - b[0]) * (a[0] - b[0])
            + (a[1] - b[1]) * (a[1] - b[1])
            + (a[2] - b[2]) * (a[2] - b[2])
        )

    def _param(self, name: str, default):
        try:
            value = self.node.get_parameter(name).value
        except Exception:  # noqa: BLE001
            return default
        return default if value is None else value

    @staticmethod
    def _as_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    @staticmethod
    def _normalize_tomogram_path(tomogram_dir: str, tomogram_file: str) -> tuple[str, str]:
        expanded_file = Path(os.path.expanduser(tomogram_file))
        expanded_dir = str(Path(os.path.expanduser(tomogram_dir)))
        if expanded_file.suffix == ".pickle":
            if expanded_file.is_absolute() or str(expanded_file.parent) != ".":
                return str(expanded_file.parent), expanded_file.stem
            return expanded_dir, expanded_file.stem
        if expanded_file.is_absolute():
            return str(expanded_file.parent), expanded_file.name
        return expanded_dir, tomogram_file

    def _prepare_import_path(self) -> None:
        workspace = self._find_workspace_root()
        candidates = [
            self._param("pct_ros2_source_dir", ""),
            os.environ.get("PCT_PLANNER_ROS2_SOURCE_DIR", ""),
            str(workspace / "src" / "map_process" / "PCT_planner" / "pct_planner_ros2"),
            str(Path.cwd() / "src" / "map_process" / "PCT_planner" / "pct_planner_ros2"),
        ]

        for candidate in candidates:
            if not candidate:
                continue
            path = Path(os.path.expanduser(str(candidate))).resolve()
            if path.is_dir() and str(path) not in sys.path:
                sys.path.insert(0, str(path))

    @staticmethod
    def _find_workspace_root() -> Path:
        for candidate in (Path.cwd(), *Path.cwd().parents):
            if (candidate / "src").is_dir() and (candidate / "maps").is_dir():
                return candidate

        source = Path(__file__).resolve()
        for candidate in source.parents:
            if (candidate / "src").is_dir() and (candidate / "maps").is_dir():
                return candidate

        return Path.cwd()
