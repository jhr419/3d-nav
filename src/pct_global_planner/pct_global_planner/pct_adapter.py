import os
import sys
import math
from pathlib import Path


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
            )

            self.planner = TomogramPlanner(planner_cfg, logger=self.logger)
            self.logger.info(f"Loading PCT tomogram: {tomogram_path}")
            self.planner.load_tomogram(tomogram_file)
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
            return True, path
        except Exception as exc:  # noqa: BLE001 - planning failures should surface in ROS logs/status.
            self.last_error = f"PCT planning failed: {exc}"
            return False, []

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
        candidates = [
            self._param("pct_ros2_source_dir", ""),
            os.environ.get("PCT_PLANNER_ROS2_SOURCE_DIR", ""),
            str(Path.cwd() / "src" / "PCT_planner" / "pct_planner_ros2"),
            str(Path(__file__).resolve().parents[2] / "PCT_planner" / "pct_planner_ros2"),
        ]

        for candidate in candidates:
            if not candidate:
                continue
            path = Path(os.path.expanduser(str(candidate))).resolve()
            if path.is_dir() and str(path) not in sys.path:
                sys.path.insert(0, str(path))
