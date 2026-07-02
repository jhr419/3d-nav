import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from scipy.spatial import cKDTree
except Exception:  # noqa: BLE001 - scipy is optional at runtime.
    cKDTree = None


Point3 = tuple[float, float, float]


@dataclass
class PathPostprocessConfig:
    enable_path_smoothing: bool = True
    enable_path_resampling: bool = True
    path_resample_resolution: float = 0.2
    max_path_z_jump: float = 0.4
    remove_duplicate_points: bool = True
    duplicate_epsilon: float = 1.0e-4
    smoothing_passes: int = 1

    clearance_cost_enabled: bool = True
    robot_radius: float = 0.35
    leg_safety_margin: float = 0.20
    preferred_clearance: float = 0.75
    hard_min_clearance: float = 0.35
    clearance_weight: float = 2.0
    path_length_weight: float = 1.0
    smoothness_weight: float = 0.5
    centerline_weight: float = 1.0
    stair_edge_weight: float = 3.0
    unknown_as_occupied: bool = True

    adaptive_inflation_enabled: bool = True
    normal_inflation_radius: float = 0.45
    narrow_passage_inflation_radius: float = 0.25
    stair_area_inflation_radius: float = 0.60
    narrow_passage_detect_enabled: bool = True
    narrow_passage_width_threshold: float = 1.2
    narrow_passage_min_clearance: float = 0.25
    stair_edge_detect_enabled: bool = True
    stair_edge_z_gradient_threshold: float = 0.20
    stair_edge_clearance_boost: float = 1.5

    path_postprocess_enabled: bool = True
    enable_clearance_optimization: bool = True
    clearance_optimization_iterations: int = 30
    clearance_optimization_step: float = 0.05
    smoothing_iterations: int = 20
    max_smoothing_deviation: float = 0.5
    keep_start_goal_fixed: bool = True

    pcd_file: str = "maps/map_preprocessed.pcd"
    map_file: str = "maps/map_preprocessed.pcd"
    map_distance_max_points: int = 250000
    map_distance_xy_resolution: float = 0.15
    obstacle_min_relative_z: float = 0.12
    obstacle_max_relative_z: float = 1.60
    height_aware_clearance_enabled: bool = True
    body_obstacle_min_relative_z: float = 0.30
    body_obstacle_max_relative_z: float = 1.60
    body_clearance_check_radius: float = 0.70
    body_clearance_use_all_map_points: bool = False
    terrain_grid_resolution: float = 0.25
    terrain_edge_check_radius: float = 0.60


@dataclass
class PathRiskMetrics:
    path_length: float = 0.0
    min_clearance: float = math.inf
    avg_clearance: float = math.inf
    max_risk: float = 0.0
    max_risk_point: Point3 = (0.0, 0.0, 0.0)
    narrow_passage_count: int = 0
    stair_edge_risk_count: int = 0
    postprocess_improved: bool = False


@dataclass
class PathPostprocessResult:
    raw_path: list[Point3]
    basic_path: list[Point3]
    optimized_path: list[Point3]
    metrics_before: PathRiskMetrics
    metrics_after: PathRiskMetrics
    used_optimized_path: bool


def distance(a: Point3, b: Point3) -> float:
    return math.sqrt(
        (a[0] - b[0]) * (a[0] - b[0])
        + (a[1] - b[1]) * (a[1] - b[1])
        + (a[2] - b[2]) * (a[2] - b[2])
    )


def path_length(points: list[Point3]) -> float:
    return sum(distance(points[i], points[i + 1]) for i in range(len(points) - 1))


def _xy_distance(a: Point3, b: Point3) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _add(a: Point3, b: Point3) -> Point3:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _scale(a: Point3, scale: float) -> Point3:
    return (a[0] * scale, a[1] * scale, a[2] * scale)


def _interpolate(a: Point3, b: Point3, t: float) -> Point3:
    return (
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    )


def _find_workspace_root() -> Path:
    for candidate in (Path.cwd(), *Path.cwd().parents):
        if (candidate / "src").is_dir() and (candidate / "maps").is_dir():
            return candidate

    source = Path(__file__).resolve()
    for candidate in source.parents:
        if (candidate / "src").is_dir() and (candidate / "maps").is_dir():
            return candidate

    return Path.cwd()


def resolve_workspace_path(path: str) -> Path:
    value = str(path or "").strip()
    if not value:
        return Path()
    expanded = Path(os.path.expanduser(value))
    if expanded.is_absolute():
        return expanded
    return (_find_workspace_root() / expanded).resolve()


def remove_duplicates(points: list[Point3], epsilon: float = 1.0e-4) -> list[Point3]:
    if not points:
        return []

    out = [points[0]]
    for point in points[1:]:
        if distance(out[-1], point) > epsilon:
            out.append(point)
    return out


def clamp_z_jumps(points: list[Point3], max_jump: float) -> list[Point3]:
    if len(points) < 2 or max_jump <= 0.0:
        return points[:]

    out = [points[0]]
    for point in points[1:]:
        previous = out[-1]
        dz = abs(point[2] - previous[2])
        steps = max(1, int(math.ceil(dz / max_jump)))
        for step in range(1, steps + 1):
            ratio = step / float(steps)
            out.append(_interpolate(previous, point, ratio))
    return out


def resample_path(points: list[Point3], resolution: float) -> list[Point3]:
    if len(points) < 2 or resolution <= 0.0:
        return points[:]

    out = [points[0]]
    current = points[0]
    remaining = resolution

    for target in points[1:]:
        segment_length = distance(current, target)
        if segment_length <= 1.0e-9:
            current = target
            continue

        while segment_length >= remaining:
            ratio = remaining / segment_length
            next_point = _interpolate(current, target, ratio)
            out.append(next_point)
            current = next_point
            segment_length = distance(current, target)
            remaining = resolution

        remaining -= segment_length
        current = target

    if distance(out[-1], points[-1]) > 1.0e-6:
        out.append(points[-1])
    return out


def smooth_path(points: list[Point3], passes: int = 1) -> list[Point3]:
    if len(points) < 3 or passes <= 0:
        return points[:]

    smoothed = points[:]
    for _ in range(passes):
        next_points = [smoothed[0]]
        for index in range(1, len(smoothed) - 1):
            prev_p = smoothed[index - 1]
            cur_p = smoothed[index]
            next_p = smoothed[index + 1]
            next_points.append(
                (
                    0.25 * prev_p[0] + 0.50 * cur_p[0] + 0.25 * next_p[0],
                    0.25 * prev_p[1] + 0.50 * cur_p[1] + 0.25 * next_p[1],
                    0.25 * prev_p[2] + 0.50 * cur_p[2] + 0.25 * next_p[2],
                )
            )
        next_points.append(smoothed[-1])
        smoothed = next_points
    return smoothed


class PlanningSafetyMap:
    def __init__(self, cfg: PathPostprocessConfig, logger=None):
        self.cfg = cfg
        self.logger = logger
        self.map_points = np.empty((0, 3), dtype=np.float32)
        self.obstacle_points = np.empty((0, 3), dtype=np.float32)
        self.map_xy_tree = None
        self.obstacle_tree = None
        self.ground_lookup: dict[tuple[int, int], tuple[float, float]] = {}
        self.loaded = False
        self.load_error = ""

    def load(self) -> bool:
        candidates = [self.cfg.pcd_file, self.cfg.map_file]
        for candidate in candidates:
            path = resolve_workspace_path(candidate)
            if path.is_file() and self.load_from_pcd(path):
                return True
        self.load_error = "No PCD map file found for clearance queries."
        if self.logger:
            self.logger.warning(self.load_error)
        return False

    def load_from_pcd(self, pcd_path: Path) -> bool:
        try:
            from .map_publisher_node import read_pcd_xyz

            points = np.asarray(read_pcd_xyz(str(pcd_path)), dtype=np.float32)[:, :3]
        except Exception as exc:  # noqa: BLE001
            self.load_error = f"Failed to read PCD for clearance queries: {pcd_path}: {exc}"
            if self.logger:
                self.logger.warning(self.load_error)
            return False

        points = points[np.isfinite(points).all(axis=1)]
        if points.size == 0:
            self.load_error = f"PCD map has no valid XYZ points: {pcd_path}"
            if self.logger:
                self.logger.warning(self.load_error)
            return False

        self.map_points = self._limit_points(points, self.cfg.map_distance_max_points)
        self._build_height_lookup(points)
        self.obstacle_points = self._extract_obstacle_points(points)
        self.obstacle_points = self._limit_points(self.obstacle_points, self.cfg.map_distance_max_points)

        if self.obstacle_points.size == 0:
            self.obstacle_points = self._limit_points(points, self.cfg.map_distance_max_points)
            if self.logger:
                self.logger.warning(
                    "Ground-relative obstacle extraction found no obstacles; "
                    "falling back to all map points for clearance queries."
                )

        if cKDTree is not None and self.map_points.size:
            self.map_xy_tree = cKDTree(self.map_points[:, :2])
        else:
            self.map_xy_tree = None

        if cKDTree is not None and self.obstacle_points.size:
            self.obstacle_tree = cKDTree(self.obstacle_points[:, :2])
        else:
            self.obstacle_tree = None

        self.loaded = True
        if self.logger:
            self.logger.info(
                "Loaded clearance map from %s: map_points=%d obstacle_points=%d kdtree=%s"
                % (pcd_path, len(self.map_points), len(self.obstacle_points), self.obstacle_tree is not None)
            )
        return True

    def queryObstacleDistance(self, p: Point3) -> float:
        return self.query_obstacle_distance(p)

    def computeClearanceCost(self, p: Point3) -> float:
        return self.compute_clearance_cost(p)

    def computePathRiskCost(self, path: list[Point3]) -> float:
        return compute_path_risk_cost(path, self, self.cfg)

    def isNarrowPassage(self, p: Point3) -> bool:
        return self.is_narrow_passage(p)

    def isStairOrDropEdge(self, p: Point3) -> bool:
        return self.is_stair_or_drop_edge(p)

    def getAdaptiveInflationRadius(self, p: Point3) -> float:
        return self.get_adaptive_inflation_radius(p)

    def computeTerrainRiskCost(self, p: Point3) -> float:
        return self.compute_terrain_risk_cost(p)

    def query_obstacle_distance(self, p: Point3) -> float:
        if not self.loaded or (self.obstacle_points.size == 0 and self.map_points.size == 0):
            return math.inf
        if self.cfg.height_aware_clearance_enabled:
            body_distance = self.query_body_obstacle_distance(p)
            if math.isfinite(body_distance):
                return body_distance
        return self._query_projected_obstacle_distance(p)

    def query_body_obstacle_distance(self, p: Point3) -> float:
        points = self._nearby_body_obstacles(p, self.cfg.body_clearance_check_radius)
        if points.size == 0:
            return math.inf
        deltas = points[:, :2] - np.asarray([p[0], p[1]], dtype=np.float32)
        distances = np.hypot(deltas[:, 0], deltas[:, 1])
        if distances.size == 0:
            return math.inf
        return float(np.min(distances))

    def _query_projected_obstacle_distance(self, p: Point3) -> float:
        if not self.loaded or self.obstacle_points.size == 0:
            return math.inf
        query = np.asarray([p[0], p[1]], dtype=np.float64)
        if self.obstacle_tree is not None:
            dist, _idx = self.obstacle_tree.query(query, k=1)
            return float(dist) if np.isfinite(dist) else math.inf

        deltas = self.obstacle_points[:, :2] - query
        distances = np.hypot(deltas[:, 0], deltas[:, 1])
        if distances.size == 0:
            return math.inf
        return float(np.min(distances))

    def nearest_obstacle_vector(self, p: Point3) -> tuple[float, Point3]:
        if not self.loaded or (self.obstacle_points.size == 0 and self.map_points.size == 0):
            return math.inf, (0.0, 0.0, 0.0)
        if self.cfg.height_aware_clearance_enabled:
            body_distance, body_vector = self._nearest_body_obstacle_vector(p)
            if math.isfinite(body_distance):
                return body_distance, body_vector
        return self._nearest_projected_obstacle_vector(p)

    def _nearest_body_obstacle_vector(self, p: Point3) -> tuple[float, Point3]:
        points = self._nearby_body_obstacles(p, self.cfg.body_clearance_check_radius)
        if points.size == 0:
            return math.inf, (0.0, 0.0, 0.0)
        query = np.asarray([p[0], p[1]], dtype=np.float32)
        deltas = points[:, :2] - query
        distances = np.hypot(deltas[:, 0], deltas[:, 1])
        if distances.size == 0:
            return math.inf, (0.0, 0.0, 0.0)
        idx = int(np.argmin(distances))
        dist = float(distances[idx])
        obstacle = points[idx]
        vx = p[0] - float(obstacle[0])
        vy = p[1] - float(obstacle[1])
        norm = math.hypot(vx, vy)
        if norm <= 1.0e-9:
            return dist, (0.0, 0.0, 0.0)
        return dist, (vx / norm, vy / norm, 0.0)

    def _nearest_projected_obstacle_vector(self, p: Point3) -> tuple[float, Point3]:
        if not self.loaded or self.obstacle_points.size == 0:
            return math.inf, (0.0, 0.0, 0.0)
        query = np.asarray([p[0], p[1]], dtype=np.float64)
        if self.obstacle_tree is not None:
            dist, idx = self.obstacle_tree.query(query, k=1)
            if not np.isfinite(dist):
                return math.inf, (0.0, 0.0, 0.0)
            obstacle = self.obstacle_points[int(idx)]
        else:
            deltas = self.obstacle_points[:, :2] - query
            distances = np.hypot(deltas[:, 0], deltas[:, 1])
            if distances.size == 0:
                return math.inf, (0.0, 0.0, 0.0)
            idx = int(np.argmin(distances))
            dist = float(distances[idx])
            obstacle = self.obstacle_points[idx]
        vx = p[0] - float(obstacle[0])
        vy = p[1] - float(obstacle[1])
        norm = math.hypot(vx, vy)
        if norm <= 1.0e-9:
            return float(dist), (0.0, 0.0, 0.0)
        return float(dist), (vx / norm, vy / norm, 0.0)

    def compute_clearance_cost(self, p: Point3) -> float:
        if not self.cfg.clearance_cost_enabled:
            return 0.0

        clearance = self.query_obstacle_distance(p)
        min_clearance = self.required_clearance(p)
        if clearance < min_clearance:
            return 1.0e6 + (min_clearance - clearance) * 1.0e5

        preferred = max(min_clearance, self.cfg.preferred_clearance)
        if clearance >= preferred:
            base_cost = 0.0
        else:
            span = max(1.0e-3, preferred - min_clearance)
            ratio = (preferred - clearance) / span
            base_cost = self.cfg.clearance_weight * ratio * ratio
        return base_cost + self.compute_terrain_risk_cost(p)

    def required_clearance(self, p: Point3) -> float:
        if not self.cfg.adaptive_inflation_enabled:
            return self.cfg.hard_min_clearance
        if self.is_narrow_passage(p):
            return max(
                self.cfg.narrow_passage_min_clearance,
                min(self.cfg.hard_min_clearance, self.cfg.narrow_passage_inflation_radius),
            )
        if self.is_stair_or_drop_edge(p):
            return max(self.cfg.hard_min_clearance, self.cfg.stair_area_inflation_radius)
        return max(self.cfg.hard_min_clearance, self.cfg.normal_inflation_radius)

    def is_narrow_passage(self, p: Point3) -> bool:
        if not self.cfg.narrow_passage_detect_enabled or not self.loaded:
            return False
        clearance = self.query_obstacle_distance(p)
        half_width = 0.5 * self.cfg.narrow_passage_width_threshold
        if clearance > half_width:
            return False

        radius = max(half_width + 0.15, self.cfg.preferred_clearance)
        points = self._nearby_obstacles_xy(p, radius)
        if len(points) < 4:
            return False

        vectors = points[:, :2] - np.asarray([p[0], p[1]], dtype=np.float32)
        dists = np.hypot(vectors[:, 0], vectors[:, 1])
        valid = dists > 1.0e-3
        vectors = vectors[valid]
        dists = dists[valid]
        if len(vectors) < 4:
            return False

        unit = vectors / dists[:, None]
        close = dists <= max(half_width + 0.10, self.cfg.narrow_passage_min_clearance)
        unit = unit[close]
        dists = dists[close]
        if len(unit) < 2:
            return False
        for i in range(len(unit)):
            dots = unit[i + 1 :] @ unit[i]
            if np.any(dots < -0.55):
                return True
        return False

    def is_stair_or_drop_edge(self, p: Point3) -> bool:
        if not self.cfg.stair_edge_detect_enabled or not self.ground_lookup:
            return False
        return self.height_gradient(p) >= self.cfg.stair_edge_z_gradient_threshold

    def get_adaptive_inflation_radius(self, p: Point3) -> float:
        if not self.cfg.adaptive_inflation_enabled:
            return self.cfg.normal_inflation_radius
        if self.is_stair_or_drop_edge(p):
            return self.cfg.stair_area_inflation_radius
        if self.is_narrow_passage(p):
            return self.cfg.narrow_passage_inflation_radius
        return self.cfg.normal_inflation_radius

    def compute_terrain_risk_cost(self, p: Point3) -> float:
        if not self.is_stair_or_drop_edge(p):
            return 0.0
        gradient = self.height_gradient(p)
        threshold = max(1.0e-3, self.cfg.stair_edge_z_gradient_threshold)
        return self.cfg.stair_edge_weight * self.cfg.stair_edge_clearance_boost * (gradient / threshold)

    def height_gradient(self, p: Point3) -> float:
        if not self.ground_lookup:
            return 0.0
        resolution = self.cfg.terrain_grid_resolution
        radius_cells = max(1, int(math.ceil(self.cfg.terrain_edge_check_radius / resolution)))
        cx = int(math.floor(p[0] / resolution))
        cy = int(math.floor(p[1] / resolution))
        values = []
        for dx in range(-radius_cells, radius_cells + 1):
            for dy in range(-radius_cells, radius_cells + 1):
                item = self.ground_lookup.get((cx + dx, cy + dy))
                if item is None:
                    continue
                values.append(item[0])
                values.append(item[1])
        if len(values) < 2:
            return 0.0
        return float(max(values) - min(values))

    def sample_clearance_points(self, max_samples: int = 2500) -> list[tuple[Point3, float]]:
        if self.map_points.size == 0:
            return []
        stride = max(1, int(math.ceil(len(self.map_points) / max(1, max_samples))))
        out = []
        for row in self.map_points[::stride]:
            point = (float(row[0]), float(row[1]), float(row[2]))
            out.append((point, self.query_obstacle_distance(point)))
        return out

    def _nearby_obstacles_xy(self, p: Point3, radius: float) -> np.ndarray:
        if self.obstacle_points.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        if self.cfg.height_aware_clearance_enabled:
            body_points = self._nearby_body_obstacles(p, radius)
            if body_points.size:
                return body_points
        query = np.asarray([p[0], p[1]], dtype=np.float64)
        if self.obstacle_tree is not None:
            indices = self.obstacle_tree.query_ball_point(query, radius)
            if not indices:
                return np.empty((0, 3), dtype=np.float32)
            return self.obstacle_points[np.asarray(indices, dtype=np.int64)]
        deltas = self.obstacle_points[:, :2] - query
        dists = np.hypot(deltas[:, 0], deltas[:, 1])
        return self.obstacle_points[dists <= radius]

    def _nearby_body_obstacles(self, p: Point3, radius: float) -> np.ndarray:
        use_map_points = self.cfg.body_clearance_use_all_map_points or self.obstacle_points.size == 0
        source_points = self.map_points if use_map_points else self.obstacle_points
        source_tree = self.map_xy_tree if use_map_points else self.obstacle_tree
        if source_points.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        radius = max(0.05, float(radius))
        query = np.asarray([p[0], p[1]], dtype=np.float64)
        if source_tree is not None:
            indices = source_tree.query_ball_point(query, radius)
            if not indices:
                return np.empty((0, 3), dtype=np.float32)
            candidates = source_points[np.asarray(indices, dtype=np.int64)]
        else:
            deltas = source_points[:, :2] - query
            dists = np.hypot(deltas[:, 0], deltas[:, 1])
            candidates = source_points[dists <= radius]
        if candidates.size == 0:
            return np.empty((0, 3), dtype=np.float32)

        z_min = p[2] + min(self.cfg.body_obstacle_min_relative_z, self.cfg.body_obstacle_max_relative_z)
        z_max = p[2] + max(self.cfg.body_obstacle_min_relative_z, self.cfg.body_obstacle_max_relative_z)
        mask = (candidates[:, 2] >= z_min) & (candidates[:, 2] <= z_max)
        return candidates[mask].astype(np.float32, copy=False)

    def _build_height_lookup(self, points: np.ndarray) -> None:
        resolution = self.cfg.terrain_grid_resolution
        if points.size == 0 or resolution <= 0.0:
            self.ground_lookup = {}
            return
        keys = np.floor(points[:, :2] / resolution).astype(np.int32)
        lookup: dict[tuple[int, int], tuple[float, float]] = {}
        for key, z in zip(keys, points[:, 2]):
            item = (int(key[0]), int(key[1]))
            old = lookup.get(item)
            if old is None:
                lookup[item] = (float(z), float(z))
            else:
                lookup[item] = (min(old[0], float(z)), max(old[1], float(z)))
        self.ground_lookup = lookup

    def _extract_obstacle_points(self, points: np.ndarray) -> np.ndarray:
        resolution = self.cfg.terrain_grid_resolution
        keys = np.floor(points[:, :2] / resolution).astype(np.int32)
        ground_min = np.empty(points.shape[0], dtype=np.float32)
        for index, key in enumerate(keys):
            ground_min[index] = self.ground_lookup.get((int(key[0]), int(key[1])), (points[index, 2], points[index, 2]))[0]
        rel_z = points[:, 2] - ground_min
        mask = (
            (rel_z >= self.cfg.obstacle_min_relative_z)
            & (rel_z <= self.cfg.obstacle_max_relative_z)
        )
        obstacles = points[mask]
        if obstacles.size == 0:
            return obstacles

        xy_resolution = max(0.03, self.cfg.map_distance_xy_resolution)
        voxel_keys = np.floor(obstacles[:, :3] / xy_resolution).astype(np.int32)
        _, unique_indices = np.unique(voxel_keys, axis=0, return_index=True)
        return obstacles[np.sort(unique_indices)]

    @staticmethod
    def _limit_points(points: np.ndarray, max_points: int) -> np.ndarray:
        if max_points <= 0 or len(points) <= max_points:
            return points.astype(np.float32, copy=False)
        stride = int(math.ceil(len(points) / max_points))
        return points[::stride].astype(np.float32, copy=False)


def compute_path_metrics(
    points: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
) -> PathRiskMetrics:
    metrics = PathRiskMetrics(path_length=path_length(points))
    if not points or safety_map is None or not safety_map.loaded or not cfg.clearance_cost_enabled:
        return metrics

    clearances = []
    max_risk = -math.inf
    max_risk_point = points[0]
    narrow_count = 0
    stair_count = 0
    for point in points:
        clearance = safety_map.query_obstacle_distance(point)
        risk = safety_map.compute_clearance_cost(point)
        clearances.append(clearance)
        if risk > max_risk:
            max_risk = risk
            max_risk_point = point
        if safety_map.is_narrow_passage(point):
            narrow_count += 1
        if safety_map.is_stair_or_drop_edge(point):
            stair_count += 1

    finite_clearances = [value for value in clearances if math.isfinite(value)]
    if finite_clearances:
        metrics.min_clearance = min(finite_clearances)
        metrics.avg_clearance = sum(finite_clearances) / len(finite_clearances)
    metrics.max_risk = max(0.0, max_risk if math.isfinite(max_risk) else 0.0)
    metrics.max_risk_point = max_risk_point
    metrics.narrow_passage_count = narrow_count
    metrics.stair_edge_risk_count = stair_count
    return metrics


def compute_path_risk_cost(
    path: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
) -> float:
    if not path:
        return math.inf
    length_cost = cfg.path_length_weight * path_length(path)
    if safety_map is None or not safety_map.loaded or not cfg.clearance_cost_enabled:
        return length_cost
    risk = sum(safety_map.compute_clearance_cost(point) for point in path)
    return length_cost + risk / max(1, len(path))


def validate_path_collision_free(
    path: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
) -> bool:
    if len(path) < 2:
        return False
    if safety_map is None or not safety_map.loaded or not cfg.clearance_cost_enabled:
        return True

    sample_resolution = max(0.05, cfg.path_resample_resolution * 0.5)
    for index, point in enumerate(path):
        if safety_map.query_obstacle_distance(point) < safety_map.required_clearance(point):
            return False
        if index + 1 >= len(path):
            continue
        segment = distance(path[index], path[index + 1])
        steps = max(1, int(math.ceil(segment / sample_resolution)))
        for step in range(1, steps):
            sample = _interpolate(path[index], path[index + 1], step / float(steps))
            if safety_map.query_obstacle_distance(sample) < safety_map.required_clearance(sample):
                return False
    return True


def _segment_min_clearance(
    start: Point3,
    end: Point3,
    safety_map: PlanningSafetyMap,
    cfg: PathPostprocessConfig,
) -> float:
    sample_resolution = max(0.05, cfg.path_resample_resolution * 0.5)
    segment = distance(start, end)
    steps = max(1, int(math.ceil(segment / sample_resolution)))
    best = math.inf
    for step in range(steps + 1):
        sample = _interpolate(start, end, step / float(steps))
        best = min(best, safety_map.query_obstacle_distance(sample))
    return best


def _local_path_quality(
    path: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
) -> tuple[float, float]:
    if not path or safety_map is None or not safety_map.loaded:
        return 0.0, math.inf

    risk = 0.0
    smoothness = 0.0
    min_clearance = math.inf
    for point in path:
        risk += safety_map.compute_clearance_cost(point)
        min_clearance = min(min_clearance, safety_map.query_obstacle_distance(point))
    for index in range(len(path) - 1):
        min_clearance = min(
            min_clearance,
            _segment_min_clearance(path[index], path[index + 1], safety_map, cfg),
        )
    for index in range(1, len(path) - 1):
        prev_p = path[index - 1]
        cur_p = path[index]
        next_p = path[index + 1]
        smoothness += (
            (prev_p[0] - 2.0 * cur_p[0] + next_p[0]) ** 2
            + (prev_p[1] - 2.0 * cur_p[1] + next_p[1]) ** 2
            + 0.25 * (prev_p[2] - 2.0 * cur_p[2] + next_p[2]) ** 2
        )
    quality = (
        risk / max(1, len(path))
        + cfg.smoothness_weight * smoothness
        + 0.1 * cfg.path_length_weight * path_length(path)
    )
    return quality, min_clearance


def _local_change_improves_clearance(
    old_segment: list[Point3],
    new_segment: list[Point3],
    safety_map: PlanningSafetyMap,
    cfg: PathPostprocessConfig,
) -> bool:
    old_quality, old_min_clearance = _local_path_quality(old_segment, safety_map, cfg)
    new_quality, new_min_clearance = _local_path_quality(new_segment, safety_map, cfg)
    min_tolerance = max(0.01, cfg.path_resample_resolution * 0.05)
    quality_tolerance = max(1.0e-6, 0.02 * max(1.0, old_quality))
    return (
        new_min_clearance + min_tolerance >= old_min_clearance
        and new_quality <= old_quality + quality_tolerance
    )


def _limit_vector(vec: Point3, max_norm: float) -> Point3:
    norm = math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2])
    if norm <= max_norm or norm <= 1.0e-9:
        return vec
    scale = max_norm / norm
    return (vec[0] * scale, vec[1] * scale, vec[2] * scale)


def _path_normal_clearance_direction(
    optimized: list[Point3],
    index: int,
    raw_direction: Point3,
) -> Point3:
    if index <= 0 or index + 1 >= len(optimized):
        return raw_direction
    prev_p = optimized[index - 1]
    next_p = optimized[index + 1]
    tx = next_p[0] - prev_p[0]
    ty = next_p[1] - prev_p[1]
    t_norm = math.hypot(tx, ty)
    if t_norm <= 1.0e-6:
        return raw_direction
    tx /= t_norm
    ty /= t_norm
    dot = raw_direction[0] * tx + raw_direction[1] * ty
    nx = raw_direction[0] - dot * tx
    ny = raw_direction[1] - dot * ty
    n_norm = math.hypot(nx, ny)
    if n_norm <= 0.15:
        return raw_direction
    return (nx / n_norm, ny / n_norm, 0.0)


def optimize_path_clearance(
    points: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
) -> list[Point3]:
    if (
        len(points) < 3
        or safety_map is None
        or not safety_map.loaded
        or not cfg.enable_clearance_optimization
        or not cfg.clearance_cost_enabled
    ):
        return points[:]

    optimized = points[:]
    original = points[:]
    first_index = 1 if cfg.keep_start_goal_fixed else 0
    last_index = len(points) - 1 if cfg.keep_start_goal_fixed else len(points)
    step_size = max(0.0, cfg.clearance_optimization_step)

    for _ in range(max(0, cfg.clearance_optimization_iterations)):
        changed = False
        for index in range(first_index, last_index):
            point = optimized[index]
            clearance, direction = safety_map.nearest_obstacle_vector(point)
            target_clearance = max(
                safety_map.required_clearance(point),
                min(cfg.preferred_clearance, cfg.narrow_passage_width_threshold * 0.5)
                if safety_map.is_narrow_passage(point)
                else cfg.preferred_clearance,
            )
            if not math.isfinite(clearance) or clearance >= target_clearance:
                continue
            if math.hypot(direction[0], direction[1]) <= 1.0e-6:
                continue

            severity = (target_clearance - clearance) / max(1.0e-3, target_clearance)
            clearance_direction = _path_normal_clearance_direction(optimized, index, direction)
            clearance_move = _scale(clearance_direction, step_size * max(0.25, min(1.0, severity)))
            smooth_move = (0.0, 0.0, 0.0)
            if 0 < index and index + 1 < len(optimized):
                prev_p = optimized[index - 1]
                next_p = optimized[index + 1]
                midpoint = (
                    0.5 * (prev_p[0] + next_p[0]),
                    0.5 * (prev_p[1] + next_p[1]),
                    0.5 * (prev_p[2] + next_p[2]),
                )
                smooth_move = _scale(
                    (midpoint[0] - point[0], midpoint[1] - point[1], midpoint[2] - point[2]),
                    min(0.35, 0.15 + 0.10 * cfg.smoothness_weight),
                )
            move = _limit_vector(
                _add(clearance_move, smooth_move),
                max(0.03, step_size * 1.25),
            )
            candidate = _add(point, move)
            if _xy_distance(candidate, original[index]) > cfg.max_smoothing_deviation:
                continue
            if safety_map.compute_clearance_cost(candidate) > safety_map.compute_clearance_cost(point):
                continue

            start = max(0, index - 1)
            stop = min(len(optimized), index + 2)
            old_segment = optimized[start:stop]
            new_segment = old_segment[:]
            new_segment[index - start] = candidate
            if _local_change_improves_clearance(old_segment, new_segment, safety_map, cfg):
                optimized[index] = candidate
                changed = True
        if not changed:
            break

    return optimized


def smooth_path_with_validation(
    points: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
) -> list[Point3]:
    if len(points) < 3 or cfg.smoothing_iterations <= 0:
        return points[:]

    smoothed = points[:]
    original = points[:]
    first_index = 1
    last_index = len(points) - 1

    for _ in range(cfg.smoothing_iterations):
        changed = False
        next_points = smoothed[:]
        for index in range(first_index, last_index):
            prev_p = smoothed[index - 1]
            cur_p = smoothed[index]
            next_p = smoothed[index + 1]
            midpoint = (
                0.25 * prev_p[0] + 0.50 * cur_p[0] + 0.25 * next_p[0],
                0.25 * prev_p[1] + 0.50 * cur_p[1] + 0.25 * next_p[1],
                0.25 * prev_p[2] + 0.50 * cur_p[2] + 0.25 * next_p[2],
            )
            if _xy_distance(midpoint, original[index]) > cfg.max_smoothing_deviation:
                continue
            start = max(0, index - 1)
            stop = min(len(next_points), index + 2)
            old_segment = next_points[start:stop]
            new_segment = old_segment[:]
            new_segment[index - start] = midpoint
            if _local_change_improves_clearance(old_segment, new_segment, safety_map, cfg):
                next_points[index] = midpoint
                changed = True
        smoothed = next_points
        if not changed:
            break
    return smoothed


def basic_postprocess_path(points: list[Point3], cfg: PathPostprocessConfig) -> list[Point3]:
    processed = points[:]

    if cfg.remove_duplicate_points:
        processed = remove_duplicates(processed, cfg.duplicate_epsilon)

    processed = clamp_z_jumps(processed, cfg.max_path_z_jump)

    if cfg.enable_path_resampling:
        processed = resample_path(processed, cfg.path_resample_resolution)

    if cfg.enable_path_smoothing:
        processed = smooth_path(processed, cfg.smoothing_passes)
        processed = clamp_z_jumps(processed, cfg.max_path_z_jump)

    if cfg.remove_duplicate_points:
        processed = remove_duplicates(processed, cfg.duplicate_epsilon)

    return processed


def postprocess_path_with_report(
    points: list[Point3],
    cfg: PathPostprocessConfig,
    safety_map: Optional[PlanningSafetyMap] = None,
) -> PathPostprocessResult:
    basic = basic_postprocess_path(points, cfg)
    before = compute_path_metrics(basic, safety_map, cfg)
    if not cfg.path_postprocess_enabled:
        return PathPostprocessResult(points[:], basic, basic, before, before, False)

    candidate = basic[:]
    candidate = optimize_path_clearance(candidate, safety_map, cfg)
    if cfg.enable_path_resampling:
        candidate = resample_path(candidate, cfg.path_resample_resolution)
    if cfg.enable_path_smoothing:
        candidate = smooth_path_with_validation(candidate, safety_map, cfg)
    if cfg.enable_path_resampling:
        candidate = resample_path(candidate, cfg.path_resample_resolution)
    final_smoothing_passes = min(8, max(0, cfg.smoothing_iterations // 10))
    if final_smoothing_passes > 0 and safety_map is not None and safety_map.loaded:
        final_candidate = smooth_path(candidate, final_smoothing_passes)
        if cfg.enable_path_resampling:
            final_candidate = resample_path(final_candidate, cfg.path_resample_resolution)
        current_metrics = compute_path_metrics(candidate, safety_map, cfg)
        final_metrics = compute_path_metrics(final_candidate, safety_map, cfg)
        if (
            final_metrics.avg_clearance >= before.avg_clearance + 0.02
            and final_metrics.min_clearance + 0.02 >= current_metrics.min_clearance
        ):
            candidate = final_candidate
    candidate = clamp_z_jumps(candidate, cfg.max_path_z_jump)
    if cfg.remove_duplicate_points:
        candidate = remove_duplicates(candidate, cfg.duplicate_epsilon)

    after = compute_path_metrics(candidate, safety_map, cfg)
    candidate_valid = validate_path_collision_free(candidate, safety_map, cfg)
    before_cost = compute_path_risk_cost(basic, safety_map, cfg)
    after_cost = compute_path_risk_cost(candidate, safety_map, cfg)
    improved = candidate_valid and after_cost <= before_cost + 1.0e-6

    if not improved and math.isfinite(before.min_clearance) and math.isfinite(after.min_clearance):
        improved = candidate_valid and after.min_clearance >= before.min_clearance

    if not improved and math.isfinite(before.avg_clearance) and math.isfinite(after.avg_clearance):
        soft_clearance_improved = (
            after_cost < before_cost
            and after.avg_clearance >= before.avg_clearance + 0.02
            and after.min_clearance + 0.01 >= before.min_clearance
        )
        improved = soft_clearance_improved

    if improved:
        after.postprocess_improved = True
        return PathPostprocessResult(points[:], basic, candidate, before, after, True)

    before.postprocess_improved = False
    return PathPostprocessResult(points[:], basic, basic, before, before, False)


def postprocess_path(points: list[Point3], cfg: PathPostprocessConfig) -> list[Point3]:
    return basic_postprocess_path(points, cfg)


def resamplePath(points: list[Point3], resolution: float) -> list[Point3]:
    return resample_path(points, resolution)


def smoothPath(points: list[Point3], passes: int = 1) -> list[Point3]:
    return smooth_path(points, passes)


def optimizePathClearance(
    points: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
) -> list[Point3]:
    return optimize_path_clearance(points, safety_map, cfg)


def validatePathCollisionFree(
    path: list[Point3],
    safety_map: Optional[PlanningSafetyMap],
    cfg: PathPostprocessConfig,
) -> bool:
    return validate_path_collision_free(path, safety_map, cfg)
