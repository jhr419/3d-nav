import ctypes
import importlib
import os
import pickle
import sys
from pathlib import Path

import numpy as np

from .utils import trans_traj_grid_to_map


def _log_warning(logger, message):
    if logger is None:
        return
    if hasattr(logger, 'warning'):
        logger.warning(message)
    else:
        logger.warn(message)


def _candidate_roots(planner_lib_dir):
    candidates = []
    if planner_lib_dir:
        candidates.append(Path(planner_lib_dir).expanduser())

    env_dir = os.environ.get('PCT_PLANNER_LIB_DIR')
    if env_dir:
        candidates.append(Path(env_dir).expanduser())

    repo_root = Path(__file__).resolve().parents[2]
    candidates.append(repo_root / 'planner' / 'lib')
    candidates.append(Path.cwd() / 'planner' / 'lib')

    unique = []
    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved not in unique:
            unique.append(resolved)
    return unique


def _python_module_paths(root):
    return [
        root,
        root / 'build' / 'src' / 'a_star',
        root / 'build' / 'src' / 'ele_planner',
        root / 'build' / 'src' / 'map_manager',
        root / 'build' / 'src' / 'trajectory_optimization',
        root / 'src' / 'a_star',
        root / 'src' / 'ele_planner',
        root / 'src' / 'map_manager',
        root / 'src' / 'trajectory_optimization',
    ]


def _shared_library_paths(root):
    return [
        root / '3rdparty' / 'gtsam-4.1.1' / 'install' / 'lib' / 'libmetis-gtsam.so',
        root / '3rdparty' / 'gtsam-4.1.1' / 'install' / 'lib' / 'libcephes-gtsam.so.1',
        root / '3rdparty' / 'gtsam-4.1.1' / 'install' / 'lib' / 'libgtsam.so.4',
        root / 'build' / 'src' / 'a_star' / 'liba_star_search.so',
        root / 'build' / 'src' / 'map_manager' / 'libmap_manager.so',
        root / 'build' / 'src' / 'common' / 'smoothing' / 'libcommon_smoothing.so',
        root / 'libcommon_smoothing.so',
        root / 'build' / 'src' / 'trajectory_optimization' / 'libgpmp_optimizer.so',
        root / 'build' / 'src' / 'ele_planner' / 'libele_planner_lib.so',
        root / 'src' / 'a_star' / 'liba_star_search.so',
        root / 'src' / 'map_manager' / 'libmap_manager.so',
        root / 'src' / 'common' / 'smoothing' / 'libcommon_smoothing.so',
        root / 'src' / 'trajectory_optimization' / 'libgpmp_optimizer.so',
        root / 'src' / 'ele_planner' / 'libele_planner_lib.so',
    ]


def _prepare_planner_library_path(root, logger=None):
    for module_path in _python_module_paths(root):
        if module_path.exists():
            module_path_str = str(module_path)
            if module_path_str not in sys.path:
                sys.path.insert(0, module_path_str)

    if not hasattr(ctypes, 'RTLD_GLOBAL'):
        return

    for lib_path in _shared_library_paths(root):
        if not lib_path.exists():
            continue
        try:
            ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)
        except OSError as exc:
            _log_warning(logger, f'Could not preload {lib_path}: {exc}')


def load_planner_modules(planner_lib_dir='', logger=None):
    tried = []
    last_error = None
    for root in _candidate_roots(planner_lib_dir):
        if not root.exists():
            tried.append(str(root))
            continue

        _prepare_planner_library_path(root, logger=logger)
        tried.append(str(root))
        try:
            return (
                importlib.import_module('a_star'),
                importlib.import_module('ele_planner'),
                importlib.import_module('traj_opt'),
            )
        except ImportError as exc:
            last_error = exc

    tried_text = '\n  - '.join(tried) if tried else '(none)'
    raise RuntimeError(
        'Could not import PCT Planner pybind modules: a_star, ele_planner, traj_opt.\n'
        'Build the original planner core first, then set planner_lib_dir or '
        'PCT_PLANNER_LIB_DIR to the built planner/lib directory.\n'
        f'Tried:\n  - {tried_text}\n'
        f'Last import error: {last_error}'
    )


class TomogramPlanner:
    def __init__(self, cfg, logger=None):
        self.cfg = cfg
        self.logger = logger

        self.use_quintic = self.cfg.use_quintic
        self.max_heading_rate = self.cfg.max_heading_rate
        self.path_z_offset = float(self.cfg.path_z_offset)
        self.astar_cost_threshold = float(self.cfg.astar_cost_threshold)
        self.astar_step_cost_weight = float(self.cfg.astar_step_cost_weight)
        self.optimizer_safe_cost_threshold = float(self.cfg.optimizer_safe_cost_threshold)
        self.max_path_z_jump = float(getattr(self.cfg, 'max_path_z_jump', 0.8))
        self.optimized_path_collision_cost_threshold = float(
            getattr(self.cfg, 'optimized_path_collision_cost_threshold', self.astar_cost_threshold)
        )
        self.optimized_path_collision_check_resolution = float(
            getattr(self.cfg, 'optimized_path_collision_check_resolution', 0.10)
        )

        self.tomo_dir = Path(self.cfg.tomogram_dir).expanduser()
        self.a_star, self.ele_planner, self.traj_opt = load_planner_modules(
            self.cfg.planner_lib_dir,
            logger=logger,
        )

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None
        self.trav = None
        self.elev_g = None
        self.elev_c = None
        self.a_start_cost_threshold = self.astar_cost_threshold
        self.last_raw_traj_3d = None
        self.last_optimized_traj_3d = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    def load_tomogram(self, tomo_file):
        tomo_path = self.tomo_dir / f'{tomo_file}.pickle'
        with open(tomo_path, 'rb') as handle:
            data_dict = pickle.load(handle)

        tomogram = np.asarray(data_dict['data'], dtype=np.float32)

        self.resolution = float(data_dict['resolution'])
        self.center = np.asarray(data_dict['center'], dtype=np.double)
        self.n_slice = tomogram.shape[1]
        self.slice_h0 = float(data_dict['slice_h0'])
        self.slice_dh = float(data_dict['slice_dh'])
        self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
        self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        elev_g = tomogram[3]
        elev_c = tomogram[4]
        self.trav_base = trav.copy()
        self.trav = trav.copy()
        self.trav_gx = trav_gx.copy()
        self.trav_gy = trav_gy.copy()
        self.elev_g = elev_g.copy()
        self.elev_c = elev_c.copy()
        elev_g = np.nan_to_num(elev_g, nan=-100)
        elev_c = np.nan_to_num(elev_c, nan=1e6)
        self.elev_g_planner = elev_g.copy()
        self.elev_c_planner = elev_c.copy()

        self.init_planner(trav, trav_gx, trav_gy, elev_g, elev_c)

    def init_planner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)

        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        self.planner = self.ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate,
            use_quintic=self.use_quintic,
        )
        self.planner.init_map(
            self.a_start_cost_threshold,
            self.optimizer_safe_cost_threshold,
            self.resolution,
            self.n_slice,
            self.astar_step_cost_weight,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            trav_gy.reshape(-1, trav_gy.shape[-1]).astype(np.double),
            -trav_gx.reshape(-1, trav_gx.shape[-1]).astype(np.double),
        )

    def apply_traversability_cost_overlay(
        self,
        overlay,
        max_total_cost=None,
        preserve_untraversable=True,
        recompute_gradients=True,
    ):
        if self.trav is None:
            return False

        overlay = np.asarray(overlay, dtype=np.float32)
        if overlay.ndim == 2:
            overlay = np.broadcast_to(overlay[np.newaxis, :, :], self.trav.shape)
        if overlay.shape != self.trav.shape:
            raise ValueError(
                f'Clearance cost overlay shape {overlay.shape} does not match tomogram cost shape {self.trav.shape}'
            )

        base = np.asarray(self.trav_base, dtype=np.float32)
        updated = base.copy()
        valid = np.isfinite(base) & np.isfinite(overlay) & (overlay > 0.0)
        if preserve_untraversable:
            valid &= base <= self.a_start_cost_threshold
        updated[valid] = base[valid] + overlay[valid]
        if max_total_cost is not None and max_total_cost > 0.0:
            updated[valid] = np.minimum(updated[valid], float(max_total_cost))

        self.trav = updated.astype(np.float32, copy=False)
        trav_gx = self.trav_gx.copy()
        trav_gy = self.trav_gy.copy()
        if recompute_gradients:
            trav_gx = np.zeros_like(self.trav, dtype=np.float32)
            trav_gy = np.zeros_like(self.trav, dtype=np.float32)
            trav_gx[:, 1:-1, :] = self.trav[:, 2:, :] - self.trav[:, :-2, :]
            trav_gy[:, :, 1:-1] = self.trav[:, :, 2:] - self.trav[:, :, :-2]

        self.init_planner(
            self.trav,
            trav_gx,
            trav_gy,
            self.elev_g_planner,
            self.elev_c_planner,
        )
        return True

    def plan(self, start_pos, end_pos):
        self.start_idx = self.pose_to_idx(start_pos)
        self.end_idx = self.pose_to_idx(end_pos)
        self._log_plan_indices()
        if not self._is_valid_idx(self.start_idx, 'start') or not self._is_valid_idx(self.end_idx, 'goal'):
            return None

        self.planner.plan(self.start_idx, self.end_idx, True)
        path_finder = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None
        raw_traj_3d = self.raw_path_to_traj(path)
        self.last_raw_traj_3d = raw_traj_3d
        self.last_optimized_traj_3d = None

        optimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        ground = self.sample_ground(traj[:, 0], traj[:, y_idx], layers)
        valid_ground = np.isfinite(ground)
        if np.any(valid_ground):
            heights = heights.copy()
            heights[valid_ground] = np.maximum(heights[valid_ground], ground[valid_ground])

        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = trans_traj_grid_to_map(
            self.map_dim,
            self.center,
            self.resolution,
            traj_3d,
            z_offset=self.path_z_offset,
        )
        self.last_optimized_traj_3d = traj_3d
        if self.has_large_z_jump(traj_3d):
            _log_warning(
                self.logger,
                'Optimized trajectory has a large z jump; falling back to raw A* path.',
            )
            if raw_traj_3d is not None and not self.has_large_z_jump(raw_traj_3d):
                return raw_traj_3d
            if self.logger is not None:
                self.logger.error('Rejected trajectory: raw A* path also has a large z jump.')
            return None

        if self.has_high_cost_grid_trajectory(traj[:, 0], traj[:, y_idx], layers):
            _log_warning(
                self.logger,
                'Optimized trajectory enters a high-cost cell; falling back to raw A* path.',
            )
            if raw_traj_3d is not None and not self.has_large_z_jump(raw_traj_3d):
                return raw_traj_3d
            if self.logger is not None:
                self.logger.error('Rejected trajectory: raw A* fallback is unavailable or unsafe.')
            return None

        return traj_3d

    def has_large_z_jump(self, traj_3d):
        if self.max_path_z_jump <= 0.0 or traj_3d is None or len(traj_3d) < 2:
            return False

        dz = np.abs(np.diff(traj_3d[:, 2]))
        finite_dz = dz[np.isfinite(dz)]
        return finite_dz.size > 0 and float(np.max(finite_dz)) > self.max_path_z_jump

    def has_high_cost_grid_trajectory(self, cols, rows, layers):
        threshold = self.optimized_path_collision_cost_threshold
        if threshold <= 0.0 or self.trav is None:
            return False

        cols = np.asarray(cols, dtype=np.float64)
        rows = np.asarray(rows, dtype=np.float64)
        layers = np.asarray(layers, dtype=np.float64)
        if cols.size == 0 or rows.size != cols.size or layers.size != cols.size:
            return False

        check_resolution = max(0.05, self.optimized_path_collision_check_resolution)
        grid_step = max(1.0, check_resolution / max(1.0e-6, self.resolution))
        for index in range(cols.size):
            if self._sample_trav_cost(layers[index], rows[index], cols[index]) > threshold:
                return True
            if index + 1 >= cols.size:
                continue
            d_col = cols[index + 1] - cols[index]
            d_row = rows[index + 1] - rows[index]
            d_layer = layers[index + 1] - layers[index]
            span = max(abs(d_col), abs(d_row), abs(d_layer))
            steps = max(1, int(np.ceil(span / grid_step)))
            for step in range(1, steps):
                ratio = step / float(steps)
                col = cols[index] + ratio * d_col
                row = rows[index] + ratio * d_row
                layer = layers[index] + ratio * d_layer
                if self._sample_trav_cost(layer, row, col) > threshold:
                    return True
        return False

    def _sample_trav_cost(self, layer, row, col):
        layer_i = int(np.rint(layer))
        row_i = int(np.rint(row))
        col_i = int(np.rint(col))
        if not (0 <= layer_i < self.n_slice and self._in_bounds(row_i, col_i)):
            return float('inf')
        cost = float(self.trav[layer_i, row_i, col_i])
        return cost if np.isfinite(cost) else float('inf')

    def raw_path_to_traj(self, path):
        path = np.asarray(path)
        if path.ndim != 2 or path.shape[0] == 0 or path.shape[1] < 3:
            return None

        layers = np.rint(path[:, 0]).astype(np.int32)
        rows = np.rint(path[:, 1]).astype(np.int32)
        cols = np.rint(path[:, 2]).astype(np.int32)
        heights = self.sample_ground(cols, rows, layers)
        if not np.all(np.isfinite(heights)):
            return None

        traj_grid = np.stack([cols, rows, heights / self.resolution], axis=1).astype(np.float32)
        return trans_traj_grid_to_map(
            self.map_dim,
            self.center,
            self.resolution,
            traj_grid,
            z_offset=self.path_z_offset,
        )

    def sample_ground(self, cols, rows, layers):
        cols = np.rint(cols).astype(np.int32)
        rows = np.rint(rows).astype(np.int32)
        layers = np.rint(layers).astype(np.int32)

        valid = (
            (0 <= layers)
            & (layers < self.n_slice)
            & (0 <= rows)
            & (rows < self.map_dim[0])
            & (0 <= cols)
            & (cols < self.map_dim[1])
        )
        ground = np.full(cols.shape, np.nan, dtype=np.float32)
        ground[valid] = self.elev_g[layers[valid], rows[valid], cols[valid]]
        return ground

    def pose_to_idx(self, pose):
        pose = np.asarray(pose, dtype=np.float32)
        xy_idx = self.pos_to_idx(pose[:2])
        layer = 0
        if pose.shape[0] >= 3 and not np.isnan(float(pose[2])):
            layer = self.layer_from_pose_z(float(pose[2]), xy_idx)

        return np.array([layer, xy_idx[0], xy_idx[1]], dtype=np.int32)

    def pos_to_idx(self, pos):
        pos = np.asarray(pos, dtype=np.float32)
        pos = pos - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        idx = np.array([idx[1], idx[0]], dtype=np.int32)
        return idx

    def layer_from_pose_z(self, z, xy_idx):
        col = int(xy_idx[0])
        row = int(xy_idx[1])
        if not self._in_bounds(row, col):
            return int(np.clip(round((z - self.slice_h0) / self.slice_dh), 0, self.n_slice - 1))

        heights = self.elev_g[:, row, col]
        costs = self.trav[:, row, col]
        valid = np.isfinite(heights) & (heights > -1e5)
        traversable = valid & np.isfinite(costs) & (costs <= self.a_start_cost_threshold)
        candidates = np.flatnonzero(traversable if np.any(traversable) else valid)
        if candidates.size == 0:
            return 0

        best = candidates[np.argmin(np.abs(heights[candidates] - z))]
        return int(np.clip(best, 0, self.n_slice - 1))

    def describe_idx(self, idx):
        layer = int(idx[0])
        col = int(idx[1])
        row = int(idx[2])
        if not (0 <= layer < self.n_slice and self._in_bounds(row, col)):
            return f'idx=[{layer}, {col}, {row}] out of bounds'

        cost = float(self.trav[layer, row, col])
        ground = float(self.elev_g[layer, row, col])
        ceiling = float(self.elev_c[layer, row, col])
        return (
            f'idx=[layer={layer}, col={col}, row={row}], '
            f'cost={cost:.3f}, ground={ground:.3f}, ceiling={ceiling:.3f}'
        )

    def _in_bounds(self, row, col):
        return 0 <= row < self.map_dim[0] and 0 <= col < self.map_dim[1]

    def _is_valid_idx(self, idx, role):
        layer = int(idx[0])
        col = int(idx[1])
        row = int(idx[2])
        ok = 0 <= layer < self.n_slice and self._in_bounds(row, col)
        if not ok:
            if self.logger is not None:
                self.logger.error(f'{role} pose is outside tomogram bounds: {self.describe_idx(idx)}')
            return False

        cost = float(self.trav[layer, row, col])
        if not np.isfinite(cost):
            if self.logger is not None:
                self.logger.error(f'{role} pose has invalid traversal cost: {self.describe_idx(idx)}')
            return False
        if cost > self.a_start_cost_threshold and self.logger is not None:
            self.logger.warning(
                f'{role} pose is on a high-cost cell and may be unreachable: {self.describe_idx(idx)}'
            )
        return True

    def _log_plan_indices(self):
        if self.logger is None:
            return
        self.logger.info('Start grid: ' + self.describe_idx(self.start_idx))
        self.logger.info('Goal grid: ' + self.describe_idx(self.end_idx))
