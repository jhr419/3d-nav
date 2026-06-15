import math
from dataclasses import dataclass, field

import numpy as np


OPEN3D_IMPORT_ERROR = None


@dataclass
class PcdPreprocessConfig:
    enable_manual_transform: bool = False
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0

    enable_auto_level: bool = True
    ground_percentile: float = 15.0
    max_ground_points: int = 50000
    ransac_distance_threshold: float = 0.05
    ransac_n: int = 3
    ransac_num_iterations: int = 1000
    normal_target_axis: list = field(default_factory=lambda: [0.0, 0.0, 1.0])
    enable_ground_z_shift: bool = True
    target_ground_z: float = 0.0
    ground_z_shift_mode: str = 'plane_inliers_median'
    ground_z_shift_percentile: float = 50.0
    random_seed: int = 42


def log_info(logger, message):
    if logger is not None:
        logger.info(message)


def log_warning(logger, message):
    if logger is None:
        return
    if hasattr(logger, 'warning'):
        logger.warning(message)
    else:
        logger.warn(message)


def as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def import_open3d():
    global OPEN3D_IMPORT_ERROR
    try:
        import open3d as o3d
    except Exception as exc:
        OPEN3D_IMPORT_ERROR = exc
        return None
    OPEN3D_IMPORT_ERROR = None
    return o3d


def rotation_matrix_x(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ], dtype=np.float64)


def rotation_matrix_y(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, 0.0, s],
        [0.0, 1.0, 0.0],
        [-s, 0.0, c],
    ], dtype=np.float64)


def rotation_matrix_z(angle):
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([
        [c, -s, 0.0],
        [s, c, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float64)


def euler_rotation_matrix(roll, pitch, yaw):
    return rotation_matrix_z(yaw) @ rotation_matrix_y(pitch) @ rotation_matrix_x(roll)


def normalize_vector(vector, name='vector'):
    vector = np.asarray(vector, dtype=np.float64)
    norm = np.linalg.norm(vector)
    if norm < 1e-12:
        raise ValueError('%s norm is too small' % name)
    return vector / norm


def skew_symmetric(vector):
    x, y, z = vector
    return np.array([
        [0.0, -z, y],
        [z, 0.0, -x],
        [-y, x, 0.0],
    ], dtype=np.float64)


def rotation_matrix_from_vectors(a, b):
    a = normalize_vector(a, 'a')
    b = normalize_vector(b, 'b')
    dot = float(np.clip(np.dot(a, b), -1.0, 1.0))

    if np.isclose(dot, 1.0, atol=1e-10):
        return np.eye(3, dtype=np.float64)

    if np.isclose(dot, -1.0, atol=1e-10):
        axis = np.cross(a, np.array([1.0, 0.0, 0.0], dtype=np.float64))
        if np.linalg.norm(axis) < 1e-8:
            axis = np.cross(a, np.array([0.0, 1.0, 0.0], dtype=np.float64))
        axis = normalize_vector(axis, 'anti_parallel_axis')
        k = skew_symmetric(axis)
        return np.eye(3, dtype=np.float64) + 2.0 * (k @ k)

    v = np.cross(a, b)
    s = np.linalg.norm(v)
    k = skew_symmetric(v)
    return np.eye(3, dtype=np.float64) + k + k @ k * ((1.0 - dot) / (s * s))


def preprocess_points(points, config, logger=None):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError('PCD must contain at least x, y, z columns; got shape %s' % (points.shape,))
    points = points[:, :3].copy()

    finite_mask = np.isfinite(points).all(axis=1)
    if not np.all(finite_mask):
        dropped = int(points.shape[0] - np.count_nonzero(finite_mask))
        log_warning(logger, 'Dropping %d non-finite points.' % dropped)
        points = points[finite_mask]
    if points.shape[0] == 0:
        raise ValueError('No finite points remain after loading PCD')

    log_point_stats('Loaded', points, logger)

    if as_bool(config.enable_manual_transform):
        points = apply_manual_transform(points, config, logger)

    if as_bool(config.enable_auto_level):
        points = apply_auto_level(points, config, logger)

    log_point_stats('Processed', points, logger)
    return points.astype(np.float32, copy=False)


def log_point_stats(label, points, logger=None):
    points_min = np.min(points, axis=0)
    points_max = np.max(points, axis=0)
    log_info(
        logger,
        '%s points: count=%d, x=[%.3f, %.3f], y=[%.3f, %.3f], z=[%.3f, %.3f]'
        % (
            label,
            points.shape[0],
            points_min[0],
            points_max[0],
            points_min[1],
            points_max[1],
            points_min[2],
            points_max[2],
        ),
    )


def apply_manual_transform(points, config, logger=None):
    translation = np.array([config.tx, config.ty, config.tz], dtype=np.float64)
    rotation = euler_rotation_matrix(config.roll, config.pitch, config.yaw)
    log_info(
        logger,
        'Applying manual transform: roll=%.6f, pitch=%.6f, yaw=%.6f, t=[%.6f, %.6f, %.6f]'
        % (config.roll, config.pitch, config.yaw, translation[0], translation[1], translation[2]),
    )
    log_matrix('Manual rotation matrix Rz*Ry*Rx', rotation, logger)
    return points @ rotation.T + translation


def apply_auto_level(points, config, logger=None):
    o3d = import_open3d()
    if o3d is None:
        raise RuntimeError(
            'enable_auto_level is true, but Open3D is not available (%s). '
            'Install Open3D or disable auto leveling and use manual transform only.'
            % OPEN3D_IMPORT_ERROR
        )

    ground_percentile = float(np.clip(config.ground_percentile, 0.0, 100.0))
    max_ground_points = max(3, int(config.max_ground_points))
    distance_threshold = float(config.ransac_distance_threshold)
    ransac_n = max(3, int(config.ransac_n))
    num_iterations = max(1, int(config.ransac_num_iterations))
    random_seed = int(config.random_seed)

    z_threshold = np.percentile(points[:, 2], ground_percentile)
    candidate_indices = np.flatnonzero(points[:, 2] <= z_threshold)
    if candidate_indices.size < ransac_n:
        raise RuntimeError(
            'Not enough ground candidates for RANSAC: %d < %d'
            % (candidate_indices.size, ransac_n)
        )

    if candidate_indices.size > max_ground_points:
        rng = np.random.default_rng(random_seed)
        candidate_indices = rng.choice(candidate_indices, size=max_ground_points, replace=False)

    candidates = points[candidate_indices]
    ground_cloud = o3d.geometry.PointCloud()
    ground_cloud.points = o3d.utility.Vector3dVector(candidates)

    log_info(
        logger,
        'Auto level: z percentile %.3f -> %.6f, ground candidates=%d'
        % (ground_percentile, z_threshold, candidates.shape[0]),
    )
    plane_model, inliers = ground_cloud.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )
    a, b, c, d = [float(value) for value in plane_model]
    normal = normalize_vector(np.array([a, b, c], dtype=np.float64), 'plane normal')
    if normal[2] < 0.0:
        normal = -normal
        a, b, c, d = -a, -b, -c, -d

    target_axis = normalize_vector(config.normal_target_axis, 'normal_target_axis')
    rotation = rotation_matrix_from_vectors(normal, target_axis)
    leveled = points @ rotation.T

    log_info(
        logger,
        'RANSAC plane: %.6f*x + %.6f*y + %.6f*z + %.6f = 0, inliers=%d'
        % (a, b, c, d, len(inliers)),
    )
    log_info(
        logger,
        'Plane normal after sign check: [%.9f, %.9f, %.9f], target=[%.9f, %.9f, %.9f]'
        % (normal[0], normal[1], normal[2], target_axis[0], target_axis[1], target_axis[2]),
    )
    log_matrix('Auto-level rotation matrix', rotation, logger)

    if as_bool(config.enable_ground_z_shift):
        reference_z = ground_shift_reference_z(
            leveled,
            candidate_indices,
            inliers,
            config,
            logger,
        )
        leveled[:, 2] += float(config.target_ground_z) - reference_z
        log_info(
            logger,
            'Shifted leveled cloud reference z from %.6f to target_ground_z %.6f'
            % (reference_z, float(config.target_ground_z)),
        )

    return leveled


def ground_shift_reference_z(leveled_points, candidate_indices, inliers, config, logger=None):
    mode = str(config.ground_z_shift_mode).strip().lower()
    if mode == 'min':
        return float(np.min(leveled_points[:, 2]))

    inliers = np.asarray(inliers, dtype=np.int64)
    if inliers.size == 0:
        log_warning(logger, 'No RANSAC inliers for z shift; falling back to global min z.')
        return float(np.min(leveled_points[:, 2]))

    ground_indices = candidate_indices[inliers]
    ground_z = leveled_points[ground_indices, 2]
    if mode in ('plane_inliers_median', 'inlier_median', 'median'):
        reference_z = float(np.median(ground_z))
    elif mode in ('plane_inliers_percentile', 'inlier_percentile', 'percentile'):
        percentile = float(np.clip(config.ground_z_shift_percentile, 0.0, 100.0))
        reference_z = float(np.percentile(ground_z, percentile))
    else:
        raise ValueError(
            'Unsupported ground_z_shift_mode "%s"; use min, plane_inliers_median, '
            'or plane_inliers_percentile.' % mode
        )

    log_info(
        logger,
        'Ground z shift mode=%s, reference_z=%.6f, inlier_z=[%.6f, %.6f]'
        % (mode, reference_z, float(np.min(ground_z)), float(np.max(ground_z))),
    )
    return reference_z


def log_matrix(label, matrix, logger=None):
    log_info(
        logger,
        '%s:\n[[%.9f, %.9f, %.9f],\n [%.9f, %.9f, %.9f],\n [%.9f, %.9f, %.9f]]'
        % (
            label,
            matrix[0, 0],
            matrix[0, 1],
            matrix[0, 2],
            matrix[1, 0],
            matrix[1, 1],
            matrix[1, 2],
            matrix[2, 0],
            matrix[2, 1],
            matrix[2, 2],
        ),
    )
