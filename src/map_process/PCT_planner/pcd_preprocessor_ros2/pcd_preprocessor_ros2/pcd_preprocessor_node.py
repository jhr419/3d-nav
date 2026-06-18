import math
import os
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


OPEN3D_IMPORT_ERROR = None


def latched_qos():
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


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


def _log_warning(logger, message):
    if logger is None:
        return
    if hasattr(logger, 'warning'):
        logger.warning(message)
    else:
        logger.warn(message)


def _pcd_dtype(type_name, size):
    if type_name == 'F':
        if size == 4:
            return '<f4'
        if size == 8:
            return '<f8'
    if type_name == 'I':
        if size in (1, 2, 4, 8):
            return '<i%d' % size
    if type_name == 'U':
        if size in (1, 2, 4, 8):
            return '<u%d' % size
    raise ValueError('Unsupported PCD field type: %s%d' % (type_name, size))


def _read_pcd_header(pcd_file):
    header = {}
    while True:
        line = pcd_file.readline()
        if not line:
            raise ValueError('Invalid PCD file: missing DATA header')

        text = line.decode('utf-8', errors='replace').strip()
        if not text or text.startswith('#'):
            continue

        tokens = text.split()
        key = tokens[0].upper()
        header[key] = tokens[1:]
        if key == 'DATA':
            if len(tokens) < 2:
                raise ValueError('Invalid PCD file: DATA header is empty')
            return header


def _pcd_points_count(header):
    if 'POINTS' in header:
        return int(header['POINTS'][0])
    if 'WIDTH' in header and 'HEIGHT' in header:
        return int(header['WIDTH'][0]) * int(header['HEIGHT'][0])
    raise ValueError('Invalid PCD file: missing POINTS or WIDTH/HEIGHT')


def _pcd_xyz_columns(fields, counts):
    offsets = {}
    column = 0
    for field, count in zip(fields, counts):
        offsets.setdefault(field, column)
        column += count

    try:
        return [offsets['x'], offsets['y'], offsets['z']]
    except KeyError as exc:
        raise ValueError('PCD file must contain x, y, and z fields') from exc


def read_pcd_xyz_fallback(pcd_path):
    with open(pcd_path, 'rb') as pcd_file:
        header = _read_pcd_header(pcd_file)

        fields = header.get('FIELDS')
        sizes = [int(value) for value in header.get('SIZE', [])]
        types = header.get('TYPE')
        counts = [int(value) for value in header.get('COUNT', [])]
        if not counts and fields:
            counts = [1] * len(fields)

        if not fields or not sizes or not types:
            raise ValueError('Invalid PCD file: missing FIELDS, SIZE, or TYPE')
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError('Invalid PCD file: inconsistent field metadata')

        data_kind = header['DATA'][0].lower()
        point_count = _pcd_points_count(header)

        if data_kind == 'ascii':
            columns = _pcd_xyz_columns(fields, counts)
            points = np.loadtxt(pcd_file, dtype=np.float32, usecols=columns)
            return np.atleast_2d(points).astype(np.float64, copy=False)

        if data_kind != 'binary':
            raise NotImplementedError(
                'Fallback reader supports ASCII and uncompressed binary PCD only. '
                'Install Open3D for this PCD format.'
            )

        dtype_fields = []
        dtype_names = {}
        for index, (field, size, type_name, count) in enumerate(zip(fields, sizes, types, counts)):
            dtype_name = field if field not in dtype_names.values() else '%s_%d' % (field, index)
            dtype_names.setdefault(field, dtype_name)
            dtype = _pcd_dtype(type_name, size)
            if count == 1:
                dtype_fields.append((dtype_name, dtype))
            else:
                dtype_fields.append((dtype_name, dtype, (count,)))

        point_dtype = np.dtype(dtype_fields)
        data = np.fromfile(pcd_file, dtype=point_dtype, count=point_count)
        if data.shape[0] != point_count:
            raise ValueError('Invalid PCD file: expected %d points, read %d' % (point_count, data.shape[0]))

        xyz = []
        for field in ('x', 'y', 'z'):
            if field not in dtype_names:
                raise ValueError('PCD file must contain x, y, and z fields')
            values = data[dtype_names[field]]
            if values.ndim > 1:
                values = values[:, 0]
            xyz.append(values)
        return np.column_stack(xyz).astype(np.float64, copy=False)


def read_pcd_xyz(pcd_path, logger=None):
    if not os.path.isfile(pcd_path):
        raise FileNotFoundError('PCD file not found: %s' % pcd_path)

    o3d = import_open3d()
    if o3d is not None:
        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points, dtype=np.float64)
        if points.size == 0:
            raise ValueError('Open3D returned an empty point cloud: %s' % pcd_path)
        return points

    _log_warning(logger, 'Open3D is not available (%s); using fallback PCD reader.' % OPEN3D_IMPORT_ERROR)
    return read_pcd_xyz_fallback(pcd_path)


def write_pcd_xyz_fallback(pcd_path, points):
    points = np.asarray(points, dtype=np.float32)
    Path(pcd_path).parent.mkdir(parents=True, exist_ok=True)
    with open(pcd_path, 'w', encoding='utf-8') as pcd_file:
        pcd_file.write('# .PCD v0.7 - Point Cloud Data file format\n')
        pcd_file.write('VERSION 0.7\n')
        pcd_file.write('FIELDS x y z\n')
        pcd_file.write('SIZE 4 4 4\n')
        pcd_file.write('TYPE F F F\n')
        pcd_file.write('COUNT 1 1 1\n')
        pcd_file.write('WIDTH %d\n' % points.shape[0])
        pcd_file.write('HEIGHT 1\n')
        pcd_file.write('VIEWPOINT 0 0 0 1 0 0 0\n')
        pcd_file.write('POINTS %d\n' % points.shape[0])
        pcd_file.write('DATA ascii\n')
        np.savetxt(pcd_file, points[:, :3], fmt='%.8f %.8f %.8f')


def write_pcd_xyz(pcd_path, points, logger=None):
    Path(pcd_path).parent.mkdir(parents=True, exist_ok=True)
    o3d = import_open3d()
    if o3d is not None:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64)[:, :3])
        ok = o3d.io.write_point_cloud(pcd_path, pcd)
        if not ok:
            raise RuntimeError('Open3D failed to write PCD: %s' % pcd_path)
        return

    _log_warning(
        logger,
        'Open3D is not available (%s); writing ASCII XYZ PCD with fallback writer.'
        % OPEN3D_IMPORT_ERROR,
    )
    write_pcd_xyz_fallback(pcd_path, points)


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


class PcdPreprocessorNode(Node):
    def __init__(self):
        super().__init__('pcd_preprocessor_node')

        self.declare_parameter('input_pcd', '')
        self.declare_parameter('output_pcd', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('processed_points_topic', '/processed_points')
        self.declare_parameter('publish_period_sec', 1.0)

        self.declare_parameter('enable_manual_transform', False)
        self.declare_parameter('roll', 0.0)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('tx', 0.0)
        self.declare_parameter('ty', 0.0)
        self.declare_parameter('tz', 0.0)

        self.declare_parameter('enable_auto_level', True)
        self.declare_parameter('ground_percentile', 15.0)
        self.declare_parameter('max_ground_points', 50000)
        self.declare_parameter('ransac_distance_threshold', 0.05)
        self.declare_parameter('ransac_n', 3)
        self.declare_parameter('ransac_num_iterations', 1000)
        self.declare_parameter('normal_target_axis', [0.0, 0.0, 1.0])
        self.declare_parameter('target_ground_z', 0.0)
        self.declare_parameter('enable_ground_z_shift', True)
        self.declare_parameter('ground_z_shift_mode', 'plane_inliers_median')
        self.declare_parameter('ground_z_shift_percentile', 50.0)
        self.declare_parameter('random_seed', 42)

        self.input_pcd = str(self.get_parameter('input_pcd').value)
        self.output_pcd = str(self.get_parameter('output_pcd').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.topic = str(self.get_parameter('processed_points_topic').value)
        self.publish_period_sec = max(0.1, float(self.get_parameter('publish_period_sec').value))

        self.publisher = self.create_publisher(PointCloud2, self.topic, latched_qos())
        self.cloud_msg = None
        self.points = None

        try:
            self.process_once()
        except Exception as exc:
            self.get_logger().error('PCD preprocessing failed: %s' % exc)
            raise

        self.timer = self.create_timer(self.publish_period_sec, self.publish_processed_cloud)
        self.publish_processed_cloud()

    def process_once(self):
        if not self.input_pcd:
            raise ValueError('input_pcd parameter is empty')
        if not self.output_pcd:
            raise ValueError('output_pcd parameter is empty')

        self.get_logger().info('Input PCD: %s' % self.input_pcd)
        self.get_logger().info('Output PCD: %s' % self.output_pcd)

        points = read_pcd_xyz(self.input_pcd, logger=self.get_logger())
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError('PCD must contain at least x, y, z columns; got shape %s' % (points.shape,))
        points = points[:, :3].copy()

        finite_mask = np.isfinite(points).all(axis=1)
        if not np.all(finite_mask):
            dropped = int(points.shape[0] - np.count_nonzero(finite_mask))
            self.get_logger().warning('Dropping %d non-finite points.' % dropped)
            points = points[finite_mask]
        if points.shape[0] == 0:
            raise ValueError('No finite points remain after loading PCD')

        self.log_point_stats('Loaded', points)

        if as_bool(self.get_parameter('enable_manual_transform').value):
            points = self.apply_manual_transform(points)

        if as_bool(self.get_parameter('enable_auto_level').value):
            points = self.apply_auto_level(points)

        self.log_point_stats('Processed', points)
        write_pcd_xyz(self.output_pcd, points, logger=self.get_logger())
        self.get_logger().info('Saved processed PCD: %s' % self.output_pcd)

        self.points = points.astype(np.float32, copy=False)
        self.cloud_msg = self.make_cloud_msg(self.points)

    def log_point_stats(self, label, points):
        points_min = np.min(points, axis=0)
        points_max = np.max(points, axis=0)
        self.get_logger().info(
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
            )
        )

    def apply_manual_transform(self, points):
        roll = float(self.get_parameter('roll').value)
        pitch = float(self.get_parameter('pitch').value)
        yaw = float(self.get_parameter('yaw').value)
        translation = np.array([
            float(self.get_parameter('tx').value),
            float(self.get_parameter('ty').value),
            float(self.get_parameter('tz').value),
        ], dtype=np.float64)

        rotation = euler_rotation_matrix(roll, pitch, yaw)
        self.get_logger().info(
            'Applying manual transform: roll=%.6f, pitch=%.6f, yaw=%.6f, t=[%.6f, %.6f, %.6f]'
            % (roll, pitch, yaw, translation[0], translation[1], translation[2])
        )
        self.log_matrix('Manual rotation matrix Rz*Ry*Rx', rotation)
        return points @ rotation.T + translation

    def apply_auto_level(self, points):
        o3d = import_open3d()
        if o3d is None:
            raise RuntimeError(
                'enable_auto_level is true, but Open3D is not available (%s). '
                'Install Open3D or disable auto leveling and use manual transform only.'
                % OPEN3D_IMPORT_ERROR
            )

        ground_percentile = float(self.get_parameter('ground_percentile').value)
        ground_percentile = float(np.clip(ground_percentile, 0.0, 100.0))
        max_ground_points = max(3, int(self.get_parameter('max_ground_points').value))
        distance_threshold = float(self.get_parameter('ransac_distance_threshold').value)
        ransac_n = max(3, int(self.get_parameter('ransac_n').value))
        num_iterations = max(1, int(self.get_parameter('ransac_num_iterations').value))
        random_seed = int(self.get_parameter('random_seed').value)

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

        self.get_logger().info(
            'Auto level: z percentile %.3f -> %.6f, ground candidates=%d'
            % (ground_percentile, z_threshold, candidates.shape[0])
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

        target_axis = normalize_vector(self.get_parameter('normal_target_axis').value, 'normal_target_axis')
        rotation = rotation_matrix_from_vectors(normal, target_axis)
        leveled = points @ rotation.T

        self.get_logger().info(
            'RANSAC plane: %.6f*x + %.6f*y + %.6f*z + %.6f = 0, inliers=%d'
            % (a, b, c, d, len(inliers))
        )
        self.get_logger().info(
            'Plane normal after sign check: [%.9f, %.9f, %.9f], target=[%.9f, %.9f, %.9f]'
            % (normal[0], normal[1], normal[2], target_axis[0], target_axis[1], target_axis[2])
        )
        self.log_matrix('Auto-level rotation matrix', rotation)

        if as_bool(self.get_parameter('enable_ground_z_shift').value):
            target_ground_z = float(self.get_parameter('target_ground_z').value)
            reference_z = self.ground_shift_reference_z(
                leveled,
                candidate_indices,
                inliers,
            )
            leveled[:, 2] += target_ground_z - reference_z
            self.get_logger().info(
                'Shifted leveled cloud reference z from %.6f to target_ground_z %.6f'
                % (reference_z, target_ground_z)
            )

        return leveled

    def ground_shift_reference_z(self, leveled_points, candidate_indices, inliers):
        mode = str(self.get_parameter('ground_z_shift_mode').value).strip().lower()
        if mode == 'min':
            return float(np.min(leveled_points[:, 2]))

        inliers = np.asarray(inliers, dtype=np.int64)
        if inliers.size == 0:
            self.get_logger().warning('No RANSAC inliers for z shift; falling back to global min z.')
            return float(np.min(leveled_points[:, 2]))

        ground_indices = candidate_indices[inliers]
        ground_z = leveled_points[ground_indices, 2]
        if mode in ('plane_inliers_median', 'inlier_median', 'median'):
            reference_z = float(np.median(ground_z))
        elif mode in ('plane_inliers_percentile', 'inlier_percentile', 'percentile'):
            percentile = float(self.get_parameter('ground_z_shift_percentile').value)
            percentile = float(np.clip(percentile, 0.0, 100.0))
            reference_z = float(np.percentile(ground_z, percentile))
        else:
            raise ValueError(
                'Unsupported ground_z_shift_mode "%s"; use min, plane_inliers_median, '
                'or plane_inliers_percentile.' % mode
            )

        self.get_logger().info(
            'Ground z shift mode=%s, reference_z=%.6f, inlier_z=[%.6f, %.6f]'
            % (mode, reference_z, float(np.min(ground_z)), float(np.max(ground_z)))
        )
        return reference_z

    def log_matrix(self, label, matrix):
        self.get_logger().info(
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
            )
        )

    def make_cloud_msg(self, points):
        header = Header()
        header.frame_id = self.frame_id
        header.stamp = self.get_clock().now().to_msg()
        return pc2.create_cloud_xyz32(header, points[:, :3].astype(np.float32, copy=False))

    def publish_processed_cloud(self):
        if self.points is None:
            return
        self.cloud_msg = self.make_cloud_msg(self.points)
        self.publisher.publish(self.cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PcdPreprocessorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
