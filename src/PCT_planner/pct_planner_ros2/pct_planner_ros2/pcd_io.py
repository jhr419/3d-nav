import os
from pathlib import Path

import numpy as np


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

        line = line.decode('utf-8', errors='replace').strip()
        if not line or line.startswith('#'):
            continue

        tokens = line.split()
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


def _read_pcd_xyz_fallback(pcd_path):
    with open(pcd_path, 'rb') as pcd_file:
        header = _read_pcd_header(pcd_file)

        fields = header.get('FIELDS')
        sizes = [int(value) for value in header.get('SIZE', [])]
        types = header.get('TYPE')
        counts = [int(value) for value in header.get('COUNT', [])]

        if not fields or not sizes or not types:
            raise ValueError('Invalid PCD file: missing FIELDS, SIZE, or TYPE')
        if not counts:
            counts = [1] * len(fields)
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError('Invalid PCD file: inconsistent field metadata')

        data_kind = header['DATA'][0].lower()
        point_count = _pcd_points_count(header)

        if data_kind == 'ascii':
            columns = _pcd_xyz_columns(fields, counts)
            points = np.loadtxt(pcd_file, dtype=np.float32, usecols=columns)
            return np.atleast_2d(points).astype(np.float32, copy=False)

        if data_kind != 'binary':
            raise NotImplementedError(
                'Only ASCII and uncompressed binary PCD files are supported '
                'without Open3D'
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

        return np.column_stack(xyz).astype(np.float32, copy=False)


def read_pcd_xyz(pcd_path, logger=None):
    if not os.path.isfile(pcd_path):
        raise FileNotFoundError('PCD file not found: %s' % pcd_path)

    try:
        import open3d as o3d

        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points).astype(np.float32)
        if points.size == 0:
            raise ValueError('Open3D returned an empty point cloud')
        return points
    except Exception as exc:
        _log_warning(logger, f'Open3D PCD reader unavailable ({exc}); using internal PCD reader.')
        return _read_pcd_xyz_fallback(pcd_path)


def _write_pcd_xyz_fallback(pcd_path, points):
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
    try:
        import open3d as o3d

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64)[:, :3])
        ok = o3d.io.write_point_cloud(str(pcd_path), pcd)
        if not ok:
            raise RuntimeError('Open3D failed to write PCD: %s' % pcd_path)
    except Exception as exc:
        _log_warning(logger, f'Open3D PCD writer unavailable ({exc}); using internal ASCII PCD writer.')
        _write_pcd_xyz_fallback(pcd_path, points)
