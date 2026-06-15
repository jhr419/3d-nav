#!/usr/bin/python3
import os
import sys
import time
import pickle
import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from tomogram import Tomogram

sys.path.append('../')
from config import POINT_FIELDS_XYZI, GRID_POINTS_XYZI
from config import Config

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


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

    raise ValueError("Unsupported PCD field type: %s%d" % (type_name, size))


def _read_pcd_header(pcd_file):
    header = {}
    while True:
        line = pcd_file.readline()
        if not line:
            raise ValueError("Invalid PCD file: missing DATA header")

        line = line.decode('utf-8', errors='replace').strip()
        if not line or line.startswith('#'):
            continue

        tokens = line.split()
        key = tokens[0].upper()
        header[key] = tokens[1:]
        if key == 'DATA':
            if len(tokens) < 2:
                raise ValueError("Invalid PCD file: DATA header is empty")
            return header


def _pcd_points_count(header):
    if 'POINTS' in header:
        return int(header['POINTS'][0])
    if 'WIDTH' in header and 'HEIGHT' in header:
        return int(header['WIDTH'][0]) * int(header['HEIGHT'][0])
    raise ValueError("Invalid PCD file: missing POINTS or WIDTH/HEIGHT")


def _pcd_xyz_columns(fields, counts):
    offsets = {}
    column = 0
    for field, count in zip(fields, counts):
        offsets.setdefault(field, column)
        column += count

    try:
        return [offsets['x'], offsets['y'], offsets['z']]
    except KeyError as exc:
        raise ValueError("PCD file must contain x, y, and z fields") from exc


def _read_pcd_xyz_fallback(pcd_path):
    with open(pcd_path, 'rb') as pcd_file:
        header = _read_pcd_header(pcd_file)

        fields = header.get('FIELDS')
        sizes = [int(value) for value in header.get('SIZE', [])]
        types = header.get('TYPE')
        counts = [int(value) for value in header.get('COUNT', [])]

        if not fields or not sizes or not types:
            raise ValueError("Invalid PCD file: missing FIELDS, SIZE, or TYPE")
        if not counts:
            counts = [1] * len(fields)
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("Invalid PCD file: inconsistent field metadata")

        data_kind = header['DATA'][0].lower()
        point_count = _pcd_points_count(header)

        if data_kind == 'ascii':
            columns = _pcd_xyz_columns(fields, counts)
            points = np.loadtxt(pcd_file, dtype=np.float32, usecols=columns)
            return np.atleast_2d(points).astype(np.float32, copy=False)

        if data_kind != 'binary':
            raise NotImplementedError(
                "Only ASCII and uncompressed binary PCD files are supported "
                "without Open3D"
            )

        dtype_fields = []
        dtype_names = {}
        for index, (field, size, type_name, count) in enumerate(zip(fields, sizes, types, counts)):
            dtype_name = field if field not in dtype_names.values() else "%s_%d" % (field, index)
            dtype_names.setdefault(field, dtype_name)
            dtype = _pcd_dtype(type_name, size)
            if count == 1:
                dtype_fields.append((dtype_name, dtype))
            else:
                dtype_fields.append((dtype_name, dtype, (count,)))

        point_dtype = np.dtype(dtype_fields)
        data = np.fromfile(pcd_file, dtype=point_dtype, count=point_count)
        if data.shape[0] != point_count:
            raise ValueError("Invalid PCD file: expected %d points, read %d" % (point_count, data.shape[0]))

        xyz = []
        for field in ('x', 'y', 'z'):
            if field not in dtype_names:
                raise ValueError("PCD file must contain x, y, and z fields")
            values = data[dtype_names[field]]
            if values.ndim > 1:
                values = values[:, 0]
            xyz.append(values)

        return np.column_stack(xyz).astype(np.float32, copy=False)


def read_pcd_xyz(pcd_path):
    if not os.path.isfile(pcd_path):
        raise FileNotFoundError("PCD file not found: %s" % pcd_path)

    try:
        import open3d as o3d

        pcd = o3d.io.read_point_cloud(pcd_path)
        points = np.asarray(pcd.points).astype(np.float32)
        if points.size == 0:
            raise ValueError("Open3D returned an empty point cloud")
        return points
    except Exception as exc:
        rospy.logwarn("Open3D PCD reader unavailable (%s); using internal PCD reader.", exc)
        return _read_pcd_xyz_fallback(pcd_path)


class Tomography(object):
    def __init__(self, cfg, scene_cfg):
        self.export_dir = rsg_root + cfg.map.export_dir
        self.pcd_file = scene_cfg.pcd.file_name
        self.resolution = scene_cfg.map.resolution
        self.ground_h = scene_cfg.map.ground_h
        self.slice_dh = scene_cfg.map.slice_dh

        self.center = np.zeros(2, dtype=np.float32)
        self.tomogram = Tomogram(scene_cfg)
        points = self.loadPCD(self.pcd_file)

        # Process
        self.process(points)

    def initROS(self):
        self.map_frame = cfg.ros.map_frame

        pointcloud_topic = cfg.ros.pointcloud_topic
        self.pointcloud_pub = rospy.Publisher(pointcloud_topic, PointCloud2, latch=True, queue_size=1)

        self.layer_G_pub_list = []
        self.layer_C_pub_list = []
        layer_G_topic = cfg.ros.layer_G_topic
        layer_C_topic = cfg.ros.layer_C_topic
        for i in range(self.n_slice):
            layer_G_pub = rospy.Publisher(layer_G_topic + str(i), PointCloud2, latch=True, queue_size=1)
            self.layer_G_pub_list.append(layer_G_pub)
            layer_C_pub = rospy.Publisher(layer_C_topic + str(i), PointCloud2, latch=True, queue_size=1)
            self.layer_C_pub_list.append(layer_C_pub)

        tomogram_topic = cfg.ros.tomogram_topic
        self.tomogram_pub = rospy.Publisher(tomogram_topic, PointCloud2, latch=True, queue_size=1)

    def loadPCD(self, pcd_file):
        points = read_pcd_xyz(rsg_root + "/rsc/pcd/" + pcd_file)
        rospy.loginfo("PCD points: %d", points.shape[0])

        if points.shape[1] > 3:
            points = points[:, :3]
        self.points_max = np.max(points, axis=0)
        self.points_min = np.min(points, axis=0)           
        self.points_min[-1] = self.ground_h
        self.map_dim_x = int(np.ceil((self.points_max[0] - self.points_min[0]) / self.resolution)) + 4
        self.map_dim_y = int(np.ceil((self.points_max[1] - self.points_min[1]) / self.resolution)) + 4
        n_slice_init = int(np.ceil((self.points_max[2] - self.points_min[2]) / self.slice_dh))
        self.center = (self.points_max[:2] + self.points_min[:2]) / 2
        self.slice_h0 = self.points_min[-1] + self.slice_dh
        self.tomogram.initMappingEnv(self.center, self.map_dim_x, self.map_dim_y, n_slice_init, self.slice_h0)

        rospy.loginfo("Map center: [%.2f, %.2f]", self.center[0], self.center[1])
        rospy.loginfo("Dim_x: %d", self.map_dim_x)
        rospy.loginfo("Dim_y: %d", self.map_dim_y)
        rospy.loginfo("Num slices init: %d", n_slice_init)

        self.VISPROTO_I, self.VISPROTO_P = \
            GRID_POINTS_XYZI(self.resolution, self.map_dim_x, self.map_dim_y)

        return points
        
    def process(self, points):        
        t_map = 0.0
        t_trav = 0.0
        t_simp = 0.0
        t_all = 0.0
        n_repeat = 10

        """ 
        GPU time benchmark, where CUDA events are synchronized for correct time measurement.
        The function is repeatedly run for n_repeat times to calculate the average processing time of each modules.
        The time of the first warm-up run is excluded to reduce timing fluctuation and exclude the overhead in initial invocations.
        See https://docs.cupy.dev/en/stable/user_guide/performance.html for more details
        """
        for i in range(n_repeat + 1):
            t_start = time.time()
            layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_gpu = self.tomogram.point2map(points)

            if i > 0:
                t_map += t_gpu['t_map']
                t_trav += t_gpu['t_trav']
                t_simp += t_gpu['t_simp']
                t_all += (time.time() - t_start) * 1e3

        rospy.loginfo("Num slices simp: %d", layers_g.shape[0])
        rospy.loginfo("Num repeats (for benchmarking only): %d", n_repeat)
        rospy.loginfo(" -- avg t_map  (ms): %f", t_map / n_repeat)
        rospy.loginfo(" -- avg t_trav (ms): %f", t_trav / n_repeat)
        rospy.loginfo(" -- avg t_simp (ms): %f", t_simp / n_repeat)
        rospy.loginfo(" -- avg t_all  (ms): %f", t_all / n_repeat)

        self.n_slice = layers_g.shape[0]

        map_file = os.path.splitext(self.pcd_file)[0]
        self.exportTomogram(np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)), map_file)

        self.initROS()
        self.publishPoints(points)
        self.publishLayers(self.layer_G_pub_list, layers_g, layers_t)
        self.publishLayers(self.layer_C_pub_list, layers_c, None)
        self.publishTomogram(layers_g, layers_t)

    def exportTomogram(self, tomogram, map_file):        
        data_dict = {
            'data': tomogram.astype(np.float16),
            'resolution': self.resolution,
            'center': self.center,
            'slice_h0': self.slice_h0,
            'slice_dh': self.slice_dh,
        }
        file_name = map_file + '.pickle'
        with open(self.export_dir + file_name, 'wb') as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

        rospy.loginfo("Tomogram exported: %s", file_name)

    def publishPoints(self, points):
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        point_msg = pc2.create_cloud_xyz32(header, points)
        self.pointcloud_pub.publish(point_msg)

    def publishLayers(self, pub_list, layers, color=None):
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        for i in range(layers.shape[0]):
            layer_points[:, 2] = layers[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            if color is not None:
                layer_points[:, 3] = color[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            else:
                layer_points[:, 3] = 1.0
        
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, valid_points)
            pub_list[i].publish(points_msg) 

    def publishTomogram(self, layers_g, layers_t):
        header = Header()
        header.seq = 0
        header.stamp = rospy.Time.now()
        header.frame_id = self.map_frame

        n_slice = layers_g.shape[0]
        vis_g = layers_g.copy()
        vis_t = layers_t.copy() 
        layer_points = self.VISPROTO_P.copy()
        layer_points[:, :2] += self.center

        global_points = None
        for i in range(n_slice - 1):
            mask_h = (vis_g[i + 1] - vis_g[i]) < self.slice_dh
            vis_g[i, mask_h] = np.nan
            vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])
            layer_points[:, 2] = vis_g[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            layer_points[:, 3] = vis_t[i, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            if global_points is None:
                global_points = valid_points
            else:
                global_points = np.concatenate((global_points, valid_points), axis=0)

        layer_points[:, 2] = vis_g[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        layer_points[:, 3] = vis_t[-1, self.VISPROTO_I[:, 0], self.VISPROTO_I[:, 1]]
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
        global_points = np.concatenate((global_points, valid_points), axis=0)
        
        points_msg = pc2.create_cloud(header, POINT_FIELDS_XYZI, global_points)
        self.tomogram_pub.publish(points_msg)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('--scene', type=str, help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
    args = parser.parse_args()

    cfg = Config()
    scene_cfg = getattr(__import__('config'), 'Scene' + args.scene)

    rospy.init_node('pointcloud_tomography', anonymous=True)

    mapping = Tomography(cfg, scene_cfg)

    rospy.spin()
