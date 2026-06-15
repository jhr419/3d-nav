import pickle
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from .config import ROSConfig, TomographyConfig
from .pcd_io import read_pcd_xyz
from .point_cloud import POINT_FIELDS_XYZI, grid_points_xyzi
from .scenes import get_scene_config
from .tomogram import Tomogram


def latched_qos():
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


class TomographyNode(Node):
    def __init__(self):
        super().__init__('pct_tomography')

        self.declare_parameter('scene', 'Spiral')
        self.declare_parameter('pcd_dir', TomographyConfig.pcd_dir)
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('tomogram_dir', TomographyConfig.tomogram_dir)
        self.declare_parameter('benchmark_repeats', TomographyConfig.benchmark_repeats)
        self.declare_parameter('map_frame', ROSConfig.map_frame)
        self.declare_parameter('pointcloud_topic', ROSConfig.pointcloud_topic)
        self.declare_parameter('layer_g_topic_prefix', ROSConfig.layer_g_topic_prefix)
        self.declare_parameter('layer_c_topic_prefix', ROSConfig.layer_c_topic_prefix)
        self.declare_parameter('tomogram_topic', ROSConfig.tomogram_topic)
        self.declare_parameter('resolution', -1.0)
        self.declare_parameter('ground_h', float('nan'))
        self.declare_parameter('slice_dh', -1.0)
        self.declare_parameter('auto_run', True)

        self.scene_name = self.get_parameter('scene').value
        self.scene_cfg = get_scene_config(self.scene_name)

        pcd_file = self.get_parameter('pcd_file').value
        if pcd_file:
            self.scene_cfg.pcd.file_name = pcd_file

        resolution = float(self.get_parameter('resolution').value)
        if resolution > 0.0:
            self.scene_cfg.map.resolution = resolution

        ground_h = float(self.get_parameter('ground_h').value)
        if not np.isnan(ground_h):
            self.scene_cfg.map.ground_h = ground_h

        slice_dh = float(self.get_parameter('slice_dh').value)
        if slice_dh > 0.0:
            self.scene_cfg.map.slice_dh = slice_dh

        self.pcd_dir = Path(self.get_parameter('pcd_dir').value).expanduser()
        self.tomogram_dir = Path(self.get_parameter('tomogram_dir').value).expanduser()
        self.benchmark_repeats = max(1, int(self.get_parameter('benchmark_repeats').value))
        self.map_frame = self.get_parameter('map_frame').value
        self.pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.layer_g_topic_prefix = self.get_parameter('layer_g_topic_prefix').value
        self.layer_c_topic_prefix = self.get_parameter('layer_c_topic_prefix').value
        self.tomogram_topic = self.get_parameter('tomogram_topic').value
        self.qos = latched_qos()

        self.pcd_file = self.scene_cfg.pcd.file_name
        self.resolution = self.scene_cfg.map.resolution
        self.ground_h = self.scene_cfg.map.ground_h
        self.slice_dh = self.scene_cfg.map.slice_dh

        self.center = np.zeros(2, dtype=np.float32)
        self.tomogram = Tomogram(self.scene_cfg)

        if bool(self.get_parameter('auto_run').value):
            self.run_once()

    def run_once(self):
        pcd_path = self.pcd_dir / self.pcd_file
        self.get_logger().info(f'Loading PCD: {pcd_path}')
        points = self.load_pcd(pcd_path)
        self.process(points)

    def init_ros_publishers(self):
        self.pointcloud_pub = self.create_publisher(PointCloud2, self.pointcloud_topic, self.qos)

        self.layer_g_pub_list = []
        self.layer_c_pub_list = []
        for i in range(self.n_slice):
            layer_g_pub = self.create_publisher(PointCloud2, f'{self.layer_g_topic_prefix}{i}', self.qos)
            self.layer_g_pub_list.append(layer_g_pub)
            layer_c_pub = self.create_publisher(PointCloud2, f'{self.layer_c_topic_prefix}{i}', self.qos)
            self.layer_c_pub_list.append(layer_c_pub)

        self.tomogram_pub = self.create_publisher(PointCloud2, self.tomogram_topic, self.qos)

    def load_pcd(self, pcd_path):
        points = read_pcd_xyz(str(pcd_path), logger=self.get_logger())
        points = np.asarray(points, dtype=np.float32)
        self.get_logger().info(f'PCD points: {points.shape[0]}')

        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError(f'PCD must contain at least 3 columns, got shape {points.shape}')
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
        self.tomogram.init_mapping_env(
            self.center,
            self.map_dim_x,
            self.map_dim_y,
            n_slice_init,
            self.slice_h0,
        )

        self.get_logger().info(f'Map center: [{self.center[0]:.2f}, {self.center[1]:.2f}]')
        self.get_logger().info(f'Dim_x: {self.map_dim_x}')
        self.get_logger().info(f'Dim_y: {self.map_dim_y}')
        self.get_logger().info(f'Num slices init: {n_slice_init}')

        self.visproto_i, self.visproto_p = grid_points_xyzi(self.resolution, self.map_dim_x, self.map_dim_y)

        return points

    def process(self, points):
        t_map = 0.0
        t_trav = 0.0
        t_simp = 0.0
        t_all = 0.0

        for i in range(self.benchmark_repeats + 1):
            t_start = time.time()
            layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c, t_gpu = self.tomogram.point_to_map(points)

            if i > 0:
                t_map += t_gpu['t_map']
                t_trav += t_gpu['t_trav']
                t_simp += t_gpu['t_simp']
                t_all += (time.time() - t_start) * 1e3

        self.get_logger().info(f'Num slices simp: {layers_g.shape[0]}')
        self.get_logger().info(f'Num repeats (for benchmarking only): {self.benchmark_repeats}')
        self.get_logger().info(f' -- avg t_map  (ms): {t_map / self.benchmark_repeats:f}')
        self.get_logger().info(f' -- avg t_trav (ms): {t_trav / self.benchmark_repeats:f}')
        self.get_logger().info(f' -- avg t_simp (ms): {t_simp / self.benchmark_repeats:f}')
        self.get_logger().info(f' -- avg t_all  (ms): {t_all / self.benchmark_repeats:f}')

        self.n_slice = layers_g.shape[0]

        map_file = Path(self.pcd_file).stem
        self.export_tomogram(
            np.stack((layers_t, trav_grad_x, trav_grad_y, layers_g, layers_c)),
            map_file,
        )

        self.init_ros_publishers()
        self.publish_points(points)
        self.publish_layers(self.layer_g_pub_list, layers_g, layers_t)
        self.publish_layers(self.layer_c_pub_list, layers_c, None)
        self.publish_tomogram(layers_g, layers_t)

    def export_tomogram(self, tomogram, map_file):
        self.tomogram_dir.mkdir(parents=True, exist_ok=True)
        data_dict = {
            'data': tomogram.astype(np.float16),
            'resolution': self.resolution,
            'center': self.center,
            'slice_h0': self.slice_h0,
            'slice_dh': self.slice_dh,
        }
        file_path = self.tomogram_dir / f'{map_file}.pickle'
        with open(file_path, 'wb') as handle:
            pickle.dump(data_dict, handle, protocol=pickle.HIGHEST_PROTOCOL)

        self.get_logger().info(f'Tomogram exported: {file_path}')

    def make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        return header

    def publish_points(self, points):
        point_msg = pc2.create_cloud_xyz32(self.make_header(), points)
        self.pointcloud_pub.publish(point_msg)

    def publish_layers(self, pub_list, layers, color=None):
        layer_points = self.visproto_p.copy()
        layer_points[:, :2] += self.center

        for i in range(layers.shape[0]):
            layer_points[:, 2] = layers[i, self.visproto_i[:, 0], self.visproto_i[:, 1]]
            if color is not None:
                layer_points[:, 3] = color[i, self.visproto_i[:, 0], self.visproto_i[:, 1]]
            else:
                layer_points[:, 3] = 1.0

            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            points_msg = pc2.create_cloud(self.make_header(), POINT_FIELDS_XYZI, valid_points)
            pub_list[i].publish(points_msg)

    def publish_tomogram(self, layers_g, layers_t):
        n_slice = layers_g.shape[0]
        vis_g = layers_g.copy()
        vis_t = layers_t.copy()
        layer_points = self.visproto_p.copy()
        layer_points[:, :2] += self.center

        chunks = []
        for i in range(n_slice - 1):
            mask_h = (vis_g[i + 1] - vis_g[i]) < self.slice_dh
            vis_g[i, mask_h] = np.nan
            vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])
            layer_points[:, 2] = vis_g[i, self.visproto_i[:, 0], self.visproto_i[:, 1]]
            layer_points[:, 3] = vis_t[i, self.visproto_i[:, 0], self.visproto_i[:, 1]]
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            chunks.append(valid_points.copy())

        layer_points[:, 2] = vis_g[-1, self.visproto_i[:, 0], self.visproto_i[:, 1]]
        layer_points[:, 3] = vis_t[-1, self.visproto_i[:, 0], self.visproto_i[:, 1]]
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
        chunks.append(valid_points)

        global_points = np.concatenate(chunks, axis=0) if chunks else np.empty((0, 4), dtype=np.float32)
        points_msg = pc2.create_cloud(self.make_header(), POINT_FIELDS_XYZI, global_points)
        self.tomogram_pub.publish(points_msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TomographyNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
