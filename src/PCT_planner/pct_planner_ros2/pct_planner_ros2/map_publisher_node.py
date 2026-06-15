import pickle
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header

from .config import ROSConfig, TomographyConfig
from .pcd_io import read_pcd_xyz
from .point_cloud import POINT_FIELDS_XYZI, grid_points_xyzi
from .scenes import get_plan_defaults, get_scene_config


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


class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('pct_map_publisher')

        self.declare_parameter('scene', 'Spiral')
        self.declare_parameter('pcd_dir', TomographyConfig.pcd_dir)
        self.declare_parameter('pcd_file', '')
        self.declare_parameter('tomogram_dir', TomographyConfig.tomogram_dir)
        self.declare_parameter('tomogram_file', '')
        self.declare_parameter('map_frame', ROSConfig.map_frame)
        self.declare_parameter('pointcloud_topic', ROSConfig.pointcloud_topic)
        self.declare_parameter('tomogram_topic', ROSConfig.tomogram_topic)
        self.declare_parameter('publish_raw_cloud', True)
        self.declare_parameter('publish_tomogram_cloud', True)

        self.scene_name = self.get_parameter('scene').value
        self.scene_cfg = get_scene_config(self.scene_name)
        self.plan_defaults = get_plan_defaults(self.scene_name)

        self.pcd_dir = Path(self.get_parameter('pcd_dir').value).expanduser()
        self.tomogram_dir = Path(self.get_parameter('tomogram_dir').value).expanduser()
        self.map_frame = self.get_parameter('map_frame').value
        self.publish_raw_cloud = as_bool(self.get_parameter('publish_raw_cloud').value)
        self.publish_tomogram_cloud = as_bool(self.get_parameter('publish_tomogram_cloud').value)

        pcd_file = self.get_parameter('pcd_file').value
        self.pcd_file = pcd_file if pcd_file else self.scene_cfg.pcd.file_name

        tomogram_file = self.get_parameter('tomogram_file').value
        self.tomogram_file = tomogram_file if tomogram_file else self.plan_defaults.tomogram_file

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('pointcloud_topic').value,
            latched_qos(),
        )
        self.tomogram_pub = self.create_publisher(
            PointCloud2,
            self.get_parameter('tomogram_topic').value,
            latched_qos(),
        )

        self.publish_maps()

    def make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        return header

    def publish_maps(self):
        if self.publish_raw_cloud:
            self.publish_pcd()
        if self.publish_tomogram_cloud:
            self.publish_tomogram()

    def publish_pcd(self):
        pcd_path = self.pcd_dir / self.pcd_file
        if not pcd_path.exists():
            self.get_logger().warning(f'PCD file not found, cannot publish raw map: {pcd_path}')
            return

        self.get_logger().info(f'Loading PCD map: {pcd_path}')
        points = np.asarray(read_pcd_xyz(str(pcd_path), logger=self.get_logger()), dtype=np.float32)
        if points.ndim != 2 or points.shape[1] < 3:
            self.get_logger().error(f'PCD must contain at least 3 columns, got shape {points.shape}')
            return
        points = points[:, :3]

        msg = pc2.create_cloud_xyz32(self.make_header(), points)
        self.pointcloud_pub.publish(msg)
        self.get_logger().info(f'Published raw point cloud on "{self.pointcloud_pub.topic_name}": {points.shape[0]} points')

    def publish_tomogram(self):
        tomogram_path = self.tomogram_dir / f'{self.tomogram_file}.pickle'
        if not tomogram_path.exists():
            self.get_logger().warning(f'Tomogram file not found, cannot publish tomogram map: {tomogram_path}')
            return

        self.get_logger().info(f'Loading tomogram map: {tomogram_path}')
        with open(tomogram_path, 'rb') as handle:
            data_dict = pickle.load(handle)

        tomogram = np.asarray(data_dict['data'], dtype=np.float32)
        if tomogram.ndim != 4 or tomogram.shape[0] < 5:
            self.get_logger().error(f'Unexpected tomogram shape: {tomogram.shape}')
            return

        resolution = float(data_dict['resolution'])
        center = np.asarray(data_dict['center'], dtype=np.float32)
        slice_dh = float(data_dict['slice_dh'])
        layers_t = tomogram[0]
        layers_g = tomogram[3]
        map_dim_x = layers_g.shape[1]
        map_dim_y = layers_g.shape[2]

        visproto_i, visproto_p = grid_points_xyzi(resolution, map_dim_x, map_dim_y)
        layer_points = visproto_p.copy()
        layer_points[:, :2] += center

        vis_g = layers_g.copy()
        vis_t = layers_t.copy()
        chunks = []
        n_slice = layers_g.shape[0]
        for i in range(n_slice - 1):
            mask_h = (vis_g[i + 1] - vis_g[i]) < slice_dh
            vis_g[i, mask_h] = np.nan
            vis_t[i + 1, mask_h] = np.minimum(vis_t[i, mask_h], vis_t[i + 1, mask_h])
            layer_points[:, 2] = vis_g[i, visproto_i[:, 0], visproto_i[:, 1]]
            layer_points[:, 3] = vis_t[i, visproto_i[:, 0], visproto_i[:, 1]]
            valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
            chunks.append(valid_points.copy())

        layer_points[:, 2] = vis_g[-1, visproto_i[:, 0], visproto_i[:, 1]]
        layer_points[:, 3] = vis_t[-1, visproto_i[:, 0], visproto_i[:, 1]]
        valid_points = layer_points[~np.isnan(layer_points).any(axis=-1)]
        chunks.append(valid_points)

        global_points = np.concatenate(chunks, axis=0) if chunks else np.empty((0, 4), dtype=np.float32)
        msg = pc2.create_cloud(self.make_header(), POINT_FIELDS_XYZI, global_points)
        self.tomogram_pub.publish(msg)
        self.get_logger().info(
            f'Published tomogram point cloud on "{self.tomogram_pub.topic_name}": {global_points.shape[0]} points'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MapPublisherNode()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
