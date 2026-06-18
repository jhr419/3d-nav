import os
import pickle
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header


POINT_FIELDS_XYZI = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
]


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def _pcd_dtype(type_name: str, size: int) -> str:
    if type_name == "F":
        if size == 4:
            return "<f4"
        if size == 8:
            return "<f8"
    if type_name == "I" and size in (1, 2, 4, 8):
        return f"<i{size}"
    if type_name == "U" and size in (1, 2, 4, 8):
        return f"<u{size}"
    raise ValueError(f"Unsupported PCD field type: {type_name}{size}")


def _read_pcd_header(pcd_file):
    header = {}
    while True:
        line = pcd_file.readline()
        if not line:
            raise ValueError("Invalid PCD file: missing DATA header")
        line = line.decode("utf-8", errors="replace").strip()
        if not line or line.startswith("#"):
            continue
        tokens = line.split()
        key = tokens[0].upper()
        header[key] = tokens[1:]
        if key == "DATA":
            return header


def _pcd_points_count(header) -> int:
    if "POINTS" in header:
        return int(header["POINTS"][0])
    if "WIDTH" in header and "HEIGHT" in header:
        return int(header["WIDTH"][0]) * int(header["HEIGHT"][0])
    raise ValueError("Invalid PCD file: missing POINTS or WIDTH/HEIGHT")


def _pcd_xyz_columns(fields, counts):
    offsets = {}
    column = 0
    for field, count in zip(fields, counts):
        offsets.setdefault(field, column)
        column += count
    return [offsets["x"], offsets["y"], offsets["z"]]


def read_pcd_xyz(pcd_path: str) -> np.ndarray:
    if not os.path.isfile(pcd_path):
        raise FileNotFoundError(f"PCD file not found: {pcd_path}")

    with open(pcd_path, "rb") as pcd_file:
        header = _read_pcd_header(pcd_file)
        fields = header.get("FIELDS")
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE")
        counts = [int(value) for value in header.get("COUNT", [])] or [1] * len(fields or [])
        if not fields or not sizes or not types:
            raise ValueError("Invalid PCD file: missing FIELDS, SIZE, or TYPE")
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError("Invalid PCD file: inconsistent field metadata")
        if not {"x", "y", "z"}.issubset(set(fields)):
            raise ValueError("PCD file must contain x, y, and z fields")

        data_kind = header["DATA"][0].lower()
        point_count = _pcd_points_count(header)

        if data_kind == "ascii":
            points = np.loadtxt(pcd_file, dtype=np.float32, usecols=_pcd_xyz_columns(fields, counts))
            return np.atleast_2d(points).astype(np.float32, copy=False)

        if data_kind != "binary":
            raise NotImplementedError("Only ASCII and uncompressed binary PCD files are supported")

        dtype_fields = []
        dtype_names = {}
        for index, (field, size, type_name, count) in enumerate(zip(fields, sizes, types, counts)):
            dtype_name = field if field not in dtype_names.values() else f"{field}_{index}"
            dtype_names.setdefault(field, dtype_name)
            dtype = _pcd_dtype(type_name, size)
            dtype_fields.append((dtype_name, dtype) if count == 1 else (dtype_name, dtype, (count,)))

        data = np.fromfile(pcd_file, dtype=np.dtype(dtype_fields), count=point_count)
        xyz = []
        for field in ("x", "y", "z"):
            values = data[dtype_names[field]]
            if values.ndim > 1:
                values = values[:, 0]
            xyz.append(values)
        return np.column_stack(xyz).astype(np.float32, copy=False)


def grid_points_xyzi(resolution: float, dim_x: int, dim_y: int):
    index_proto = np.zeros((dim_x * dim_y, 2), dtype=int)
    lx = np.linspace(0, dim_x - 1, dim_x, dtype=int)
    ly = np.linspace(0, dim_y - 1, dim_y, dtype=int)
    ix, iy = np.meshgrid(lx, ly)
    index_proto[:, 0] = ix.flatten()
    index_proto[:, 1] = iy.flatten()

    point_proto = np.zeros((dim_x * dim_y, 4), dtype=np.float32)
    point_proto[:, :2] = index_proto[:, :2].astype(np.float32, copy=True)
    point_proto[:, 0] -= 0.5 * dim_x
    point_proto[:, 1] -= 0.5 * dim_y
    point_proto[:, :2] *= resolution
    point_proto[:, 3] = 1.0
    return index_proto, point_proto


class PCTGlobalMapPublisher(Node):
    def __init__(self):
        super().__init__("pct_global_map_publisher")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("pcd_dir", "maps")
        self.declare_parameter("pcd_file", "map_preprocessed.pcd")
        self.declare_parameter("tomogram_dir", "maps/tomogram")
        self.declare_parameter("tomogram_file", "map_preprocessed")
        self.declare_parameter("pointcloud_topic", "/global_points")
        self.declare_parameter("tomogram_topic", "/tomogram")
        self.declare_parameter("publish_raw_cloud", True)
        self.declare_parameter("publish_tomogram_cloud", True)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.pcd_dir = Path(str(self.get_parameter("pcd_dir").value)).expanduser()
        self.pcd_file = str(self.get_parameter("pcd_file").value)
        self.tomogram_dir = Path(str(self.get_parameter("tomogram_dir").value)).expanduser()
        self.tomogram_file = str(self.get_parameter("tomogram_file").value)
        self.publish_raw_cloud = as_bool(self.get_parameter("publish_raw_cloud").value)
        self.publish_tomogram_cloud = as_bool(self.get_parameter("publish_tomogram_cloud").value)

        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("pointcloud_topic").value),
            latched_qos(),
        )
        self.tomogram_pub = self.create_publisher(
            PointCloud2,
            str(self.get_parameter("tomogram_topic").value),
            latched_qos(),
        )

        self.publish_maps()

    def make_header(self) -> Header:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame
        return header

    def resolve_file(self, base_dir: Path, file_name: str, suffix: str = "") -> Path:
        path = Path(os.path.expanduser(file_name))
        if path.is_absolute():
            return path
        if suffix and not path.name.endswith(suffix):
            path = path.with_name(path.name + suffix)
        return base_dir / path

    def publish_maps(self) -> None:
        if self.publish_raw_cloud:
            self.publish_pcd()
        if self.publish_tomogram_cloud:
            self.publish_tomogram()

    def publish_pcd(self) -> None:
        pcd_path = self.resolve_file(self.pcd_dir, self.pcd_file)
        if not pcd_path.exists():
            self.get_logger().warning(f"PCD file not found, cannot publish raw map: {pcd_path}")
            return
        self.get_logger().info(f"Loading PCD map: {pcd_path}")
        points = read_pcd_xyz(str(pcd_path))[:, :3]
        msg = pc2.create_cloud_xyz32(self.make_header(), points)
        self.pointcloud_pub.publish(msg)
        self.get_logger().info(
            f'Published raw point cloud on "{self.pointcloud_pub.topic_name}": {points.shape[0]} points'
        )

    def publish_tomogram(self) -> None:
        tomogram_path = self.resolve_file(self.tomogram_dir, self.tomogram_file, ".pickle")
        if not tomogram_path.exists():
            self.get_logger().warning(f"Tomogram file not found, cannot publish tomogram map: {tomogram_path}")
            return

        self.get_logger().info(f"Loading tomogram map: {tomogram_path}")
        with open(tomogram_path, "rb") as handle:
            data_dict = pickle.load(handle)

        tomogram = np.asarray(data_dict["data"], dtype=np.float32)
        if tomogram.ndim != 4 or tomogram.shape[0] < 5:
            self.get_logger().error(f"Unexpected tomogram shape: {tomogram.shape}")
            return

        resolution = float(data_dict["resolution"])
        center = np.asarray(data_dict["center"], dtype=np.float32)
        slice_dh = float(data_dict["slice_dh"])
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
        for i in range(layers_g.shape[0] - 1):
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
        node = PCTGlobalMapPublisher()
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
