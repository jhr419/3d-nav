#!/usr/bin/env python3

import queue
import threading
import time
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

try:
    import numpy as np
except ImportError:  # pragma: no cover - runtime environment dependent
    np = None

try:
    import open3d as o3d
except ImportError:  # pragma: no cover - runtime environment dependent
    o3d = None


def point_cloud_to_xyz_array(msg: PointCloud2):
    points = point_cloud2.read_points(
        msg, field_names=("x", "y", "z"), skip_nans=True
    )

    if np is None:
        return None

    if isinstance(points, np.ndarray):
        if points.dtype.names:
            return np.column_stack(
                (
                    points["x"].astype(np.float64, copy=False),
                    points["y"].astype(np.float64, copy=False),
                    points["z"].astype(np.float64, copy=False),
                )
            )
        array = np.asarray(points, dtype=np.float64)
        if array.ndim == 1:
            return array.reshape((-1, 3))
        return array[:, :3]

    rows = []
    for point in points:
        if hasattr(point, "x"):
            rows.append((point.x, point.y, point.z))
        elif isinstance(point, dict):
            rows.append((point["x"], point["y"], point["z"]))
        else:
            rows.append((point[0], point[1], point[2]))
    if not rows:
        return np.empty((0, 3), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


class Open3DGoalPicker(Node):
    def __init__(self) -> None:
        super().__init__("open3d_goal_picker")

        self.declare_parameter("cloud_topic", "/octomap_points")
        self.declare_parameter("path_topic", "/global_path_3d")
        self.declare_parameter("initial_pose_topic", "/initial_pose_3d")
        self.declare_parameter("goal_pose_topic", "/goal_pose_3d")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("point_size", 3.0)

        self._cloud_points = None
        self._path_points = None
        self._cloud_dirty = False
        self._path_dirty = False
        self._commands: "queue.Queue[str]" = queue.Queue()
        self._next_is_start = True

        cloud_topic = self.get_parameter("cloud_topic").value
        path_topic = self.get_parameter("path_topic").value
        initial_pose_topic = self.get_parameter("initial_pose_topic").value
        goal_pose_topic = self.get_parameter("goal_pose_topic").value

        self.initial_pose_pub = self.create_publisher(PoseStamped, initial_pose_topic, 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, goal_pose_topic, 10)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud, 10)
        self.create_subscription(Path, path_topic, self._on_path, 10)

        threading.Thread(target=self._read_console, daemon=True).start()

        self.get_logger().info(
            "Open3D goal picker started. Commands: 'start x y z', 'goal x y z', "
            "or plain 'x y z' to alternate start/goal."
        )
        if o3d is None:
            self.get_logger().warn("Open3D is not installed. Running in terminal-only mode.")
        if np is None:
            self.get_logger().warn("numpy is not installed. Point cloud visualization is disabled.")

    def _read_console(self) -> None:
        while True:
            try:
                command = input("3D picker> ").strip()
            except EOFError:
                return
            if command:
                self._commands.put(command)

    def _on_cloud(self, msg: PointCloud2) -> None:
        if np is None:
            return
        points = point_cloud_to_xyz_array(msg)
        if points is None or len(points) == 0:
            return
        self._cloud_points = points
        self._cloud_dirty = True

    def _on_path(self, msg: Path) -> None:
        if np is None:
            return
        if not msg.poses:
            self._path_points = None
            self._path_dirty = True
            return
        self._path_points = np.asarray(
            [
                [
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                ]
                for pose in msg.poses
            ],
            dtype=np.float64,
        )
        self._path_dirty = True

    def process_console_commands(self) -> None:
        while True:
            try:
                command = self._commands.get_nowait()
            except queue.Empty:
                return
            self._handle_command(command)

    def _handle_command(self, command: str) -> None:
        parts = command.replace(",", " ").split()
        if not parts:
            return

        kind: Optional[str] = None
        values = parts
        if parts[0].lower() in ("start", "s"):
            kind = "start"
            values = parts[1:]
        elif parts[0].lower() in ("goal", "g"):
            kind = "goal"
            values = parts[1:]

        if kind is None:
            kind = "start" if self._next_is_start else "goal"
            self._next_is_start = not self._next_is_start

        if len(values) != 3:
            self.get_logger().warn("Expected coordinates: start x y z, goal x y z, or x y z.")
            return

        try:
            x, y, z = (float(v) for v in values)
        except ValueError:
            self.get_logger().warn(f"Invalid numeric coordinates: {command}")
            return

        self._publish_pose(kind, x, y, z)

    def _publish_pose(self, kind: str, x: float, y: float, z: float) -> None:
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.get_parameter("frame_id").value
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0

        if kind == "start":
            self.initial_pose_pub.publish(pose)
            self.get_logger().info(f"Published /initial_pose_3d: [{x:.3f}, {y:.3f}, {z:.3f}]")
        else:
            self.goal_pose_pub.publish(pose)
            self.get_logger().info(f"Published /goal_pose_3d: [{x:.3f}, {y:.3f}, {z:.3f}]")


def run_terminal_only(node: Open3DGoalPicker) -> None:
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        node.process_console_commands()
        time.sleep(0.02)


def run_open3d(node: Open3DGoalPicker) -> None:
    vis = o3d.visualization.Visualizer()
    if not vis.create_window(window_name="3D Goal Picker", width=1280, height=800):
        node.get_logger().error("Failed to create Open3D window. Falling back to terminal-only mode.")
        run_terminal_only(node)
        return

    pcd = o3d.geometry.PointCloud()
    path_lines = o3d.geometry.LineSet()
    pcd_added = False
    path_added = False

    render_options = vis.get_render_option()
    render_options.point_size = float(node.get_parameter("point_size").value)
    render_options.background_color = np.asarray([0.03, 0.035, 0.04])

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.process_console_commands()

            if node._cloud_dirty and node._cloud_points is not None:
                node._cloud_dirty = False
                pcd.points = o3d.utility.Vector3dVector(node._cloud_points)
                colors = np.zeros_like(node._cloud_points)
                colors[:, 0] = 0.95
                colors[:, 1] = 0.48
                colors[:, 2] = 0.14
                pcd.colors = o3d.utility.Vector3dVector(colors)
                if not pcd_added:
                    vis.add_geometry(pcd)
                    vis.get_view_control().set_zoom(0.45)
                    pcd_added = True
                else:
                    vis.update_geometry(pcd)

            if node._path_dirty:
                node._path_dirty = False
                if node._path_points is None or len(node._path_points) < 2:
                    if path_added:
                        path_lines.points = o3d.utility.Vector3dVector([])
                        path_lines.lines = o3d.utility.Vector2iVector([])
                        vis.update_geometry(path_lines)
                    continue
                path_lines.points = o3d.utility.Vector3dVector(node._path_points)
                lines = [[i, i + 1] for i in range(len(node._path_points) - 1)]
                path_lines.lines = o3d.utility.Vector2iVector(lines)
                path_lines.colors = o3d.utility.Vector3dVector(
                    np.tile(np.asarray([[0.05, 0.9, 1.0]]), (len(lines), 1))
                )
                if not path_added:
                    vis.add_geometry(path_lines)
                    path_added = True
                else:
                    vis.update_geometry(path_lines)

            keep_running = vis.poll_events()
            vis.update_renderer()
            if not keep_running:
                break
            time.sleep(0.02)
    finally:
        vis.destroy_window()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Open3DGoalPicker()
    try:
        if o3d is None or np is None:
            run_terminal_only(node)
        else:
            run_open3d(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
