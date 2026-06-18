import math
from typing import Optional

import rclpy
import tf2_ros
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker

from .path_postprocessor import (
    PathPostprocessConfig,
    distance,
    path_length,
    postprocess_path,
)
from .pct_adapter import PCTPlannerAdapter, Point3


def latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def yaw_to_quaternion(yaw: float):
    from geometry_msgs.msg import Quaternion

    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PCTGlobalPlannerNode(Node):
    def __init__(self):
        super().__init__("pct_global_planner_node")

        self._declare_parameters()
        self._read_parameters()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, self.publish_path_topic, latched_qos())
        self.alias_path_pub = None
        if self.publish_alias_path_topic and self.publish_alias_path_topic != self.publish_path_topic:
            self.alias_path_pub = self.create_publisher(Path, self.publish_alias_path_topic, latched_qos())

        self.marker_pub = self.create_publisher(Marker, self.publish_marker_topic, latched_qos())
        self.status_pub = self.create_publisher(String, self.status_topic, latched_qos())
        self.replan_srv = self.create_service(Trigger, "/replan_global_path", self._handle_replan)
        self.current_start_pub = self.create_publisher(
            PoseStamped,
            self.current_start_pose_topic,
            latched_qos(),
        )
        self.current_start_marker_pub = self.create_publisher(
            Marker,
            self.current_start_marker_topic,
            latched_qos(),
        )
        self.mirror_start_pub = None
        if self.mirror_current_start_to_start_pose_topic:
            self.mirror_start_pub = self.create_publisher(PoseStamped, self.start_pose_topic, latched_qos())

        self.last_odom: Optional[Odometry] = None
        self.last_start_pose: Optional[PoseStamped] = None
        self.last_goal: Optional[Point3] = None
        self.last_goal_frame = self.map_frame
        self.last_replan_start: Optional[Point3] = None
        self.planning_in_progress = False

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._odom_callback,
            qos_profile_sensor_data,
        )
        self.start_pose_sub = self.create_subscription(
            PoseStamped,
            self.start_pose_topic,
            self._start_pose_callback,
            latched_qos(),
        )
        self._create_goal_subscriptions()

        self.adapter = PCTPlannerAdapter()
        self._set_status("WAITING_FOR_MAP")
        if self.adapter.initialize(self):
            self._set_status("WAITING_FOR_GOAL")
        else:
            self._set_status("MAP_ERROR")

        self.current_start_timer = None
        if self.publish_current_start:
            period = 1.0 / max(0.1, self.current_start_publish_rate)
            self.current_start_timer = self.create_timer(period, self._current_start_timer_callback)

        self.get_logger().info(
            "pct_global_planner_node ready. start_source=%s goal_topics=[%s, %s, %s, %s] output=%s alias=%s marker=%s current_start=%s"
            % (
                self.start_source,
                self.goal_pose_topic,
                self.goal_point_topic,
                self.rviz_2d_goal_topic,
                self.pct_marker_goal_pose_topic,
                self.publish_path_topic,
                self.publish_alias_path_topic,
                self.publish_marker_topic,
                self.current_start_pose_topic,
            )
        )

    def _declare_parameters(self) -> None:
        defaults = {
            "map_frame": "map",
            "base_frame": "base_link",
            "use_tf_start": True,
            "start_source": "tf",
            "start_pose_topic": "/pct_planner/start_pose",
            "odom_topic": "/odom",
            "tf_lookup_timeout": 0.25,
            "goal_pose_topic": "/goal_pose_3d",
            "goal_point_topic": "/goal_point_3d",
            "rviz_2d_goal_topic": "/goal_pose",
            "pct_marker_goal_pose_topic": "/pct_planner/goal_pose",
            "default_goal_z": 0.0,
            "map_source": "tomogram",
            "map_file": "maps/map_preprocessed.pcd",
            "pcd_file": "maps/map_preprocessed.pcd",
            "pointcloud_topic": "/cloud_registered",
            "use_static_map_file": True,
            "scene": "Building",
            "tomogram_file": "map_preprocessed",
            "tomogram_dir": "maps/tomogram",
            "planner_lib_dir": "src/PCT_planner/planner/lib",
            "pct_ros2_source_dir": "src/PCT_planner/pct_planner_ros2",
            "use_quintic": True,
            "max_heading_rate": 6.0,
            "path_z_offset": 0.0,
            "astar_cost_threshold": 20.0,
            "astar_step_cost_weight": 0.65,
            "optimizer_safe_cost_threshold": 8.0,
            "min_direct_path_distance": 0.35,
            "start_z_override_enabled": False,
            "start_z_override": 0.0,
            "enable_path_smoothing": True,
            "enable_path_resampling": True,
            "path_resample_resolution": 0.2,
            "max_path_z_jump": 0.4,
            "remove_duplicate_points": True,
            "smoothing_passes": 1,
            "publish_path_topic": "/planned_path",
            "publish_alias_path_topic": "/path",
            "publish_marker_topic": "/planned_path_marker",
            "status_topic": "/pct_global_planner/status",
            "marker_line_width": 0.08,
            "publish_current_start": True,
            "current_start_pose_topic": "/pct_global_planner/current_start_pose",
            "current_start_marker_topic": "/pct_global_planner/current_start_marker",
            "current_start_publish_rate": 5.0,
            "current_start_marker_scale": 0.35,
            "mirror_current_start_to_start_pose_topic": False,
            "auto_replan_on_start_update": False,
            "start_update_min_distance": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _read_parameters(self) -> None:
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.use_tf_start = as_bool(self.get_parameter("use_tf_start").value)
        self.start_source = str(self.get_parameter("start_source").value).strip().lower()
        if not self.start_source:
            self.start_source = "tf" if self.use_tf_start else "odom"
        self.start_pose_topic = str(self.get_parameter("start_pose_topic").value)
        self.odom_topic = str(self.get_parameter("odom_topic").value)
        self.tf_lookup_timeout = float(self.get_parameter("tf_lookup_timeout").value)

        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.goal_point_topic = str(self.get_parameter("goal_point_topic").value)
        self.rviz_2d_goal_topic = str(self.get_parameter("rviz_2d_goal_topic").value)
        self.pct_marker_goal_pose_topic = str(self.get_parameter("pct_marker_goal_pose_topic").value)
        self.default_goal_z = float(self.get_parameter("default_goal_z").value)

        self.start_z_override_enabled = as_bool(self.get_parameter("start_z_override_enabled").value)
        self.start_z_override = float(self.get_parameter("start_z_override").value)

        self.postprocess_cfg = PathPostprocessConfig(
            enable_path_smoothing=as_bool(self.get_parameter("enable_path_smoothing").value),
            enable_path_resampling=as_bool(self.get_parameter("enable_path_resampling").value),
            path_resample_resolution=float(self.get_parameter("path_resample_resolution").value),
            max_path_z_jump=float(self.get_parameter("max_path_z_jump").value),
            remove_duplicate_points=as_bool(self.get_parameter("remove_duplicate_points").value),
            smoothing_passes=int(self.get_parameter("smoothing_passes").value),
        )

        self.publish_path_topic = str(self.get_parameter("publish_path_topic").value)
        self.publish_alias_path_topic = str(self.get_parameter("publish_alias_path_topic").value)
        self.publish_marker_topic = str(self.get_parameter("publish_marker_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.marker_line_width = float(self.get_parameter("marker_line_width").value)
        self.publish_current_start = as_bool(self.get_parameter("publish_current_start").value)
        self.current_start_pose_topic = str(self.get_parameter("current_start_pose_topic").value)
        self.current_start_marker_topic = str(self.get_parameter("current_start_marker_topic").value)
        self.current_start_publish_rate = float(self.get_parameter("current_start_publish_rate").value)
        self.current_start_marker_scale = float(self.get_parameter("current_start_marker_scale").value)
        self.mirror_current_start_to_start_pose_topic = as_bool(
            self.get_parameter("mirror_current_start_to_start_pose_topic").value
        )
        self.auto_replan_on_start_update = as_bool(self.get_parameter("auto_replan_on_start_update").value)
        self.start_update_min_distance = float(self.get_parameter("start_update_min_distance").value)

    def _create_goal_subscriptions(self) -> None:
        self.goal_subs = []
        pose_topics = [
            (self.goal_pose_topic, False, "goal_pose_topic"),
            (self.rviz_2d_goal_topic, True, "rviz_2d_goal_topic"),
            (self.pct_marker_goal_pose_topic, False, "pct_marker_goal_pose_topic"),
        ]

        seen_pose_topics = set()
        for topic, force_default_z, label in pose_topics:
            if not topic or topic in seen_pose_topics:
                continue
            seen_pose_topics.add(topic)
            self.goal_subs.append(
                self.create_subscription(
                    PoseStamped,
                    topic,
                    lambda msg, default_z=force_default_z, source=label: self._goal_pose_callback(
                        msg, default_z, source
                    ),
                    latched_qos(),
                )
            )

        if self.goal_point_topic:
            self.goal_subs.append(
                self.create_subscription(
                    PointStamped,
                    self.goal_point_topic,
                    self._goal_point_callback,
                    latched_qos(),
                )
            )

    def _odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg

    def _start_pose_callback(self, msg: PoseStamped) -> None:
        self.last_start_pose = msg

    def _goal_pose_callback(self, msg: PoseStamped, force_default_z: bool, source: str) -> None:
        z = self.default_goal_z if force_default_z else float(msg.pose.position.z)
        xyz = (float(msg.pose.position.x), float(msg.pose.position.y), z)
        ok, goal = self._transform_xyz_to_map(xyz, msg.header.frame_id)
        if not ok:
            self._set_status("TF_ERROR")
            return

        self.last_goal = goal
        self.last_goal_frame = self.map_frame
        self.get_logger().info(
            "Received goal from %s: [%.3f, %.3f, %.3f]" % (source, goal[0], goal[1], goal[2])
        )
        self._plan_to_goal(goal)

    def _goal_point_callback(self, msg: PointStamped) -> None:
        xyz = (float(msg.point.x), float(msg.point.y), float(msg.point.z))
        ok, goal = self._transform_xyz_to_map(xyz, msg.header.frame_id)
        if not ok:
            self._set_status("TF_ERROR")
            return

        self.last_goal = goal
        self.last_goal_frame = self.map_frame
        self.get_logger().info("Received goal point: [%.3f, %.3f, %.3f]" % goal)
        self._plan_to_goal(goal)

    def _handle_replan(self, _request, response):
        if self.last_goal is None:
            response.success = False
            response.message = "No goal has been received yet."
            self._set_status("WAITING_FOR_GOAL")
            return response

        response.success = self._plan_to_goal(self.last_goal)
        response.message = "Global replan succeeded." if response.success else "Global replan failed."
        return response

    def _plan_to_goal(self, goal: Point3) -> bool:
        if self.planning_in_progress:
            self.get_logger().debug("Skipping global replan because a plan is already running.")
            return False

        self.planning_in_progress = True
        try:
            return self._plan_to_goal_impl(goal)
        finally:
            self.planning_in_progress = False

    def _plan_to_goal_impl(self, goal: Point3) -> bool:
        if not self.adapter.initialized:
            self.get_logger().error(self.adapter.last_error or "PCT Planner adapter is not initialized.")
            self._set_status("MAP_ERROR")
            return False

        ok, start = self._get_current_start()
        if not ok:
            self._set_status("TF_ERROR" if self.start_source == "tf" else "FAILED")
            return False

        self._set_status("PLANNING")
        self.get_logger().info(
            "Planning PCT global path: start=[%.3f, %.3f, %.3f] goal=[%.3f, %.3f, %.3f]"
            % (start[0], start[1], start[2], goal[0], goal[1], goal[2])
        )

        ok, raw_path = self.adapter.plan(start, goal)
        if not ok:
            self.get_logger().error(self.adapter.last_error or "PCT planning failed.")
            self._set_status("FAILED")
            return False

        path = self._add_exact_endpoints(raw_path, start, goal)
        path = postprocess_path(path, self.postprocess_cfg)
        if not path:
            self.get_logger().error("Path postprocessing removed all waypoints.")
            self._set_status("FAILED")
            return False

        self._publish_path(path)
        length = path_length(path)
        self.get_logger().info(
            "PCT global path published: points=%d length=%.3fm" % (len(path), length)
        )
        self.last_replan_start = start
        self._set_status("SUCCESS")
        return True

    def _get_current_start(self, warn_on_failure: bool = True) -> tuple[bool, Point3]:
        if self.start_source == "tf":
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=self.tf_lookup_timeout),
                )
                t = transform.transform.translation
                start = (float(t.x), float(t.y), float(t.z))
                return True, self._apply_start_z_override(start)
            except Exception as exc:  # noqa: BLE001
                if warn_on_failure:
                    self.get_logger().warning(
                        "Cannot query TF %s -> %s for planner start: %s"
                        % (self.map_frame, self.base_frame, exc)
                    )
                return False, (0.0, 0.0, 0.0)

        if self.start_source == "odom":
            if self.last_odom is None:
                if warn_on_failure:
                    self.get_logger().warning("No odometry received on %s; cannot plan." % self.odom_topic)
                return False, (0.0, 0.0, 0.0)
            pose = self.last_odom.pose.pose
            xyz = (float(pose.position.x), float(pose.position.y), float(pose.position.z))
            ok, start = self._transform_xyz_to_map(xyz, self.last_odom.header.frame_id)
            return ok, self._apply_start_z_override(start) if ok else start

        if self.start_source == "topic":
            if self.last_start_pose is None:
                if warn_on_failure:
                    self.get_logger().warning(
                        "No start pose received on %s; cannot plan." % self.start_pose_topic
                    )
                return False, (0.0, 0.0, 0.0)
            pose = self.last_start_pose.pose
            xyz = (float(pose.position.x), float(pose.position.y), float(pose.position.z))
            ok, start = self._transform_xyz_to_map(xyz, self.last_start_pose.header.frame_id)
            return ok, self._apply_start_z_override(start) if ok else start

        self.get_logger().error("Unsupported start_source='%s'. Use tf, odom, or topic." % self.start_source)
        return False, (0.0, 0.0, 0.0)

    def _current_start_timer_callback(self) -> None:
        ok, start = self._get_current_start(warn_on_failure=False)
        if not ok:
            return

        pose_msg = self._make_start_pose_msg(start)
        self.current_start_pub.publish(pose_msg)
        if self.mirror_start_pub is not None:
            self.mirror_start_pub.publish(pose_msg)
        self.current_start_marker_pub.publish(self._make_current_start_marker(start, pose_msg.header.stamp))

        if (
            self.auto_replan_on_start_update
            and self.last_goal is not None
            and self.adapter.initialized
            and not self.planning_in_progress
        ):
            if self.last_replan_start is None or distance(start, self.last_replan_start) >= self.start_update_min_distance:
                self.get_logger().info(
                    "Current start moved %.3fm; replanning to last goal."
                    % (0.0 if self.last_replan_start is None else distance(start, self.last_replan_start))
                )
                self._plan_to_goal(self.last_goal)

    def _make_start_pose_msg(self, start: Point3) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = start[0]
        msg.pose.position.y = start[1]
        msg.pose.position.z = start[2]
        msg.pose.orientation.w = 1.0
        return msg

    def _make_current_start_marker(self, start: Point3, stamp) -> Marker:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = "pct_current_start"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = start[0]
        marker.pose.position.y = start[1]
        marker.pose.position.z = start[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.current_start_marker_scale
        marker.scale.y = self.current_start_marker_scale
        marker.scale.z = self.current_start_marker_scale
        marker.color.r = 0.0
        marker.color.g = 0.95
        marker.color.b = 0.2
        marker.color.a = 0.9
        return marker

    def _apply_start_z_override(self, start: Point3) -> Point3:
        if not self.start_z_override_enabled:
            return start
        return (start[0], start[1], self.start_z_override)

    def _transform_xyz_to_map(self, xyz: Point3, frame_id: str) -> tuple[bool, Point3]:
        source_frame = frame_id.strip() if frame_id else self.map_frame
        if source_frame == self.map_frame:
            return True, xyz

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout),
            )
            return True, self._apply_transform(transform, xyz)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                "Cannot transform point from '%s' to '%s': %s"
                % (source_frame, self.map_frame, exc)
            )
            return False, xyz

    @staticmethod
    def _apply_transform(transform, xyz: Point3) -> Point3:
        q = transform.transform.rotation
        t = transform.transform.translation
        rx, ry, rz = PCTGlobalPlannerNode._rotate_vector(
            (q.x, q.y, q.z, q.w),
            xyz,
        )
        return (rx + t.x, ry + t.y, rz + t.z)

    @staticmethod
    def _rotate_vector(q, v: Point3) -> Point3:
        qx, qy, qz, qw = q
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm <= 1.0e-12:
            return v
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        uv = (
            qy * v[2] - qz * v[1],
            qz * v[0] - qx * v[2],
            qx * v[1] - qy * v[0],
        )
        uuv = (
            qy * uv[2] - qz * uv[1],
            qz * uv[0] - qx * uv[2],
            qx * uv[1] - qy * uv[0],
        )
        return (
            v[0] + 2.0 * (qw * uv[0] + uuv[0]),
            v[1] + 2.0 * (qw * uv[1] + uuv[1]),
            v[2] + 2.0 * (qw * uv[2] + uuv[2]),
        )

    def _add_exact_endpoints(self, raw_path: list[Point3], start: Point3, goal: Point3) -> list[Point3]:
        points = []
        if not raw_path or distance(start, raw_path[0]) > self.postprocess_cfg.duplicate_epsilon:
            points.append(start)
        points.extend(raw_path)
        if not points or distance(points[-1], goal) > self.postprocess_cfg.duplicate_epsilon:
            points.append(goal)
        return points

    def _publish_path(self, points: list[Point3]) -> None:
        stamp = self.get_clock().now().to_msg()
        path_msg = Path()
        path_msg.header.stamp = stamp
        path_msg.header.frame_id = self.map_frame

        for index, point in enumerate(points):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]

            if index + 1 < len(points):
                next_point = points[index + 1]
                yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
            elif index > 0:
                prev_point = points[index - 1]
                yaw = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
            else:
                yaw = 0.0
            pose.pose.orientation = yaw_to_quaternion(yaw)
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)
        if self.alias_path_pub is not None:
            self.alias_path_pub.publish(path_msg)

        marker = Marker()
        marker.header = path_msg.header
        marker.ns = "pct_global_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = self.marker_line_width
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        for point in points:
            p = Point()
            p.x, p.y, p.z = point
            marker.points.append(p)
        self.marker_pub.publish(marker)

    def _set_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PCTGlobalPlannerNode()
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
