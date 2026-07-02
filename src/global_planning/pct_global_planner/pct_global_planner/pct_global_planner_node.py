import math
import time
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
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from .debug_artifacts import save_debug_artifacts
from .path_postprocessor import (
    PathPostprocessConfig,
    PlanningSafetyMap,
    distance,
    path_length,
    postprocess_path_with_report,
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
        self.global_debug_pub = self.create_publisher(String, self.global_planner_debug_topic, latched_qos())
        self.clearance_marker_pub = self.create_publisher(
            MarkerArray,
            self.planned_path_clearance_marker_topic,
            latched_qos(),
        )
        self.risk_marker_pub = self.create_publisher(
            MarkerArray,
            self.planned_path_risk_marker_topic,
            latched_qos(),
        )
        self.global_clearance_marker_pub = self.create_publisher(
            MarkerArray,
            self.global_clearance_map_marker_topic,
            latched_qos(),
        )
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
        self.safety_map = PlanningSafetyMap(self.postprocess_cfg, logger=self.get_logger())
        if self.postprocess_cfg.clearance_cost_enabled:
            self.safety_map.load()
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
            "planner_lib_dir": "src/map_process/PCT_planner/planner/lib",
            "pct_ros2_source_dir": "src/map_process/PCT_planner/pct_planner_ros2",
            "use_quintic": True,
            "max_heading_rate": 6.0,
            "path_z_offset": 0.0,
            "astar_cost_threshold": 20.0,
            "astar_step_cost_weight": 0.85,
            "optimizer_safe_cost_threshold": 8.0,
            "min_direct_path_distance": 0.35,
            "start_z_override_enabled": False,
            "start_z_override": 0.0,
            "enable_path_smoothing": True,
            "enable_path_resampling": True,
            "path_resample_resolution": 0.3,
            "max_path_z_jump": 0.4,
            "optimized_path_collision_cost_threshold": 20.0,
            "optimized_path_collision_check_resolution": 0.15,
            "remove_duplicate_points": True,
            "smoothing_passes": 1,
            "clearance_cost_enabled": True,
            "robot_radius": 0.35,
            "leg_safety_margin": 0.20,
            "preferred_clearance": 0.75,
            "hard_min_clearance": 0.35,
            "clearance_weight": 2.0,
            "path_length_weight": 1.0,
            "smoothness_weight": 0.5,
            "centerline_weight": 1.0,
            "stair_edge_weight": 3.0,
            "unknown_as_occupied": True,
            "adaptive_inflation_enabled": True,
            "normal_inflation_radius": 0.45,
            "narrow_passage_inflation_radius": 0.25,
            "stair_area_inflation_radius": 0.60,
            "narrow_passage_detect_enabled": True,
            "narrow_passage_width_threshold": 1.2,
            "narrow_passage_min_clearance": 0.25,
            "stair_edge_detect_enabled": True,
            "stair_edge_z_gradient_threshold": 0.20,
            "stair_edge_clearance_boost": 1.5,
            "path_postprocess_enabled": True,
            "enable_clearance_optimization": True,
            "clearance_optimization_iterations": 35,
            "clearance_optimization_step": 0.05,
            "smoothing_iterations": 18,
            "max_smoothing_deviation": 0.8,
            "keep_start_goal_fixed": True,
            "map_distance_max_points": 250000,
            "map_distance_xy_resolution": 0.15,
            "obstacle_min_relative_z": 0.12,
            "obstacle_max_relative_z": 1.60,
            "height_aware_clearance_enabled": True,
            "body_obstacle_min_relative_z": 0.20,
            "body_obstacle_max_relative_z": 1.60,
            "body_clearance_check_radius": 0.70,
            "body_clearance_use_all_map_points": False,
            "terrain_grid_resolution": 0.25,
            "terrain_edge_check_radius": 0.60,
            "global_search_clearance_enabled": True,
            "global_search_preferred_clearance": 0.50,
            "global_search_hard_min_clearance": 0.25,
            "global_search_clearance_weight": 35.0,
            "global_search_clearance_power": 2.0,
            "global_search_clearance_max_extra_cost": 12.0,
            "global_search_clearance_max_total_cost": 17.5,
            "publish_path_topic": "/planned_path",
            "publish_alias_path_topic": "/path",
            "publish_marker_topic": "/planned_path_marker",
            "status_topic": "/pct_global_planner/status",
            "global_planner_debug_topic": "/global_planner/debug",
            "planned_path_clearance_marker_topic": "/planned_path_clearance_marker",
            "planned_path_risk_marker_topic": "/planned_path_risk_marker",
            "global_clearance_map_marker_topic": "/global_clearance_map_marker",
            "publish_path_risk_debug": False,
            "publish_global_clearance_map_debug": False,
            "global_clearance_map_max_samples": 0,
            "marker_line_width": 0.08,
            "debug_output_enabled": False,
            "debug_output_dir": "debug",
            "debug_render_images": False,
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
        self.map_file = str(self.get_parameter("map_file").value)
        self.pcd_file = str(self.get_parameter("pcd_file").value)

        self.postprocess_cfg = PathPostprocessConfig(
            enable_path_smoothing=as_bool(self.get_parameter("enable_path_smoothing").value),
            enable_path_resampling=as_bool(self.get_parameter("enable_path_resampling").value),
            path_resample_resolution=float(self.get_parameter("path_resample_resolution").value),
            max_path_z_jump=float(self.get_parameter("max_path_z_jump").value),
            remove_duplicate_points=as_bool(self.get_parameter("remove_duplicate_points").value),
            smoothing_passes=int(self.get_parameter("smoothing_passes").value),
            clearance_cost_enabled=as_bool(self.get_parameter("clearance_cost_enabled").value),
            robot_radius=float(self.get_parameter("robot_radius").value),
            leg_safety_margin=float(self.get_parameter("leg_safety_margin").value),
            preferred_clearance=float(self.get_parameter("preferred_clearance").value),
            hard_min_clearance=float(self.get_parameter("hard_min_clearance").value),
            clearance_weight=float(self.get_parameter("clearance_weight").value),
            path_length_weight=float(self.get_parameter("path_length_weight").value),
            smoothness_weight=float(self.get_parameter("smoothness_weight").value),
            centerline_weight=float(self.get_parameter("centerline_weight").value),
            stair_edge_weight=float(self.get_parameter("stair_edge_weight").value),
            unknown_as_occupied=as_bool(self.get_parameter("unknown_as_occupied").value),
            adaptive_inflation_enabled=as_bool(self.get_parameter("adaptive_inflation_enabled").value),
            normal_inflation_radius=float(self.get_parameter("normal_inflation_radius").value),
            narrow_passage_inflation_radius=float(self.get_parameter("narrow_passage_inflation_radius").value),
            stair_area_inflation_radius=float(self.get_parameter("stair_area_inflation_radius").value),
            narrow_passage_detect_enabled=as_bool(self.get_parameter("narrow_passage_detect_enabled").value),
            narrow_passage_width_threshold=float(self.get_parameter("narrow_passage_width_threshold").value),
            narrow_passage_min_clearance=float(self.get_parameter("narrow_passage_min_clearance").value),
            stair_edge_detect_enabled=as_bool(self.get_parameter("stair_edge_detect_enabled").value),
            stair_edge_z_gradient_threshold=float(self.get_parameter("stair_edge_z_gradient_threshold").value),
            stair_edge_clearance_boost=float(self.get_parameter("stair_edge_clearance_boost").value),
            path_postprocess_enabled=as_bool(self.get_parameter("path_postprocess_enabled").value),
            enable_clearance_optimization=as_bool(self.get_parameter("enable_clearance_optimization").value),
            clearance_optimization_iterations=int(self.get_parameter("clearance_optimization_iterations").value),
            clearance_optimization_step=float(self.get_parameter("clearance_optimization_step").value),
            smoothing_iterations=int(self.get_parameter("smoothing_iterations").value),
            max_smoothing_deviation=float(self.get_parameter("max_smoothing_deviation").value),
            keep_start_goal_fixed=as_bool(self.get_parameter("keep_start_goal_fixed").value),
            pcd_file=self.pcd_file,
            map_file=self.map_file,
            map_distance_max_points=int(self.get_parameter("map_distance_max_points").value),
            map_distance_xy_resolution=float(self.get_parameter("map_distance_xy_resolution").value),
            obstacle_min_relative_z=float(self.get_parameter("obstacle_min_relative_z").value),
            obstacle_max_relative_z=float(self.get_parameter("obstacle_max_relative_z").value),
            height_aware_clearance_enabled=as_bool(self.get_parameter("height_aware_clearance_enabled").value),
            body_obstacle_min_relative_z=float(self.get_parameter("body_obstacle_min_relative_z").value),
            body_obstacle_max_relative_z=float(self.get_parameter("body_obstacle_max_relative_z").value),
            body_clearance_check_radius=float(self.get_parameter("body_clearance_check_radius").value),
            body_clearance_use_all_map_points=as_bool(self.get_parameter("body_clearance_use_all_map_points").value),
            terrain_grid_resolution=float(self.get_parameter("terrain_grid_resolution").value),
            terrain_edge_check_radius=float(self.get_parameter("terrain_edge_check_radius").value),
        )

        self.publish_path_topic = str(self.get_parameter("publish_path_topic").value)
        self.publish_alias_path_topic = str(self.get_parameter("publish_alias_path_topic").value)
        self.publish_marker_topic = str(self.get_parameter("publish_marker_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.global_planner_debug_topic = str(self.get_parameter("global_planner_debug_topic").value)
        self.planned_path_clearance_marker_topic = str(self.get_parameter("planned_path_clearance_marker_topic").value)
        self.planned_path_risk_marker_topic = str(self.get_parameter("planned_path_risk_marker_topic").value)
        self.global_clearance_map_marker_topic = str(self.get_parameter("global_clearance_map_marker_topic").value)
        self.publish_path_risk_debug = as_bool(self.get_parameter("publish_path_risk_debug").value)
        self.publish_global_clearance_map_debug = as_bool(self.get_parameter("publish_global_clearance_map_debug").value)
        self.global_clearance_map_max_samples = int(self.get_parameter("global_clearance_map_max_samples").value)
        self.marker_line_width = float(self.get_parameter("marker_line_width").value)
        self.debug_output_enabled = as_bool(self.get_parameter("debug_output_enabled").value)
        self.debug_output_dir = str(self.get_parameter("debug_output_dir").value)
        self.debug_render_images = as_bool(self.get_parameter("debug_render_images").value)
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

        t0 = time.perf_counter()
        ok, raw_path = self.adapter.plan(start, goal)
        t_plan = time.perf_counter()
        if not ok:
            self.get_logger().error(self.adapter.last_error or "PCT planning failed.")
            self._set_status("FAILED")
            return False

        path_with_endpoints = self._add_exact_endpoints(raw_path, start, goal)
        result = postprocess_path_with_report(path_with_endpoints, self.postprocess_cfg, self.safety_map)
        t_postprocess = time.perf_counter()
        path = result.optimized_path
        if not path:
            self.get_logger().error("Path postprocessing removed all waypoints.")
            self._set_status("FAILED")
            return False

        self._publish_path(path)
        self._publish_risk_debug(path, result.metrics_after)
        self._publish_global_clearance_map()
        t_publish = time.perf_counter()
        if self.debug_output_enabled:
            save_debug_artifacts(
                result.raw_path,
                path,
                self.safety_map,
                self.postprocess_cfg,
                self.debug_output_dir,
                self.debug_render_images,
                logger=self.get_logger(),
            )
        t_debug = time.perf_counter()
        length = path_length(path)
        self.get_logger().info(
            "PCT global path published: points=%d length=%.3fm min_clearance=%.3fm avg_clearance=%.3fm postprocess_improved=%s"
            % (
                len(path),
                length,
                result.metrics_after.min_clearance,
                result.metrics_after.avg_clearance,
                result.metrics_after.postprocess_improved,
            )
        )
        self.get_logger().info(
            "PCT planning timing: pct_core=%.1fms postprocess=%.1fms publish_debug=%.1fms file_debug=%.1fms total=%.1fms"
            % (
                (t_plan - t0) * 1000.0,
                (t_postprocess - t_plan) * 1000.0,
                (t_publish - t_postprocess) * 1000.0,
                (t_debug - t_publish) * 1000.0,
                (t_debug - t0) * 1000.0,
            )
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

    def _publish_risk_debug(self, points: list[Point3], metrics) -> None:
        if self.publish_path_risk_debug:
            stamp = self.get_clock().now().to_msg()
            self.clearance_marker_pub.publish(
                self._make_path_risk_marker_array(
                    points,
                    stamp,
                    "planned_path_clearance",
                    color_by_clearance=True,
                )
            )
            self.risk_marker_pub.publish(
                self._make_path_risk_marker_array(
                    points,
                    stamp,
                    "planned_path_risk",
                    color_by_clearance=False,
                )
            )

        msg = String()
        msg.data = (
            "path_length=%.3f min_clearance=%s avg_clearance=%s max_risk=%.3f "
            "max_risk_point=(%.3f, %.3f, %.3f) narrow_passage_count=%d "
            "stair_edge_risk_count=%d postprocess_improved=%s"
            % (
                metrics.path_length,
                self._format_float(metrics.min_clearance),
                self._format_float(metrics.avg_clearance),
                metrics.max_risk,
                metrics.max_risk_point[0],
                metrics.max_risk_point[1],
                metrics.max_risk_point[2],
                metrics.narrow_passage_count,
                metrics.stair_edge_risk_count,
                str(metrics.postprocess_improved).lower(),
            )
        )
        self.global_debug_pub.publish(msg)

    def _make_path_risk_marker_array(
        self,
        points: list[Point3],
        stamp,
        namespace: str,
        color_by_clearance: bool,
    ) -> MarkerArray:
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.map_frame
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = max(0.08, self.marker_line_width * 1.6)
        marker.scale.y = marker.scale.x

        for point in points:
            p = Point()
            p.x, p.y, p.z = point
            marker.points.append(p)
            clearance = self.safety_map.query_obstacle_distance(point)
            risk = self.safety_map.compute_clearance_cost(point)
            marker.colors.append(
                self._risk_color_from_clearance(clearance)
                if color_by_clearance
                else self._risk_color_from_cost(risk)
            )

        array = MarkerArray()
        array.markers.append(marker)
        return array

    def _publish_global_clearance_map(self) -> None:
        if not self.publish_global_clearance_map_debug or self.global_clearance_map_max_samples <= 0:
            return
        if not self.safety_map.loaded:
            return

        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.map_frame
        marker.ns = "global_clearance_map"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.06
        marker.scale.y = 0.06

        for point, clearance in self.safety_map.sample_clearance_points(self.global_clearance_map_max_samples):
            p = Point()
            p.x, p.y, p.z = point
            marker.points.append(p)
            marker.colors.append(self._risk_color_from_clearance(clearance))

        array = MarkerArray()
        array.markers.append(marker)
        self.global_clearance_marker_pub.publish(array)

    def _risk_color_from_clearance(self, clearance: float) -> ColorRGBA:
        if not math.isfinite(clearance):
            return self._rgba(0.25, 0.25, 0.25, 0.35)
        if clearance >= self.postprocess_cfg.preferred_clearance:
            return self._rgba(0.0, 0.85, 0.20, 0.95)
        if clearance >= 0.75 * self.postprocess_cfg.preferred_clearance:
            return self._rgba(1.0, 0.85, 0.05, 0.95)
        if clearance >= self.postprocess_cfg.hard_min_clearance:
            return self._rgba(1.0, 0.45, 0.0, 0.95)
        return self._rgba(1.0, 0.05, 0.02, 0.98)

    def _risk_color_from_cost(self, risk: float) -> ColorRGBA:
        if risk >= 1000.0:
            return self._rgba(1.0, 0.02, 0.02, 0.98)
        if risk >= 5.0:
            return self._rgba(1.0, 0.25, 0.0, 0.95)
        if risk >= 1.0:
            return self._rgba(1.0, 0.85, 0.05, 0.95)
        return self._rgba(0.0, 0.85, 0.20, 0.95)

    @staticmethod
    def _rgba(r: float, g: float, b: float, a: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color

    @staticmethod
    def _format_float(value: float) -> str:
        if not math.isfinite(value):
            return "inf"
        return "%.3f" % value

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
