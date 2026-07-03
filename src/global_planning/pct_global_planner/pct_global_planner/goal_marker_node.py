import copy
import math

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from interactive_markers import InteractiveMarkerServer, MenuHandler
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)


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


def quaternion_from_rpy(roll: float, pitch: float, yaw: float) -> Quaternion:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def make_pose(x: float, y: float, z: float, roll: float, pitch: float, yaw: float) -> Pose:
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation = quaternion_from_rpy(float(roll), float(pitch), float(yaw))
    return pose


class PCTGoalMarkerNode(Node):
    def __init__(self):
        super().__init__("pct_start_goal_marker")

        self.declare_parameter("frame_id", "map")
        self.declare_parameter("goal_pose_topic", "/pct_planner/goal_pose")
        self.declare_parameter("goal_pose_alias_topic", "/goal_pose_3d")
        self.declare_parameter("marker_array_topic", "/pct_planner/start_goal_markers")
        self.declare_parameter("server_namespace", "pct_start_goal_marker")
        self.declare_parameter("marker_scale", 0.8)
        self.declare_parameter("sphere_radius", 0.175)
        self.declare_parameter("direct_sphere_drag", False)
        self.declare_parameter("publish_on_feedback", True)
        self.declare_parameter("publish_initial_pose", False)
        self.declare_parameter("publish_marker_array", True)
        self.declare_parameter("default_goal_x", 5.0)
        self.declare_parameter("default_goal_y", 11.0)
        self.declare_parameter("default_goal_z", 0.0)
        self.declare_parameter("default_goal_roll", 0.0)
        self.declare_parameter("default_goal_pitch", 0.0)
        self.declare_parameter("default_goal_yaw", 0.0)

        self.frame_id = str(self.get_parameter("frame_id").value)
        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.goal_pose_alias_topic = str(self.get_parameter("goal_pose_alias_topic").value)
        self.marker_array_topic = str(self.get_parameter("marker_array_topic").value)
        self.server_namespace = str(self.get_parameter("server_namespace").value)
        self.marker_scale = float(self.get_parameter("marker_scale").value)
        self.sphere_radius = float(self.get_parameter("sphere_radius").value)
        self.direct_sphere_drag = as_bool(self.get_parameter("direct_sphere_drag").value)
        self.publish_on_feedback = as_bool(self.get_parameter("publish_on_feedback").value)
        self.publish_initial_pose = as_bool(self.get_parameter("publish_initial_pose").value)
        self.publish_marker_array_enabled = as_bool(self.get_parameter("publish_marker_array").value)

        self.goal_pose = make_pose(
            float(self.get_parameter("default_goal_x").value),
            float(self.get_parameter("default_goal_y").value),
            float(self.get_parameter("default_goal_z").value),
            float(self.get_parameter("default_goal_roll").value),
            float(self.get_parameter("default_goal_pitch").value),
            float(self.get_parameter("default_goal_yaw").value),
        )

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, latched_qos())
        self.goal_alias_pub = None
        if self.goal_pose_alias_topic and self.goal_pose_alias_topic != self.goal_pose_topic:
            self.goal_alias_pub = self.create_publisher(PoseStamped, self.goal_pose_alias_topic, latched_qos())
        self.marker_array_pub = self.create_publisher(MarkerArray, self.marker_array_topic, latched_qos())
        self._last_feedback_log_time = 0.0

        self.server = InteractiveMarkerServer(self, self.server_namespace)
        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Publish Goal", callback=self._publish_goal_cb)

        marker = self._make_interactive_marker()
        self.server.insert(marker, feedback_callback=self._feedback_cb)
        self.menu_handler.apply(self.server, marker.name)
        self.server.applyChanges()
        self.publish_visual_markers()

        self.get_logger().info(
            'Goal marker ready in frame "%s"; Interactive Markers Namespace="%s"; publishing goal to "%s"'
            % (self.frame_id, self.server_namespace, self.goal_pose_topic)
        )

        if self.publish_initial_pose:
            self.publish_goal()

    def _make_interactive_marker(self) -> InteractiveMarker:
        marker = InteractiveMarker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.name = "pct_goal_pose"
        marker.description = (
            "Goal Pose: drag axes to move"
            if not self.direct_sphere_drag
            else "Goal Pose: drag sphere in XY, use Z axis for height"
        )
        marker.scale = self.marker_scale
        marker.pose = copy.deepcopy(self.goal_pose)

        sphere_control = InteractiveMarkerControl()
        sphere_control.name = "goal_sphere_drag_xy" if self.direct_sphere_drag else "goal_sphere_menu"
        sphere_control.always_visible = True
        if self.direct_sphere_drag:
            sphere_control.orientation_mode = InteractiveMarkerControl.FIXED
            sphere_control.orientation.w = math.sqrt(0.5)
            sphere_control.orientation.y = -math.sqrt(0.5)
            sphere_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        else:
            sphere_control.interaction_mode = InteractiveMarkerControl.MENU
        sphere_control.markers.append(self._make_goal_sphere())
        marker.controls.append(sphere_control)

        self._add_axis_controls(marker)
        return marker

    def _make_goal_sphere(self) -> Marker:
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = self.sphere_radius * 2.0
        marker.scale.y = self.sphere_radius * 2.0
        marker.scale.z = self.sphere_radius * 2.0
        marker.color.r = 0.95
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 0.9
        return marker

    def _add_axis_controls(self, marker: InteractiveMarker) -> None:
        # RViz MOVE_AXIS uses the control's local X axis. These unit
        # quaternions match the standard interactive_marker 6-DOF mapping.
        s = math.sqrt(0.5)
        axes = (
            ("x", s, 0.0, 0.0, s),
            ("z", 0.0, s, 0.0, s),
            ("y", 0.0, 0.0, s, s),
        )
        for axis_name, x, y, z, w in axes:
            marker.controls.append(
                self._make_axis_control(
                    name=f"rotate_{axis_name}",
                    interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                    x=x,
                    y=y,
                    z=z,
                    w=w,
                )
            )
            marker.controls.append(
                self._make_axis_control(
                    name=f"move_{axis_name}",
                    interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
                    x=x,
                    y=y,
                    z=z,
                    w=w,
                )
            )

    @staticmethod
    def _make_axis_control(
        name: str,
        interaction_mode: int,
        x: float,
        y: float,
        z: float,
        w: float,
    ):
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = w
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = interaction_mode
        return control

    def _feedback_cb(self, feedback: InteractiveMarkerFeedback) -> None:
        if feedback.marker_name != "pct_goal_pose":
            return

        if feedback.event_type in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
            InteractiveMarkerFeedback.BUTTON_CLICK,
        ):
            self.goal_pose = copy.deepcopy(feedback.pose)
            self.publish_visual_markers()

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE and self.publish_on_feedback:
            self.publish_goal(log_level="debug")
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
            self._log_feedback_event("mouse up", feedback.pose)
            self.publish_goal()
        elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
            self._log_feedback_event("mouse down", feedback.pose)

    def _log_feedback_event(self, event_name: str, pose: Pose) -> None:
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_feedback_log_time < 0.25:
            return
        self._last_feedback_log_time = now
        self.get_logger().info(
            "Goal marker %s: x=%.3f y=%.3f z=%.3f"
            % (event_name, pose.position.x, pose.position.y, pose.position.z)
        )

    def _publish_goal_cb(self, feedback: InteractiveMarkerFeedback) -> None:
        self.goal_pose = copy.deepcopy(feedback.pose)
        self.publish_visual_markers()
        self.publish_goal()

    def publish_goal(self, log_level: str = "info") -> None:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose = copy.deepcopy(self.goal_pose)
        self.goal_pub.publish(msg)
        if self.goal_alias_pub is not None:
            self.goal_alias_pub.publish(msg)

        log = self.get_logger().debug if log_level == "debug" else self.get_logger().info
        log(
            "Published goal pose: x=%.3f y=%.3f z=%.3f"
            % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        )

    def publish_visual_markers(self) -> None:
        if not self.publish_marker_array_enabled:
            return
        marker_array = MarkerArray()
        marker_array.markers.append(self._make_preview_marker(0, Marker.SPHERE))
        marker_array.markers.append(self._make_preview_marker(1, Marker.ARROW))
        self.marker_array_pub.publish(marker_array)

    def _make_preview_marker(self, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = "goal"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(self.goal_pose)
        marker.color.r = 1.0
        marker.color.g = 0.05
        marker.color.b = 0.05
        marker.color.a = 0.85
        if marker_type == Marker.SPHERE:
            diameter = self.sphere_radius * 2.4
            marker.scale.x = diameter
            marker.scale.y = diameter
            marker.scale.z = diameter
        else:
            marker.scale.x = self.sphere_radius * 1.8
            marker.scale.y = self.sphere_radius * 0.35
            marker.scale.z = self.sphere_radius * 0.35
        return marker

    def destroy_node(self):
        if hasattr(self, "server") and self.server is not None:
            self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PCTGoalMarkerNode()
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
