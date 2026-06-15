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

from .scenes import get_plan_defaults


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


def quaternion_from_rpy(roll, pitch, yaw):
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


def make_pose(x, y, z, roll, pitch, yaw):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    pose.orientation = quaternion_from_rpy(float(roll), float(pitch), float(yaw))
    return pose


class StartGoalMarkerNode(Node):
    def __init__(self):
        super().__init__('pct_start_goal_marker')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('start_pose_topic', '/pct_planner/start_pose')
        self.declare_parameter('goal_pose_topic', '/pct_planner/goal_pose')
        self.declare_parameter('marker_array_topic', '/pct_planner/start_goal_markers')
        self.declare_parameter('server_namespace', 'pct_start_goal_marker')
        self.declare_parameter('scene', '')
        self.declare_parameter('use_scene_defaults', False)
        self.declare_parameter('marker_scale', 0.8)
        self.declare_parameter('sphere_radius', 0.35)
        self.declare_parameter('publish_on_feedback', True)
        self.declare_parameter('publish_initial_poses', True)
        self.declare_parameter('publish_marker_array', True)

        self.declare_parameter('default_start_x', 0.0)
        self.declare_parameter('default_start_y', 0.0)
        self.declare_parameter('default_start_z', 0.0)
        self.declare_parameter('default_start_roll', 0.0)
        self.declare_parameter('default_start_pitch', 0.0)
        self.declare_parameter('default_start_yaw', 0.0)
        self.declare_parameter('default_goal_x', 5.0)
        self.declare_parameter('default_goal_y', 0.0)
        self.declare_parameter('default_goal_z', 0.0)
        self.declare_parameter('default_goal_roll', 0.0)
        self.declare_parameter('default_goal_pitch', 0.0)
        self.declare_parameter('default_goal_yaw', 0.0)

        self.frame_id = self.get_parameter('frame_id').value
        self.start_pose_topic = self.get_parameter('start_pose_topic').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.marker_array_topic = self.get_parameter('marker_array_topic').value
        self.server_namespace = self.get_parameter('server_namespace').value
        self.scene_name = self.get_parameter('scene').value
        self.use_scene_defaults = as_bool(self.get_parameter('use_scene_defaults').value)
        self.marker_scale = float(self.get_parameter('marker_scale').value)
        self.sphere_radius = float(self.get_parameter('sphere_radius').value)
        self.publish_on_feedback = as_bool(self.get_parameter('publish_on_feedback').value)
        self.publish_initial_poses = as_bool(self.get_parameter('publish_initial_poses').value)
        self.publish_marker_array = as_bool(self.get_parameter('publish_marker_array').value)

        self.marker_names = {
            'start': 'pct_start_pose',
            'goal': 'pct_goal_pose',
        }
        self.poses = {
            'start': self._default_pose('start'),
            'goal': self._default_pose('goal'),
        }

        self.start_pub = self.create_publisher(PoseStamped, self.start_pose_topic, latched_qos())
        self.goal_pub = self.create_publisher(PoseStamped, self.goal_pose_topic, latched_qos())
        self.marker_array_pub = self.create_publisher(MarkerArray, self.marker_array_topic, latched_qos())
        self.pose_publishers = {
            'start': self.start_pub,
            'goal': self.goal_pub,
        }

        self.server = InteractiveMarkerServer(self, self.server_namespace)
        self.menu_handler = self._make_menu_handler()

        self._insert_marker('start')
        self._insert_marker('goal')
        self.server.applyChanges()
        self.publish_visual_markers()

        self.get_logger().info(
            'Interactive start/goal markers ready in frame "%s" on /%s/update'
            % (self.frame_id, self.server_namespace.strip('/'))
        )
        self.get_logger().info(
            'Publishing start to "%s" and goal to "%s"'
            % (self.start_pose_topic, self.goal_pose_topic)
        )

        if self.publish_initial_poses:
            self.publish_both()

    def _default_pose(self, role):
        if self.use_scene_defaults and self.scene_name:
            try:
                defaults = get_plan_defaults(self.scene_name)
                default_pos = defaults.start_pos if role == 'start' else defaults.goal_pos
                return make_pose(
                    default_pos[0],
                    default_pos[1],
                    self.get_parameter(f'default_{role}_z').value,
                    self.get_parameter(f'default_{role}_roll').value,
                    self.get_parameter(f'default_{role}_pitch').value,
                    self.get_parameter(f'default_{role}_yaw').value,
                )
            except ValueError as exc:
                self.get_logger().warning(str(exc))

        return make_pose(
            self.get_parameter(f'default_{role}_x').value,
            self.get_parameter(f'default_{role}_y').value,
            self.get_parameter(f'default_{role}_z').value,
            self.get_parameter(f'default_{role}_roll').value,
            self.get_parameter(f'default_{role}_pitch').value,
            self.get_parameter(f'default_{role}_yaw').value,
        )

    def _make_menu_handler(self):
        menu_handler = MenuHandler()
        menu_handler.insert('Set as Start', callback=self._set_as_start_cb)
        menu_handler.insert('Set as Goal', callback=self._set_as_goal_cb)
        menu_handler.insert('Publish Start', callback=self._publish_start_cb)
        menu_handler.insert('Publish Goal', callback=self._publish_goal_cb)
        menu_handler.insert('Publish Both', callback=self._publish_both_cb)
        return menu_handler

    def _insert_marker(self, role):
        marker = self._make_interactive_marker(role)
        self.server.insert(marker, feedback_callback=self._feedback_cb)
        self.menu_handler.apply(self.server, marker.name)

    def _make_interactive_marker(self, role):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.name = self.marker_names[role]
        marker.description = 'Start Pose' if role == 'start' else 'Goal Pose'
        marker.scale = self.marker_scale
        marker.pose = copy.deepcopy(self.poses[role])

        sphere_control = InteractiveMarkerControl()
        sphere_control.name = f'{role}_sphere_menu'
        sphere_control.always_visible = True
        sphere_control.interaction_mode = InteractiveMarkerControl.MENU
        sphere_control.markers.append(self._make_sphere_marker(role))
        marker.controls.append(sphere_control)

        self._add_axis_controls(marker)
        return marker

    def _make_sphere_marker(self, role):
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = self.sphere_radius * 2.0
        marker.scale.y = self.sphere_radius * 2.0
        marker.scale.z = self.sphere_radius * 2.0
        marker.color.a = 0.9
        if role == 'start':
            marker.color.r = 0.0
            marker.color.g = 0.85
            marker.color.b = 0.2
        else:
            marker.color.r = 0.95
            marker.color.g = 0.1
            marker.color.b = 0.1
        return marker

    def _add_axis_controls(self, marker):
        axes = (
            ('x', 1.0, 0.0, 0.0),
            ('y', 0.0, 1.0, 0.0),
            ('z', 0.0, 0.0, 1.0),
        )
        for axis_name, x, y, z in axes:
            rotate = self._make_axis_control(
                name=f'rotate_{axis_name}',
                interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                x=x,
                y=y,
                z=z,
            )
            marker.controls.append(rotate)

            move = self._make_axis_control(
                name=f'move_{axis_name}',
                interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
                x=x,
                y=y,
                z=z,
            )
            marker.controls.append(move)

    def _make_axis_control(self, name, interaction_mode, x, y, z):
        control = InteractiveMarkerControl()
        control.name = name
        control.orientation.w = 1.0
        control.orientation.x = x
        control.orientation.y = y
        control.orientation.z = z
        control.orientation_mode = InteractiveMarkerControl.FIXED
        control.interaction_mode = interaction_mode
        return control

    def _feedback_cb(self, feedback):
        role = self._role_from_marker_name(feedback.marker_name)
        if role is None:
            self.get_logger().warning('Ignoring feedback for unknown marker "%s"' % feedback.marker_name)
            return

        if feedback.event_type in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
            InteractiveMarkerFeedback.BUTTON_CLICK,
        ):
            self.poses[role] = copy.deepcopy(feedback.pose)
            self.publish_visual_markers()

        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE and self.publish_on_feedback:
            self.publish_pose(role, log_level='debug')

    def _set_as_start_cb(self, feedback):
        self._set_role_pose('start', feedback.pose)
        self.publish_pose('start')

    def _set_as_goal_cb(self, feedback):
        self._set_role_pose('goal', feedback.pose)
        self.publish_pose('goal')

    def _publish_start_cb(self, feedback):
        self._remember_feedback_pose(feedback)
        self.publish_pose('start')

    def _publish_goal_cb(self, feedback):
        self._remember_feedback_pose(feedback)
        self.publish_pose('goal')

    def _publish_both_cb(self, feedback):
        self._remember_feedback_pose(feedback)
        self.publish_both()

    def _remember_feedback_pose(self, feedback):
        role = self._role_from_marker_name(feedback.marker_name)
        if role is not None:
            self.poses[role] = copy.deepcopy(feedback.pose)
            self.publish_visual_markers()

    def _set_role_pose(self, role, pose):
        self.poses[role] = copy.deepcopy(pose)
        self.server.setPose(self.marker_names[role], self.poses[role])
        self.server.applyChanges()
        self.publish_visual_markers()
        self.get_logger().info('Updated %s pose from marker menu' % role)

    def _role_from_marker_name(self, marker_name):
        for role, name in self.marker_names.items():
            if name == marker_name:
                return role
        return None

    def publish_pose(self, role, log_level='info'):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose = copy.deepcopy(self.poses[role])
        self.pose_publishers[role].publish(msg)

        log = self.get_logger().debug if log_level == 'debug' else self.get_logger().info
        log(
            'Published %s pose: x=%.3f y=%.3f z=%.3f'
            % (role, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        )

    def publish_both(self):
        self.publish_pose('start')
        self.publish_pose('goal')

    def publish_visual_markers(self):
        if not self.publish_marker_array:
            return

        marker_array = MarkerArray()
        marker_array.markers.extend([
            self._make_preview_sphere('start', 0),
            self._make_preview_arrow('start', 1),
            self._make_preview_sphere('goal', 2),
            self._make_preview_arrow('goal', 3),
        ])
        self.marker_array_pub.publish(marker_array)

    def _make_preview_sphere(self, role, marker_id):
        marker = self._make_preview_marker(role, marker_id, Marker.SPHERE)
        diameter = self.sphere_radius * 2.4
        marker.scale.x = diameter
        marker.scale.y = diameter
        marker.scale.z = diameter
        return marker

    def _make_preview_arrow(self, role, marker_id):
        marker = self._make_preview_marker(role, marker_id, Marker.ARROW)
        marker.scale.x = self.sphere_radius * 1.8
        marker.scale.y = self.sphere_radius * 0.35
        marker.scale.z = self.sphere_radius * 0.35
        return marker

    def _make_preview_marker(self, role, marker_id, marker_type):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.frame_id
        marker.ns = role
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(self.poses[role])
        marker.color.a = 0.85
        if role == 'start':
            marker.color.r = 0.0
            marker.color.g = 0.95
            marker.color.b = 0.15
        else:
            marker.color.r = 1.0
            marker.color.g = 0.05
            marker.color.b = 0.05
        return marker

    def destroy_node(self):
        if hasattr(self, 'server') and self.server is not None:
            self.server.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StartGoalMarkerNode()
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
