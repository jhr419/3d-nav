import math

import numpy as np
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from .config import PlannerConfig, ROSConfig
from .planner_wrapper import TomogramPlanner
from .scenes import get_plan_defaults
from .utils import traj_to_path


def latched_qos():
    return QoSProfile(
        depth=1,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        reliability=QoSReliabilityPolicy.RELIABLE,
    )


def _override_pose(default_value, x_value, y_value, z_value=float('nan')):
    if math.isnan(float(x_value)) or math.isnan(float(y_value)):
        return default_value
    if math.isnan(float(z_value)):
        return np.array([float(x_value), float(y_value)], dtype=np.float32)
    return np.array([float(x_value), float(y_value), float(z_value)], dtype=np.float32)


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


def _pose_xyz(pose_msg):
    return np.array(
        [
            float(pose_msg.pose.position.x),
            float(pose_msg.pose.position.y),
            float(pose_msg.pose.position.z),
        ],
        dtype=np.float32,
    )


def _format_pose(pos):
    if len(pos) >= 3:
        return '[%.2f, %.2f, %.2f]' % (pos[0], pos[1], pos[2])
    return '[%.2f, %.2f]' % (pos[0], pos[1])


class PlannerNode(Node):
    def __init__(self):
        super().__init__('pct_planner')

        self.declare_parameter('scene', 'Spiral')
        self.declare_parameter('tomogram_file', '')
        self.declare_parameter('tomogram_dir', PlannerConfig.tomogram_dir)
        self.declare_parameter('planner_lib_dir', PlannerConfig.planner_lib_dir)
        self.declare_parameter('use_quintic', PlannerConfig.use_quintic)
        self.declare_parameter('max_heading_rate', PlannerConfig.max_heading_rate)
        self.declare_parameter('path_z_offset', PlannerConfig.path_z_offset)
        self.declare_parameter('astar_cost_threshold', PlannerConfig.astar_cost_threshold)
        self.declare_parameter('astar_step_cost_weight', PlannerConfig.astar_step_cost_weight)
        self.declare_parameter('optimizer_safe_cost_threshold', PlannerConfig.optimizer_safe_cost_threshold)
        self.declare_parameter('max_path_z_jump', PlannerConfig.max_path_z_jump)
        self.declare_parameter('map_frame', ROSConfig.map_frame)
        self.declare_parameter('path_topic', ROSConfig.path_topic)
        self.declare_parameter('start_x', float('nan'))
        self.declare_parameter('start_y', float('nan'))
        self.declare_parameter('start_z', float('nan'))
        self.declare_parameter('goal_x', float('nan'))
        self.declare_parameter('goal_y', float('nan'))
        self.declare_parameter('goal_z', float('nan'))
        self.declare_parameter('start_pose_topic', '/pct_planner/start_pose')
        self.declare_parameter('goal_pose_topic', '/pct_planner/goal_pose')
        self.declare_parameter('plan_on_pose_update', True)
        self.declare_parameter('auto_run', True)

        self.scene_name = self.get_parameter('scene').value
        defaults = get_plan_defaults(self.scene_name)

        tomogram_file = self.get_parameter('tomogram_file').value
        self.tomogram_file = tomogram_file if tomogram_file else defaults.tomogram_file
        self.start_pos = _override_pose(
            defaults.start_pos,
            self.get_parameter('start_x').value,
            self.get_parameter('start_y').value,
            self.get_parameter('start_z').value,
        )
        self.goal_pos = _override_pose(
            defaults.goal_pos,
            self.get_parameter('goal_x').value,
            self.get_parameter('goal_y').value,
            self.get_parameter('goal_z').value,
        )
        self.map_frame = self.get_parameter('map_frame').value
        self.start_pose_topic = self.get_parameter('start_pose_topic').value
        self.goal_pose_topic = self.get_parameter('goal_pose_topic').value
        self.plan_on_pose_update = _as_bool(self.get_parameter('plan_on_pose_update').value)
        self.start_pose_received = False
        self.goal_pose_received = False
        self.tomogram_loaded = False

        planner_cfg = PlannerConfig(
            tomogram_dir=self.get_parameter('tomogram_dir').value,
            planner_lib_dir=self.get_parameter('planner_lib_dir').value,
            use_quintic=_as_bool(self.get_parameter('use_quintic').value),
            max_heading_rate=float(self.get_parameter('max_heading_rate').value),
            path_z_offset=float(self.get_parameter('path_z_offset').value),
            astar_cost_threshold=float(self.get_parameter('astar_cost_threshold').value),
            astar_step_cost_weight=float(self.get_parameter('astar_step_cost_weight').value),
            optimizer_safe_cost_threshold=float(self.get_parameter('optimizer_safe_cost_threshold').value),
            max_path_z_jump=float(self.get_parameter('max_path_z_jump').value),
        )
        self.path_pub = self.create_publisher(Path, self.get_parameter('path_topic').value, latched_qos())
        self.planner = TomogramPlanner(planner_cfg, logger=self.get_logger())
        self.start_pose_sub = self.create_subscription(
            PoseStamped,
            self.start_pose_topic,
            self.start_pose_callback,
            latched_qos(),
        )
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self.goal_pose_callback,
            latched_qos(),
        )
        self.get_logger().info(
            'Listening for interactive start/goal poses on "%s" and "%s"'
            % (self.start_pose_topic, self.goal_pose_topic)
        )

        if _as_bool(self.get_parameter('auto_run').value):
            self.plan_once()

    def start_pose_callback(self, msg):
        self.start_pos = _pose_xyz(msg)
        self.start_pose_received = True
        self.get_logger().info(
            'Received start pose: x=%.2f y=%.2f z=%.2f'
            % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        )
        if self.plan_on_pose_update:
            self.plan_if_ready()

    def goal_pose_callback(self, msg):
        self.goal_pos = _pose_xyz(msg)
        self.goal_pose_received = True
        self.get_logger().info(
            'Received goal pose: x=%.2f y=%.2f z=%.2f'
            % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        )
        if self.plan_on_pose_update:
            self.plan_if_ready()

    def plan_if_ready(self):
        if not (self.start_pose_received and self.goal_pose_received):
            return
        self.plan_once()

    def plan_once(self):
        if not self.tomogram_loaded:
            self.get_logger().info(f'Loading tomogram: {self.tomogram_file}')
            self.planner.load_tomogram(self.tomogram_file)
            self.tomogram_loaded = True

        self.get_logger().info(
            'Planning from %s to %s'
            % (_format_pose(self.start_pos), _format_pose(self.goal_pos))
        )
        traj_3d = self.planner.plan(self.start_pos, self.goal_pos)
        if traj_3d is None:
            self.get_logger().error('Planning failed: no path found')
            return

        stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(traj_to_path(traj_3d, frame_id=self.map_frame, stamp=stamp))
        self.get_logger().info('Trajectory published')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PlannerNode()
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
