from enum import Enum

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ExecutionState(str, Enum):
    IDLE = "IDLE"
    PATH_READY = "PATH_READY"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    STOPPED = "STOPPED"
    GOAL_REACHED = "GOAL_REACHED"
    ERROR = "ERROR"


def latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        depth=depth,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class NavExecutionController(Node):
    def __init__(self) -> None:
        super().__init__("nav_execution_controller_node")

        self.declare_parameter("planned_path_topic", "/planned_path")
        self.declare_parameter("path_clear_topic", "/planned_path")
        self.declare_parameter("execution_state_topic", "/nav3d/execution_state")
        self.declare_parameter("allow_start_without_path", False)
        self.declare_parameter("auto_start_on_new_path", False)

        self.planned_path_topic = str(self.get_parameter("planned_path_topic").value)
        self.path_clear_topic = str(self.get_parameter("path_clear_topic").value)
        self.execution_state_topic = str(self.get_parameter("execution_state_topic").value)
        self.allow_start_without_path = as_bool(self.get_parameter("allow_start_without_path").value)
        self.auto_start_on_new_path = as_bool(self.get_parameter("auto_start_on_new_path").value)

        self.state = ExecutionState.IDLE
        self.have_path = False
        self.last_path_header = None

        self.state_pub = self.create_publisher(String, self.execution_state_topic, latched_qos())
        self.path_clear_pub = self.create_publisher(Path, self.path_clear_topic, latched_qos())
        self.path_sub = self.create_subscription(
            Path,
            self.planned_path_topic,
            self._path_callback,
            latched_qos(),
        )

        self.create_service(Trigger, "/nav3d/start", self._handle_start)
        self.create_service(Trigger, "/nav3d/stop", self._handle_stop)
        self.create_service(Trigger, "/nav3d/pause", self._handle_pause)
        self.create_service(Trigger, "/nav3d/resume", self._handle_resume)
        self.create_service(Trigger, "/nav3d/clear_path", self._handle_clear_path)

        self._publish_state()
        self.get_logger().info(
            "nav_execution_controller ready. path=%s state=%s"
            % (self.planned_path_topic, self.execution_state_topic)
        )

    def _set_state(self, state: ExecutionState, reason: str = "") -> None:
        if self.state == state:
            self._publish_state()
            return
        self.state = state
        if reason:
            self.get_logger().info("Execution state -> %s (%s)" % (state.value, reason))
        else:
            self.get_logger().info("Execution state -> %s" % state.value)
        self._publish_state()

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self.state.value
        self.state_pub.publish(msg)

    def _path_callback(self, msg: Path) -> None:
        if not msg.poses:
            self.have_path = False
            if self.state not in (ExecutionState.IDLE, ExecutionState.STOPPED):
                self._set_state(ExecutionState.IDLE, "received empty path")
            return

        self.have_path = True
        self.last_path_header = msg.header
        if self.auto_start_on_new_path:
            self._set_state(ExecutionState.RUNNING, "new path")
            return
        self._set_state(ExecutionState.PATH_READY, "new path")

    def _handle_start(self, request, response):
        del request
        if not self.have_path and not self.allow_start_without_path:
            response.success = False
            response.message = "No planned path is available."
            self._publish_state()
            return response
        self._set_state(ExecutionState.RUNNING, "start service")
        response.success = True
        response.message = "Navigation execution started."
        return response

    def _handle_stop(self, request, response):
        del request
        self._set_state(ExecutionState.STOPPED, "stop service")
        response.success = True
        response.message = "Navigation execution stopped."
        return response

    def _handle_pause(self, request, response):
        del request
        if self.state != ExecutionState.RUNNING:
            response.success = False
            response.message = "Navigation is not running."
            self._publish_state()
            return response
        self._set_state(ExecutionState.PAUSED, "pause service")
        response.success = True
        response.message = "Navigation execution paused."
        return response

    def _handle_resume(self, request, response):
        del request
        if self.state != ExecutionState.PAUSED:
            response.success = False
            response.message = "Navigation is not paused."
            self._publish_state()
            return response
        if not self.have_path and not self.allow_start_without_path:
            response.success = False
            response.message = "No planned path is available."
            self._publish_state()
            return response
        self._set_state(ExecutionState.RUNNING, "resume service")
        response.success = True
        response.message = "Navigation execution resumed."
        return response

    def _handle_clear_path(self, request, response):
        del request
        self.have_path = False
        self._publish_empty_path()
        self._set_state(ExecutionState.IDLE, "clear_path service")
        response.success = True
        response.message = "Path cleared."
        return response

    def _publish_empty_path(self) -> None:
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.last_path_header.frame_id if self.last_path_header else "map"
        self.path_clear_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavExecutionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
