import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


class CmdVelGate(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_gate_node")

        self.declare_parameter("input_cmd_vel_topic", "/cmd_vel_nav")
        self.declare_parameter("output_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("execution_state_topic", "/nav3d/execution_state")
        self.declare_parameter("pass_states", ["RUNNING"])
        self.declare_parameter("zero_publish_period_s", 0.2)
        self.declare_parameter("publish_zero_on_blocked_command", True)

        self.input_cmd_vel_topic = str(self.get_parameter("input_cmd_vel_topic").value)
        self.output_cmd_vel_topic = str(self.get_parameter("output_cmd_vel_topic").value)
        self.execution_state_topic = str(self.get_parameter("execution_state_topic").value)
        self.pass_states = {
            str(state).strip().upper()
            for state in self.get_parameter("pass_states").value
            if str(state).strip()
        }
        self.zero_publish_period_s = float(self.get_parameter("zero_publish_period_s").value)
        self.publish_zero_on_blocked_command = as_bool(
            self.get_parameter("publish_zero_on_blocked_command").value
        )

        self.current_state = "IDLE"
        self.last_published_zero = False

        self.cmd_pub = self.create_publisher(Twist, self.output_cmd_vel_topic, 10)
        self.cmd_sub = self.create_subscription(Twist, self.input_cmd_vel_topic, self._cmd_callback, 10)
        self.state_sub = self.create_subscription(
            String,
            self.execution_state_topic,
            self._state_callback,
            10,
        )

        if self.zero_publish_period_s > 0.0:
            self.zero_timer = self.create_timer(self.zero_publish_period_s, self._zero_timer_callback)
        else:
            self.zero_timer = None

        self.get_logger().info(
            "cmd_vel_gate ready. %s -> %s while state in %s"
            % (self.input_cmd_vel_topic, self.output_cmd_vel_topic, sorted(self.pass_states))
        )

    def _is_open(self) -> bool:
        return self.current_state.upper() in self.pass_states

    def _state_callback(self, msg: String) -> None:
        previous_open = self._is_open()
        self.current_state = msg.data.strip().upper() or "IDLE"
        if previous_open and not self._is_open():
            self._publish_zero()

    def _cmd_callback(self, msg: Twist) -> None:
        if self._is_open():
            self.cmd_pub.publish(msg)
            self.last_published_zero = False
            return
        if self.publish_zero_on_blocked_command:
            self._publish_zero()

    def _zero_timer_callback(self) -> None:
        if not self._is_open():
            self._publish_zero()

    def _publish_zero(self) -> None:
        self.cmd_pub.publish(Twist())
        self.last_published_zero = True


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
