#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node


class InitialPose3DBridge(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_3d_bridge")

        self.declare_parameter("input_topic", "/initial_pose_3d")
        self.declare_parameter("output_topic", "/initialpose")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("covariance_xy", 0.25)
        self.declare_parameter("covariance_z", 0.25)
        self.declare_parameter("covariance_yaw", 0.0685)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value

        self.publisher = self.create_publisher(PoseWithCovarianceStamped, output_topic, 10)
        self.create_subscription(PoseStamped, input_topic, self._on_pose, 10)

        self.get_logger().info(
            f"Initial pose bridge started. {input_topic} -> {output_topic}"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self.get_parameter("map_frame").value
        out.pose.pose = msg.pose

        covariance_xy = float(self.get_parameter("covariance_xy").value)
        covariance_z = float(self.get_parameter("covariance_z").value)
        covariance_yaw = float(self.get_parameter("covariance_yaw").value)
        out.pose.covariance[0] = covariance_xy
        out.pose.covariance[7] = covariance_xy
        out.pose.covariance[14] = covariance_z
        out.pose.covariance[35] = covariance_yaw

        self.publisher.publish(out)
        self.get_logger().info(
            "Forwarded 3D initial pose to /initialpose: "
            f"[{out.pose.pose.position.x:.3f}, "
            f"{out.pose.pose.position.y:.3f}, "
            f"{out.pose.pose.position.z:.3f}]"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPose3DBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
