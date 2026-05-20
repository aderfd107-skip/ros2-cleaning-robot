#!/usr/bin/env python3

import math

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class InitialPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_publisher")

        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_delay", 4.0)
        self.declare_parameter("publish_period", 2.0)
        self.declare_parameter("max_attempts", 30)
        self.declare_parameter("xy_tolerance", 0.20)
        self.declare_parameter("yaw_tolerance", 0.35)
        self.declare_parameter("required_stable_count", 3)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._on_map,
            map_qos,
        )
        self.create_subscription(
            LaserScan,
            self.get_parameter("scan_topic").value,
            self._on_scan,
            scan_qos,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("amcl_pose_topic").value,
            self._on_amcl_pose,
            10,
        )

        self.initial_pose_confirmed = False
        self.publish_started = False
        self.delay_elapsed = False
        self.attempt_count = 0
        self.stable_alignment_count = 0
        self.map_received = False
        self.scan_received = False
        self.publish_delay = max(0.0, float(self.get_parameter("publish_delay").value))
        self.publish_period = max(0.5, float(self.get_parameter("publish_period").value))
        self.max_attempts = max(1, int(self.get_parameter("max_attempts").value))
        self.xy_tolerance = max(0.01, float(self.get_parameter("xy_tolerance").value))
        self.yaw_tolerance = max(0.01, float(self.get_parameter("yaw_tolerance").value))
        self.required_stable_count = max(
            1, int(self.get_parameter("required_stable_count").value)
        )
        self.target_x = float(self.get_parameter("x").value)
        self.target_y = float(self.get_parameter("y").value)
        self.target_yaw = float(self.get_parameter("yaw").value)

        self.delay_timer = self.create_timer(self.publish_delay, self._on_delay_elapsed)
        self.readiness_timer = self.create_timer(0.5, self._try_start_publishing)
        self.publish_timer = None
        self.get_logger().info(
            "Initial pose publisher will start in %.1fs at (%.2f, %.2f, %.2f rad) once /map and /scan are ready."
            % (
                self.publish_delay,
                self.target_x,
                self.target_y,
                self.target_yaw,
            )
        )

    def _on_delay_elapsed(self) -> None:
        if self.delay_elapsed:
            return
        self.delay_elapsed = True
        self.delay_timer.cancel()

    def _try_start_publishing(self) -> None:
        if self.publish_started or not self.delay_elapsed:
            return
        if not self.map_received or not self.scan_received:
            return
        self.publish_started = True
        self.get_logger().info(
            "Initial pose prerequisites ready. Starting /initialpose publication."
        )
        self._publish_initial_pose()
        self.publish_timer = self.create_timer(self.publish_period, self._publish_initial_pose)

    def _on_map(self, _: OccupancyGrid) -> None:
        if self.map_received:
            return
        self.map_received = True
        self.get_logger().info("Received /map. Initial localization can use the saved map.")

    def _on_scan(self, _: LaserScan) -> None:
        if self.scan_received:
            return
        self.scan_received = True
        self.get_logger().info("Received /scan. Laser data is ready for AMCL.")

    def _publish_initial_pose(self) -> None:
        if self.initial_pose_confirmed:
            return

        if self.attempt_count >= self.max_attempts:
            self.get_logger().warn(
                "Initial pose was published %d times but AMCL never aligned with the requested spawn pose."
                % self.max_attempts
            )
            if self.publish_timer is not None:
                self.publish_timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.get_parameter("frame_id").value
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.target_x
        msg.pose.pose.position.y = self.target_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(self.target_yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.target_yaw * 0.5)

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06853891945200942

        self.pose_pub.publish(msg)
        self.attempt_count += 1
        self.get_logger().info(
            "Published initial pose attempt %d/%d." % (self.attempt_count, self.max_attempts)
        )

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        if self.initial_pose_confirmed:
            return

        if not self.publish_started:
            return

        if not self._is_pose_aligned(msg):
            if self.stable_alignment_count != 0:
                self.get_logger().info(
                    "AMCL pose drifted away from the requested spawn pose. Resetting alignment counter."
                )
            self.stable_alignment_count = 0
            return

        self.stable_alignment_count += 1
        self.get_logger().info(
            "AMCL pose alignment %d/%d confirmed."
            % (self.stable_alignment_count, self.required_stable_count)
        )
        if self.stable_alignment_count < self.required_stable_count:
            return

        self.initial_pose_confirmed = True
        if self.publish_timer is not None:
            self.publish_timer.cancel()
        self.readiness_timer.cancel()
        self.get_logger().info(
            "Initial localization is aligned with the requested spawn pose."
        )

    def _is_pose_aligned(self, msg: PoseWithCovarianceStamped) -> bool:
        dx = float(msg.pose.pose.position.x) - self.target_x
        dy = float(msg.pose.pose.position.y) - self.target_y
        distance = math.hypot(dx, dy)
        yaw = self._yaw_from_pose(msg)
        yaw_error = abs(self._normalize_angle(yaw - self.target_yaw))
        return distance <= self.xy_tolerance and yaw_error <= self.yaw_tolerance

    def _yaw_from_pose(self, msg: PoseWithCovarianceStamped) -> float:
        z = float(msg.pose.pose.orientation.z)
        w = float(msg.pose.pose.orientation.w)
        return 2.0 * math.atan2(z, w)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main() -> None:
    rclpy.init()
    node = InitialPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
