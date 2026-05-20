import math
from collections import deque

from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from tf2_ros import Buffer, LookupException, TransformException, TransformListener


class CoverageProgressPublisher(Node):
    def __init__(self):
        super().__init__("coverage_progress_publisher")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("progress_topic", "/coverage_progress")
        self.declare_parameter("coverage_map_topic", "/coverage_map")
        self.declare_parameter("robot_frame", "base_footprint")
        self.declare_parameter("coverage_radius", 0.22)
        self.declare_parameter("publish_period", 1.0)

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.progress_pub = self.create_publisher(
            Float32, self.get_parameter("progress_topic").value, latched_qos
        )
        self.coverage_map_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter("coverage_map_topic").value, latched_qos
        )

        self.create_subscription(
            OccupancyGrid,
            self.get_parameter("map_topic").value,
            self._on_map,
            latched_qos,
        )

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.map_msg = None
        self.raw_free_cell_indices = set()
        self.reachable_free_cell_indices = set()
        self.covered_cell_indices = set()
        self.coverage_map_msg = None
        self.map_frame = None
        self.resolution = 0.0
        self.coverage_radius = float(self.get_parameter("coverage_radius").value)
        self.coverage_radius_cells = 0
        self.last_progress_percent = -1.0
        self.reachable_region_initialized = False
        self._lookup_warned = False

        period = max(0.1, float(self.get_parameter("publish_period").value))
        self.create_timer(period, self._update_progress)
        self.get_logger().info("Coverage progress publisher started.")

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg
        self.map_frame = msg.header.frame_id or "map"
        self.resolution = msg.info.resolution
        self.coverage_radius_cells = max(
            0, int(math.ceil(self.coverage_radius / self.resolution))
        )

        self.raw_free_cell_indices.clear()
        self.reachable_free_cell_indices.clear()
        self.covered_cell_indices.clear()
        self.reachable_region_initialized = False
        self.coverage_map_msg = OccupancyGrid()
        self.coverage_map_msg.header.frame_id = self.map_frame
        self.coverage_map_msg.info = msg.info
        self.coverage_map_msg.data = [-1] * len(msg.data)

        for index, value in enumerate(msg.data):
            if value == 0:
                self.raw_free_cell_indices.add(index)

        self._publish_progress(force=True)
        self.get_logger().info(
            "Loaded map for coverage tracking with %d raw free cells."
            % len(self.raw_free_cell_indices)
        )

    def _update_progress(self) -> None:
        if self.map_msg is None or not self.raw_free_cell_indices:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.get_parameter("robot_frame").value,
                rclpy.time.Time(),
            )
        except (LookupException, TransformException):
            if not self._lookup_warned:
                self.get_logger().warn(
                    "Waiting for transform %s -> %s"
                    % (self.map_frame, self.get_parameter("robot_frame").value)
                )
                self._lookup_warned = True
            return

        self._lookup_warned = False
        robot_point = transform.transform.translation
        center = self._world_to_cell(robot_point.x, robot_point.y)
        if center is None:
            self._publish_progress(force=False)
            return

        if not self.reachable_region_initialized:
            self._initialize_reachable_region(center)
            if not self.reachable_free_cell_indices:
                self._publish_progress(force=True)
                return

        updated = self._mark_covered(center)
        self._publish_progress(force=updated)

    def _initialize_reachable_region(self, center) -> None:
        if self.map_msg is None:
            return

        center_x, center_y = center
        width = self.map_msg.info.width
        start_index = center_y * width + center_x

        if start_index not in self.raw_free_cell_indices:
            self.get_logger().warn(
                "Robot start cell (%d, %d) is not free in /map; reachable coverage region not initialized yet."
                % (center_x, center_y)
            )
            return

        reachable = set()
        queue = deque([start_index])
        reachable.add(start_index)

        while queue:
            index = queue.popleft()
            x = index % width
            y = index // width

            for nx, ny in self._neighbors4(x, y):
                neighbor_index = ny * width + nx
                if (
                    neighbor_index in reachable
                    or neighbor_index not in self.raw_free_cell_indices
                ):
                    continue
                reachable.add(neighbor_index)
                queue.append(neighbor_index)

        self.reachable_free_cell_indices = reachable
        self.reachable_region_initialized = True
        self.coverage_map_msg.data = [-1] * len(self.map_msg.data)
        for index in self.reachable_free_cell_indices:
            self.coverage_map_msg.data[index] = 0

        self.get_logger().info(
            "Initialized reachable coverage region with %d/%d free cells."
            % (len(self.reachable_free_cell_indices), len(self.raw_free_cell_indices))
        )

    def _mark_covered(self, center) -> bool:
        if self.map_msg is None:
            return False

        center_x, center_y = center
        width = self.map_msg.info.width
        updated = False

        for dy in range(-self.coverage_radius_cells, self.coverage_radius_cells + 1):
            for dx in range(-self.coverage_radius_cells, self.coverage_radius_cells + 1):
                if math.hypot(dx, dy) * self.resolution > self.coverage_radius:
                    continue
                mx = center_x + dx
                my = center_y + dy
                if (
                    mx < 0
                    or my < 0
                    or mx >= self.map_msg.info.width
                    or my >= self.map_msg.info.height
                ):
                    continue
                index = my * width + mx
                if (
                    index not in self.reachable_free_cell_indices
                    or index in self.covered_cell_indices
                ):
                    continue
                self.covered_cell_indices.add(index)
                self.coverage_map_msg.data[index] = 100
                updated = True

        return updated

    def _publish_progress(self, force: bool) -> None:
        if self.map_msg is None:
            return

        denominator = len(self.reachable_free_cell_indices)
        if denominator == 0:
            progress_percent = 0.0
        else:
            progress_percent = 100.0 * len(self.covered_cell_indices) / float(denominator)

        if force or abs(progress_percent - self.last_progress_percent) >= 1e-3:
            msg = Float32()
            msg.data = progress_percent
            self.progress_pub.publish(msg)

            self.coverage_map_msg.header.stamp = self.get_clock().now().to_msg()
            self.coverage_map_pub.publish(self.coverage_map_msg)
            self.last_progress_percent = progress_percent

    def _world_to_cell(self, world_x: float, world_y: float):
        if self.map_msg is None:
            return None

        origin = self.map_msg.info.origin
        yaw = quaternion_to_yaw(origin.orientation)
        dx = world_x - origin.position.x
        dy = world_y - origin.position.y
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw
        mx = int(math.floor(local_x / self.resolution))
        my = int(math.floor(local_y / self.resolution))

        if (
            mx < 0
            or my < 0
            or mx >= self.map_msg.info.width
            or my >= self.map_msg.info.height
        ):
            return None
        return mx, my

    def _neighbors4(self, x: int, y: int):
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < width and 0 <= ny < height:
                yield nx, ny


def quaternion_to_yaw(quaternion) -> float:
    siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)


def main() -> None:
    rclpy.init()
    node = CoverageProgressPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
