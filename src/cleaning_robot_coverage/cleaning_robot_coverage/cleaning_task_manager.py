from copy import deepcopy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class CleaningTaskManager(Node):
    IDLE = "IDLE"
    WAIT_FOR_MAP = "WAIT_FOR_MAP"
    WAIT_FOR_LOCALIZATION = "WAIT_FOR_LOCALIZATION"
    WAIT_FOR_NAVIGATION = "WAIT_FOR_NAVIGATION"
    GENERATE_PATH = "GENERATE_PATH"
    NAVIGATING = "NAVIGATING"
    RECOVERY = "RECOVERY"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"

    def __init__(self):
        super().__init__("cleaning_task_manager")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("waypoints_topic", "/coverage_waypoints")
        self.declare_parameter("coverage_progress_topic", "/coverage_progress")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter(
            "navigation_is_active_service", "/lifecycle_manager_navigation/is_active"
        )
        self.declare_parameter("state_topic", "/cleaning_task_state")
        self.declare_parameter("navigate_to_pose_action", "navigate_to_pose")
        self.declare_parameter("coverage_target_percent", 95.0)
        self.declare_parameter("path_wait_timeout", 30.0)
        self.declare_parameter("single_goal_retry_limit", 2)
        self.declare_parameter("max_consecutive_failed_waypoints", 3)
        self.declare_parameter("skip_failed_waypoints", True)
        self.declare_parameter("recovery_wait_seconds", 2.0)
        self.declare_parameter(
            "clear_local_costmap_service", "/local_costmap/clear_entirely_local_costmap"
        )
        self.declare_parameter(
            "clear_global_costmap_service", "/global_costmap/clear_entirely_global_costmap"
        )

        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.create_subscription(
            OccupancyGrid, self.get_parameter("map_topic").value, self._on_map, latched_qos
        )
        self.create_subscription(
            PoseArray,
            self.get_parameter("waypoints_topic").value,
            self._on_waypoints,
            latched_qos,
        )
        self.create_subscription(
            Float32,
            self.get_parameter("coverage_progress_topic").value,
            self._on_coverage_progress,
            QoSProfile(depth=10),
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self.get_parameter("amcl_pose_topic").value,
            self._on_amcl_pose,
            QoSProfile(depth=10),
        )

        self.state_pub = self.create_publisher(
            String, self.get_parameter("state_topic").value, latched_qos
        )

        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            self.get_parameter("navigate_to_pose_action").value,
        )
        self.clear_local_costmap_client = self.create_client(
            ClearEntireCostmap, self.get_parameter("clear_local_costmap_service").value
        )
        self.clear_global_costmap_client = self.create_client(
            ClearEntireCostmap, self.get_parameter("clear_global_costmap_service").value
        )
        self.navigation_active_client = self.create_client(
            Trigger, self.get_parameter("navigation_is_active_service").value
        )

        self.current_state = self.IDLE
        self.map_received = False
        self.localization_received = False
        self.navigation_ready = False
        self.path_received = False
        self.path_frame = "map"
        self.waypoints = []
        self.current_index = 0
        self.current_goal_handle = None
        self.current_result_future = None
        self.goal_request_pending = False
        self.current_waypoint_failures = 0
        self.consecutive_failed_waypoints = 0
        self.skipped_waypoints = 0
        self.successful_waypoints = 0
        self.coverage_percent = 0.0
        self.recovery_deadline_ns = 0
        self.recovery_reason = ""
        self.recovery_services_requested = False
        self.path_wait_start_ns = 0
        self.mission_started = False
        self.navigation_check_future = None
        self.navigation_check_interval_ns = int(1e9)
        self.last_navigation_check_ns = 0

        self.create_timer(0.5, self._on_timer)
        self.create_timer(1.0, self._publish_state)
        self._publish_state()
        self.get_logger().info("Cleaning task manager started.")

    def _on_map(self, _: OccupancyGrid) -> None:
        self.map_received = True

    def _on_waypoints(self, msg: PoseArray) -> None:
        if not msg.poses:
            self.get_logger().warn("Received empty coverage waypoint list.")
            return
        if self.mission_started:
            return
        if self.path_received and len(self.waypoints) == len(msg.poses):
            return
        self.waypoints = [deepcopy(pose) for pose in msg.poses]
        self.path_frame = msg.header.frame_id or "map"
        self.path_received = True
        self.get_logger().info("Received %d coverage waypoints." % len(self.waypoints))

    def _on_coverage_progress(self, msg: Float32) -> None:
        self.coverage_percent = float(msg.data)

    def _on_amcl_pose(self, _: PoseWithCovarianceStamped) -> None:
        self.localization_received = True

    def _on_timer(self) -> None:
        if self.current_state in (self.COMPLETED, self.FAILED):
            return

        if self._coverage_target_reached():
            self._finish_task(self.COMPLETED, "Coverage target reached")
            return

        if self.current_state == self.IDLE:
            if not self.map_received:
                self._set_state(self.WAIT_FOR_MAP)
            elif not self.localization_received:
                self._set_state(self.WAIT_FOR_LOCALIZATION)
            elif not self.navigation_ready:
                self._set_state(self.WAIT_FOR_NAVIGATION)
            else:
                self._start_waiting_for_path()
            return

        if self.current_state == self.WAIT_FOR_MAP:
            if self.map_received:
                if self.localization_received:
                    if self.navigation_ready:
                        self._start_waiting_for_path()
                    else:
                        self._set_state(self.WAIT_FOR_NAVIGATION)
                else:
                    self._set_state(self.WAIT_FOR_LOCALIZATION)
            return

        if self.current_state == self.WAIT_FOR_LOCALIZATION:
            if not self.map_received:
                self._set_state(self.WAIT_FOR_MAP)
                return
            if self.localization_received:
                if self.navigation_ready:
                    self._start_waiting_for_path()
                else:
                    self._set_state(self.WAIT_FOR_NAVIGATION)
            return

        if self.current_state == self.WAIT_FOR_NAVIGATION:
            if not self.map_received:
                self._set_state(self.WAIT_FOR_MAP)
                return
            if not self.localization_received:
                self._set_state(self.WAIT_FOR_LOCALIZATION)
                return
            self._check_navigation_ready()
            if self.navigation_ready:
                self._start_waiting_for_path()
            return

        if self.current_state == self.GENERATE_PATH:
            if self.path_received:
                self.mission_started = True
                self._set_state(self.NAVIGATING)
                return
            if self._path_wait_timed_out():
                self._finish_task(self.FAILED, "Timed out waiting for coverage waypoints")
            return

        if self.current_state == self.NAVIGATING:
            if self.current_index >= len(self.waypoints):
                self._finish_task(self.COMPLETED, "All waypoints processed")
                return
            if self.current_goal_handle is None and not self.goal_request_pending:
                self._send_current_goal()
            return

        if self.current_state == self.RECOVERY:
            if self.get_clock().now().nanoseconds >= self.recovery_deadline_ns:
                self._complete_recovery_step()

    def _start_waiting_for_path(self) -> None:
        self.path_wait_start_ns = self.get_clock().now().nanoseconds
        self._set_state(self.GENERATE_PATH)

    def _check_navigation_ready(self) -> None:
        if self.navigation_ready or self.navigation_check_future is not None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_navigation_check_ns < self.navigation_check_interval_ns:
            return
        self.last_navigation_check_ns = now_ns

        if not self.navigation_active_client.wait_for_service(timeout_sec=0.1):
            return

        future = self.navigation_active_client.call_async(Trigger.Request())
        self.navigation_check_future = future
        future.add_done_callback(self._on_navigation_ready_response)

    def _on_navigation_ready_response(self, future) -> None:
        self.navigation_check_future = None
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().warn("Navigation readiness check failed: %s" % exc)
            return

        if response.success and not self.navigation_ready:
            self.navigation_ready = True
            self.get_logger().info("Navigation stack is active and ready for goals.")

    def _path_wait_timed_out(self) -> bool:
        timeout_ns = int(float(self.get_parameter("path_wait_timeout").value) * 1e9)
        return (
            timeout_ns > 0
            and self.get_clock().now().nanoseconds - self.path_wait_start_ns >= timeout_ns
        )

    def _send_current_goal(self) -> None:
        if not self.navigate_client.server_is_ready():
            if not self.navigate_client.wait_for_server(timeout_sec=0.1):
                self.get_logger().info("Waiting for NavigateToPose action server...")
                return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.path_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose = deepcopy(self.waypoints[self.current_index])

        self.goal_request_pending = True
        future = self.navigate_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)
        self.get_logger().info(
            "Sending waypoint %d/%d"
            % (self.current_index + 1, len(self.waypoints))
        )

    def _on_goal_response(self, future) -> None:
        self.goal_request_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._schedule_recovery("Goal request failed: %s" % exc)
            return

        if not goal_handle.accepted:
            self._schedule_recovery("NavigateToPose goal rejected")
            return

        self.current_goal_handle = goal_handle
        self.current_result_future = goal_handle.get_result_async()
        self.current_result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future) -> None:
        self.current_goal_handle = None
        self.current_result_future = None

        try:
            result = future.result()
        except Exception as exc:
            self._schedule_recovery("Goal result failed: %s" % exc)
            return

        if self.current_state in (self.COMPLETED, self.FAILED):
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.successful_waypoints += 1
            self.consecutive_failed_waypoints = 0
            self.current_waypoint_failures = 0
            self.current_index += 1
            self.get_logger().info(
                "Waypoint %d reached. Coverage: %.1f%%"
                % (self.current_index, self.coverage_percent)
            )
            if self.current_index >= len(self.waypoints):
                self._finish_task(self.COMPLETED, "All waypoints completed")
            return

        if result.status == GoalStatus.STATUS_CANCELED and self.current_state == self.COMPLETED:
            return

        self._schedule_recovery(
            "Waypoint %d failed with status %s"
            % (self.current_index + 1, goal_status_to_string(result.status))
        )

    def _schedule_recovery(self, reason: str) -> None:
        if self.current_state in (self.COMPLETED, self.FAILED):
            return

        self.current_waypoint_failures += 1
        self.recovery_reason = reason
        self.recovery_services_requested = False
        self.recovery_deadline_ns = self.get_clock().now().nanoseconds + int(
            float(self.get_parameter("recovery_wait_seconds").value) * 1e9
        )
        self._request_costmap_clear()
        self._set_state(self.RECOVERY)
        self.get_logger().warn(reason)

    def _request_costmap_clear(self) -> None:
        if self.recovery_services_requested:
            return

        request = ClearEntireCostmap.Request()
        for client, label in (
            (self.clear_local_costmap_client, "local"),
            (self.clear_global_costmap_client, "global"),
        ):
            if client.wait_for_service(timeout_sec=0.1):
                client.call_async(request)
                self.get_logger().info("Requested %s costmap clear during recovery." % label)
            else:
                self.get_logger().warn(
                    "Costmap clear service for %s map is not ready." % label
                )
        self.recovery_services_requested = True

    def _complete_recovery_step(self) -> None:
        retry_limit = int(self.get_parameter("single_goal_retry_limit").value)
        if self.current_waypoint_failures <= retry_limit:
            self._set_state(self.NAVIGATING)
            self.get_logger().info(
                "Retrying waypoint %d (%d/%d retries used)."
                % (
                    self.current_index + 1,
                    self.current_waypoint_failures,
                    retry_limit,
                )
            )
            return

        self.consecutive_failed_waypoints += 1
        self.skipped_waypoints += 1
        self.current_waypoint_failures = 0

        if not parameter_as_bool(self.get_parameter("skip_failed_waypoints").value):
            self._finish_task(self.FAILED, "Waypoint retry budget exhausted")
            return

        max_consecutive = int(self.get_parameter("max_consecutive_failed_waypoints").value)
        if self.consecutive_failed_waypoints >= max_consecutive:
            self._finish_task(
                self.FAILED,
                "Too many consecutive failed waypoints (%d)" % self.consecutive_failed_waypoints,
            )
            return

        if self.current_index + 1 >= len(self.waypoints):
            self._finish_task(self.FAILED, "Final waypoint failed and no more waypoints remain")
            return

        self.current_index += 1
        self._set_state(self.NAVIGATING)
        self.get_logger().warn(
            "Skipping waypoint %d after retries. Moving to waypoint %d."
            % (self.current_index, self.current_index + 1)
        )

    def _coverage_target_reached(self) -> bool:
        return self.coverage_percent >= float(
            self.get_parameter("coverage_target_percent").value
        )

    def _finish_task(self, final_state: str, reason: str) -> None:
        if final_state == self.COMPLETED:
            self._cancel_active_goal()
        elif final_state == self.FAILED:
            self._cancel_active_goal()

        self._set_state(final_state)
        self.get_logger().info(
            "%s. Success=%d Skipped=%d Coverage=%.1f%%"
            % (
                reason,
                self.successful_waypoints,
                self.skipped_waypoints,
                self.coverage_percent,
            )
        )

    def _cancel_active_goal(self) -> None:
        if self.current_goal_handle is None:
            return
        self.current_goal_handle.cancel_goal_async()
        self.current_goal_handle = None
        self.current_result_future = None
        self.goal_request_pending = False

    def _set_state(self, state: str) -> None:
        if state == self.current_state:
            return
        old_state = self.current_state
        self.current_state = state
        self._publish_state()
        self.get_logger().info("State transition: %s -> %s" % (old_state, state))

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)


def goal_status_to_string(status: int) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return mapping.get(status, str(status))


def parameter_as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def main() -> None:
    rclpy.init()
    node = CleaningTaskManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
