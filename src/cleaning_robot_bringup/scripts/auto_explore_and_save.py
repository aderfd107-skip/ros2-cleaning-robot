#!/usr/bin/env python3

import math
import os
from collections import deque

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav2_msgs.srv import ClearEntireCostmap, SaveMap
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration as RosDuration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException
from tf2_ros.transform_listener import TransformListener


class AutoExploreAndSave(Node):
    def __init__(self) -> None:
        super().__init__("auto_explore_and_save")

        self._declare_parameters()
        self._load_parameters()
        self._apply_world_profile_overrides()

        if not self.save_map_url:
            raise ValueError("Parameter 'save_map_url' must not be empty.")

        self.save_map_url = self._normalize_map_save_url(self.save_map_url)
        map_dir = os.path.dirname(self.save_map_url)
        if map_dir:
            os.makedirs(map_dir, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.create_subscription(OccupancyGrid, self.map_topic, self._map_callback, 10)

        self.navigate_to_pose_client = ActionClient(
            self, NavigateToPose, self.navigate_to_pose_action
        )
        self.spin_client = ActionClient(self, Spin, self.bootstrap_spin_action)

        self.clear_costmap_clients = {}
        for service_name in [
            self.clear_global_costmap_service,
            self.clear_local_costmap_service,
        ]:
            if service_name and service_name not in self.clear_costmap_clients:
                self.clear_costmap_clients[service_name] = self.create_client(
                    ClearEntireCostmap, service_name
                )

        self.save_map_service_candidates = []
        for service_name in [
            preferred
            for preferred in [self.save_map_service, "/map_saver/save_map", "/save_map"]
        ]:
            if service_name and service_name not in self.save_map_service_candidates:
                self.save_map_service_candidates.append(service_name)
        self.save_map_clients = {
            service_name: self.create_client(SaveMap, service_name)
            for service_name in self.save_map_service_candidates
        }
        self.active_save_map_service = None

        self.latest_map = None
        self.latest_map_stats = None
        self.current_position_x = None
        self.current_position_y = None
        self.current_yaw = None

        self.start_time = self.get_clock().now()
        self.last_map_update_ns = 0
        self.last_map_progress_time_ns = 0
        self.last_status_log_ns = 0
        self.last_nav_wait_log_ns = 0
        self.last_spin_wait_log_ns = 0
        self.last_tf_warning_ns = 0
        self.last_costmap_clear_ns = 0

        self.progress_baseline_known_area_m2 = 0.0
        self.progress_baseline_free_area_m2 = 0.0
        self.progress_baseline_observed_area_m2 = 0.0
        self.progress_baseline_bbox_unknown_area_m2 = 0.0

        self.active_frontier_target = None
        self.pending_goal_target = None
        self.goal_send_future = None
        self.active_goal_handle = None
        self.active_result_future = None
        self.active_goal_sent_ns = 0
        self.active_goal_cancel_reason = None
        self.cancel_future = None
        self.pending_save_reason = None
        self.blocked_targets = []
        self.completed_targets = []
        self.failed_clusters = []
        self.latest_feedback_distance_remaining = None
        self.latest_feedback_recoveries = 0
        self.best_feedback_distance_remaining = None
        self.last_goal_progress_ns = 0
        self.latest_frontier_selection_debug = "frontier 选点调试信息尚未生成。"
        self.bootstrap_spin_completed = not self.bootstrap_spin_enabled
        self.bootstrap_spin_send_future = None
        self.bootstrap_spin_goal_handle = None
        self.bootstrap_spin_result_future = None
        self.bootstrap_spin_feedback_yaw = None

        self.save_requested = False
        self.save_future = None
        self.save_retry_deadline_ns = 0
        self.save_retry_cooldown_until_ns = 0

        self.exploration_state = "WAIT_FOR_MAP"

        self.timer = self.create_timer(1.0 / self.control_rate, self._control_loop)
        self.get_logger().info(
            "自动探索已启动，模式=frontier+Nav2，场景=%s，地图保存到 %s，软超时 %.1fs，硬超时 %.1fs"
            % (
                self.world_name,
                self.save_map_url,
                self.exploration_timeout,
                self.max_exploration_timeout,
            )
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("world_name", "unknown")
        self.declare_parameter("world_profile_path", "")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_footprint")
        self.declare_parameter("navigate_to_pose_action", "/navigate_to_pose")
        self.declare_parameter("bootstrap_spin_action", "/spin")
        self.declare_parameter(
            "clear_global_costmap_service",
            "/global_costmap/clear_entirely_global_costmap",
        )
        self.declare_parameter(
            "clear_local_costmap_service",
            "/local_costmap/clear_entirely_local_costmap",
        )
        self.declare_parameter("save_map_service", "/map_saver/save_map")
        self.declare_parameter("save_map_url", "")
        self.declare_parameter("exploration_timeout", 120.0)
        self.declare_parameter("max_exploration_timeout", 220.0)
        self.declare_parameter("minimum_exploration_time", 45.0)
        self.declare_parameter("control_rate", 2.0)
        self.declare_parameter("status_log_interval", 10.0)
        self.declare_parameter("nav2_wait_timeout", 40.0)
        self.declare_parameter("save_retry_interval", 3.0)
        self.declare_parameter("clear_costmap_on_failure", True)
        self.declare_parameter("costmap_clear_cooldown", 4.0)
        self.declare_parameter("bootstrap_spin_enabled", False)
        self.declare_parameter("bootstrap_spin_target_yaw", 0.0)
        self.declare_parameter("bootstrap_spin_time_allowance", 20.0)

        self.declare_parameter("map_stall_duration", 18.0)
        self.declare_parameter("map_progress_min_known_area", 0.8)
        self.declare_parameter("map_progress_min_observed_area", 1.2)
        self.declare_parameter("map_progress_min_bbox_unknown_area_reduction", 0.5)
        self.declare_parameter("completion_min_observed_area", 18.0)
        self.declare_parameter("completion_max_bbox_unknown_ratio", 0.18)
        self.declare_parameter("completion_max_bbox_unknown_area", 3.0)
        self.declare_parameter("completion_require_both_bbox_metrics", False)

        self.declare_parameter("frontier_cluster_min_cells", 12)
        self.declare_parameter("frontier_residual_cluster_min_cells", 12)
        self.declare_parameter("frontier_target_reached_distance", 0.40)
        self.declare_parameter("frontier_target_timeout", 45.0)
        self.declare_parameter("frontier_target_switch_distance", 0.8)
        self.declare_parameter("frontier_size_weight", 0.08)
        self.declare_parameter("frontier_distance_weight", 1.0)
        self.declare_parameter("frontier_heading_weight", 0.35)
        self.declare_parameter("frontier_completion_max_clusters", 0)
        self.declare_parameter("frontier_completion_max_cells", 10)
        self.declare_parameter("frontier_target_cell_stride", 3)
        self.declare_parameter("frontier_target_offset", 0.20)
        self.declare_parameter("frontier_clearance_radius", 0.18)
        self.declare_parameter("frontier_blocked_path_penalty", 3.0)
        self.declare_parameter("frontier_unknown_path_penalty", 0.8)
        self.declare_parameter("frontier_max_path_occupied_ratio", 0.0)
        self.declare_parameter("frontier_max_path_unknown_ratio", 0.35)
        self.declare_parameter("frontier_fallback_path_unknown_ratio", 0.65)
        self.declare_parameter("frontier_failed_goal_cooldown", 18.0)
        self.declare_parameter("frontier_failed_goal_radius", 0.45)
        self.declare_parameter("frontier_failed_goal_max_retries", 3)
        self.declare_parameter("frontier_failed_goal_backoff", 1.5)
        self.declare_parameter("frontier_reached_goal_cooldown", 30.0)
        self.declare_parameter("frontier_reached_goal_radius", 0.0)
        self.declare_parameter("frontier_reached_cluster_radius", 0.0)
        self.declare_parameter("frontier_goal_progress_timeout", 0.0)
        self.declare_parameter("frontier_goal_progress_epsilon", 0.12)
        self.declare_parameter("frontier_max_recoveries", -1)

    def _load_parameters(self) -> None:
        self.world_name = self.get_parameter("world_name").value
        self.world_profile_path = self.get_parameter("world_profile_path").value
        self.map_topic = self.get_parameter("map_topic").value
        self.global_frame = self.get_parameter("global_frame").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.navigate_to_pose_action = self.get_parameter("navigate_to_pose_action").value
        self.bootstrap_spin_action = self.get_parameter("bootstrap_spin_action").value
        self.clear_global_costmap_service = self.get_parameter(
            "clear_global_costmap_service"
        ).value
        self.clear_local_costmap_service = self.get_parameter(
            "clear_local_costmap_service"
        ).value
        self.save_map_service = self.get_parameter("save_map_service").value
        self.save_map_url = self.get_parameter("save_map_url").value
        self.exploration_timeout = float(self.get_parameter("exploration_timeout").value)
        self.max_exploration_timeout = float(
            self.get_parameter("max_exploration_timeout").value
        )
        self.minimum_exploration_time = float(
            self.get_parameter("minimum_exploration_time").value
        )
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.status_log_interval = float(self.get_parameter("status_log_interval").value)
        self.nav2_wait_timeout = float(self.get_parameter("nav2_wait_timeout").value)
        self.save_retry_interval = float(self.get_parameter("save_retry_interval").value)
        self.clear_costmap_on_failure = bool(
            self.get_parameter("clear_costmap_on_failure").value
        )
        self.costmap_clear_cooldown = float(
            self.get_parameter("costmap_clear_cooldown").value
        )
        self.bootstrap_spin_enabled = bool(
            self.get_parameter("bootstrap_spin_enabled").value
        )
        self.bootstrap_spin_target_yaw = abs(
            float(self.get_parameter("bootstrap_spin_target_yaw").value)
        )
        self.bootstrap_spin_time_allowance = max(
            1.0, float(self.get_parameter("bootstrap_spin_time_allowance").value)
        )

        self.map_stall_duration = float(self.get_parameter("map_stall_duration").value)
        self.map_progress_min_known_area = float(
            self.get_parameter("map_progress_min_known_area").value
        )
        self.map_progress_min_observed_area = float(
            self.get_parameter("map_progress_min_observed_area").value
        )
        self.map_progress_min_bbox_unknown_area_reduction = float(
            self.get_parameter("map_progress_min_bbox_unknown_area_reduction").value
        )
        self.completion_min_observed_area = float(
            self.get_parameter("completion_min_observed_area").value
        )
        self.completion_max_bbox_unknown_ratio = float(
            self.get_parameter("completion_max_bbox_unknown_ratio").value
        )
        self.completion_max_bbox_unknown_area = float(
            self.get_parameter("completion_max_bbox_unknown_area").value
        )
        self.completion_require_both_bbox_metrics = bool(
            self.get_parameter("completion_require_both_bbox_metrics").value
        )

        self.frontier_cluster_min_cells = int(
            self.get_parameter("frontier_cluster_min_cells").value
        )
        self.frontier_residual_cluster_min_cells = int(
            self.get_parameter("frontier_residual_cluster_min_cells").value
        )
        self.frontier_target_reached_distance = float(
            self.get_parameter("frontier_target_reached_distance").value
        )
        self.frontier_target_timeout = float(
            self.get_parameter("frontier_target_timeout").value
        )
        self.frontier_target_switch_distance = float(
            self.get_parameter("frontier_target_switch_distance").value
        )
        self.frontier_size_weight = float(
            self.get_parameter("frontier_size_weight").value
        )
        self.frontier_distance_weight = float(
            self.get_parameter("frontier_distance_weight").value
        )
        self.frontier_heading_weight = float(
            self.get_parameter("frontier_heading_weight").value
        )
        self.frontier_completion_max_clusters = int(
            self.get_parameter("frontier_completion_max_clusters").value
        )
        self.frontier_completion_max_cells = int(
            self.get_parameter("frontier_completion_max_cells").value
        )
        self.frontier_target_cell_stride = max(
            1, int(self.get_parameter("frontier_target_cell_stride").value)
        )
        self.frontier_target_offset = float(
            self.get_parameter("frontier_target_offset").value
        )
        self.frontier_clearance_radius = float(
            self.get_parameter("frontier_clearance_radius").value
        )
        self.frontier_blocked_path_penalty = float(
            self.get_parameter("frontier_blocked_path_penalty").value
        )
        self.frontier_unknown_path_penalty = float(
            self.get_parameter("frontier_unknown_path_penalty").value
        )
        self.frontier_max_path_occupied_ratio = float(
            self.get_parameter("frontier_max_path_occupied_ratio").value
        )
        self.frontier_max_path_unknown_ratio = float(
            self.get_parameter("frontier_max_path_unknown_ratio").value
        )
        self.frontier_fallback_path_unknown_ratio = float(
            self.get_parameter("frontier_fallback_path_unknown_ratio").value
        )
        self.frontier_failed_goal_cooldown = float(
            self.get_parameter("frontier_failed_goal_cooldown").value
        )
        self.frontier_failed_goal_radius = float(
            self.get_parameter("frontier_failed_goal_radius").value
        )
        self.frontier_failed_goal_max_retries = int(
            self.get_parameter("frontier_failed_goal_max_retries").value
        )
        self.frontier_failed_goal_backoff = float(
            self.get_parameter("frontier_failed_goal_backoff").value
        )
        self.frontier_reached_goal_cooldown = max(
            0.0, float(self.get_parameter("frontier_reached_goal_cooldown").value)
        )
        self.frontier_reached_goal_radius = max(
            float(self.get_parameter("frontier_reached_goal_radius").value),
            self.frontier_target_reached_distance * 1.5,
        )
        self.frontier_reached_cluster_radius = max(
            float(self.get_parameter("frontier_reached_cluster_radius").value),
            self.frontier_target_switch_distance,
            self.frontier_failed_goal_radius * 2.0,
        )
        self.frontier_goal_progress_timeout = float(
            self.get_parameter("frontier_goal_progress_timeout").value
        )
        self.frontier_goal_progress_epsilon = float(
            self.get_parameter("frontier_goal_progress_epsilon").value
        )
        self.frontier_max_recoveries = int(
            self.get_parameter("frontier_max_recoveries").value
        )

    def _apply_world_profile_overrides(self) -> None:
        if not self.world_profile_path:
            return
        if not os.path.isfile(self.world_profile_path):
            self.get_logger().warn(
                "场景参数文件不存在，跳过覆盖：%s" % self.world_profile_path
            )
            return

        with open(self.world_profile_path, "r", encoding="utf-8") as profile_file:
            data = yaml.safe_load(profile_file) or {}

        params = data.get("auto_explore_and_save", {}).get("ros__parameters", {})
        if not isinstance(params, dict):
            return

        for key, value in params.items():
            if not hasattr(self, key):
                continue
            current_value = getattr(self, key)
            if isinstance(current_value, bool):
                setattr(self, key, bool(value))
            elif isinstance(current_value, int) and not isinstance(current_value, bool):
                setattr(self, key, int(value))
            elif isinstance(current_value, float):
                setattr(self, key, float(value))
            else:
                setattr(self, key, value)

        self.get_logger().info("已应用场景参数覆盖：%s" % self.world_profile_path)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        stats = self._extract_map_stats(msg)
        if stats is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        self.last_map_update_ns = now_ns
        self.latest_map_stats = stats

        if self.last_map_progress_time_ns == 0:
            self.last_map_progress_time_ns = now_ns
            self._update_map_progress_baseline(stats)
            self.get_logger().info(
                "已收到初始地图：已知面积 %.1fm^2，观测包围盒 %.1fm^2。"
                % (stats["known_area_m2"], stats["observed_area_m2"])
            )
            return

        if self._map_progressed(stats):
            self.last_map_progress_time_ns = now_ns
            self._update_map_progress_baseline(stats)
            self._clear_blocked_targets_on_progress()
            self._clear_completed_targets_on_progress()
            if now_ns - self.last_status_log_ns >= int(self.status_log_interval * 1e9):
                self.get_logger().info(
                    "地图继续增长：已知面积 %.1fm^2，raw frontier %d 簇/%d 格，主 frontier %d 簇/%d 格，残余 frontier %d 簇/%d 格，包围盒未知比 %.1f%%。"
                    % (
                        stats["known_area_m2"],
                        stats["raw_frontier_cluster_count"],
                        stats["raw_frontier_cells"],
                        stats["frontier_cluster_count"],
                        stats["frontier_cells"],
                        stats["residual_frontier_cluster_count"],
                        stats["residual_frontier_cells"],
                        stats["bbox_unknown_ratio"] * 100.0,
                    )
                )
                self.last_status_log_ns = now_ns

    def _control_loop(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        self._prune_blocked_targets(now_ns)
        self._prune_completed_targets(now_ns)
        self._prune_failed_clusters(now_ns)
        self._update_robot_pose(now_ns)
        self._check_goal_response()
        self._check_navigation_result()
        self._check_bootstrap_spin_response()
        self._check_bootstrap_spin_result()
        self._check_save_result()

        if self.pending_save_reason is None:
            self.pending_save_reason = self._evaluate_save_reason(now_ns, elapsed)

        if self.pending_save_reason is not None:
            if self._navigation_busy():
                self._set_exploration_state("STOP_FOR_SAVE")
                self._cancel_active_goal("save_map")
                return
            if self.save_requested:
                self._set_exploration_state("SAVE_MAP")
                return
            if now_ns < self.save_retry_cooldown_until_ns:
                self._set_exploration_state("SAVE_RETRY_WAIT")
                return
            self.get_logger().info("%s，停止探索并开始存图。" % self.pending_save_reason)
            self.pending_save_reason = None
            self._request_map_save()
            return

        if self.save_requested:
            self._set_exploration_state("SAVE_MAP")
            return

        if self.latest_map_stats is None:
            self._set_exploration_state("WAIT_FOR_MAP")
            return

        if not self._has_pose():
            self._set_exploration_state("WAIT_FOR_TF")
            return

        if not self.bootstrap_spin_completed:
            if not self.spin_client.wait_for_server(timeout_sec=0.0):
                self._set_exploration_state("WAIT_FOR_BOOTSTRAP_SPIN")
                if now_ns - self.last_spin_wait_log_ns >= int(
                    self.status_log_interval * 1e9
                ):
                    self.get_logger().info(
                        "等待启动自旋 action 就绪：%s" % self.bootstrap_spin_action
                    )
                    self.last_spin_wait_log_ns = now_ns
                return

            if self._bootstrap_spin_busy():
                self._monitor_bootstrap_spin(now_ns)
                return

            if self.bootstrap_spin_target_yaw > 1e-3:
                self._send_bootstrap_spin()
                return

            self.bootstrap_spin_completed = True

        if not self.navigate_to_pose_client.wait_for_server(timeout_sec=0.0):
            self._set_exploration_state("WAIT_FOR_NAV2")
            if now_ns - self.last_nav_wait_log_ns >= int(self.status_log_interval * 1e9):
                if elapsed >= self.nav2_wait_timeout:
                    self.get_logger().warn(
                        "Nav2 action 仍未就绪：%s，已等待 %.1fs。"
                        % (self.navigate_to_pose_action, elapsed)
                    )
                else:
                    self.get_logger().info(
                        "等待 Nav2 action 就绪：%s" % self.navigate_to_pose_action
                    )
                self.last_nav_wait_log_ns = now_ns
            return

        if self._navigation_busy():
            self._monitor_active_goal(now_ns)
            return

        frontier_target = self._select_frontier_target(now_ns)
        if frontier_target is None:
            self._set_exploration_state("WAIT_FOR_FRONTIER")
            self._log_status(
                now_ns,
                "当前没有可执行的 frontier 目标，等待地图继续更新。%s"
                % self.latest_frontier_selection_debug,
            )
            return

        self._send_navigation_goal(frontier_target, now_ns)

    def _monitor_active_goal(self, now_ns: int) -> None:
        if self.active_frontier_target is None:
            return

        if (
            self.active_goal_handle is not None
            and now_ns - self.active_goal_sent_ns >= int(self.frontier_target_timeout * 1e9)
        ):
            self.get_logger().warn(
                "frontier 目标超时 %.1fs，取消当前导航并改选下一个目标。"
                % self.frontier_target_timeout
            )
            self._cancel_active_goal("goal_timeout")
            return

        if not self._is_frontier_target_still_valid(
            self.active_frontier_target,
            self.latest_map_stats["raw_frontier_clusters"],
        ) and not self._is_frontier_target_reached(self.active_frontier_target):
            self.get_logger().info("当前 frontier 已失效，取消旧目标并重新选点。")
            self._cancel_active_goal("goal_invalid")
            return

        if (
            self.frontier_max_recoveries >= 0
            and self.latest_feedback_recoveries >= self.frontier_max_recoveries
            and self.active_goal_handle is not None
        ):
            self.get_logger().warn(
                "frontier 目标恢复次数过多(%d)，取消当前导航并改选下一个目标。"
                % self.latest_feedback_recoveries
            )
            self._cancel_active_goal("too_many_recoveries")
            return

        if (
            self.frontier_goal_progress_timeout > 0.0
            and self.last_goal_progress_ns > 0
            and now_ns - self.last_goal_progress_ns
            >= int(self.frontier_goal_progress_timeout * 1e9)
        ):
            self.get_logger().warn(
                "frontier 目标 %.1fs 内没有明显进展，取消当前导航并改选下一个目标。"
                % self.frontier_goal_progress_timeout
            )
            self._cancel_active_goal("goal_no_progress")
            return

        self._set_exploration_state("NAVIGATING")
        if self.latest_feedback_distance_remaining is None:
            return
        self._log_status(
            now_ns,
            "Nav2 正在执行 frontier 目标：剩余距离 %.2fm，recoveries=%d。"
            % (
                self.latest_feedback_distance_remaining,
                self.latest_feedback_recoveries,
            ),
        )

    def _send_navigation_goal(self, frontier_target, now_ns: int) -> None:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_goal_pose(frontier_target)
        goal_msg.behavior_tree = ""

        self.pending_goal_target = frontier_target
        self.goal_send_future = self.navigate_to_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self._navigate_feedback_callback,
        )
        self.active_goal_sent_ns = now_ns
        self.latest_feedback_distance_remaining = None
        self.latest_feedback_recoveries = 0
        self.best_feedback_distance_remaining = None
        self.last_goal_progress_ns = now_ns
        self._set_exploration_state("SEND_GOAL")
        self.get_logger().info(
            "发送 frontier 目标到 Nav2：goal=(%.2f, %.2f) centroid=(%.2f, %.2f) size=%d"
            % (
                frontier_target["x"],
                frontier_target["y"],
                frontier_target["centroid_x"],
                frontier_target["centroid_y"],
                frontier_target["size"],
            )
        )

    def _check_goal_response(self) -> None:
        if self.goal_send_future is None or not self.goal_send_future.done():
            return

        target = self.pending_goal_target
        self.pending_goal_target = None
        goal_future = self.goal_send_future
        self.goal_send_future = None

        try:
            goal_handle = goal_future.result()
        except Exception as exc:  # pragma: no cover - runtime guard
            self.get_logger().error("发送 NavigateToPose 目标失败：%s" % exc)
            if target is not None:
                self._mark_target_failed(target, "goal_send_exception")
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("Nav2 拒绝了 frontier 目标。")
            if target is not None:
                self._mark_target_failed(target, "goal_rejected")
            return

        self.active_goal_handle = goal_handle
        self.active_result_future = goal_handle.get_result_async()
        self.active_goal_cancel_reason = None
        self.active_frontier_target = target
        self._set_exploration_state("NAVIGATING")

    def _check_navigation_result(self) -> None:
        if self.active_result_future is None or not self.active_result_future.done():
            return

        target = self.active_frontier_target
        cancel_reason = self.active_goal_cancel_reason
        try:
            result = self.active_result_future.result()
        except Exception as exc:  # pragma: no cover - runtime guard
            self.get_logger().error("等待导航结果时异常：%s" % exc)
            if target is not None and cancel_reason != "save_map":
                self._mark_target_failed(target, "goal_result_exception")
            self._reset_goal_state()
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            if target is not None:
                self._mark_target_completed(target)
                self.get_logger().info(
                    "frontier 目标到达：goal=(%.2f, %.2f)。"
                    % (target["x"], target["y"])
                )
        elif status == GoalStatus.STATUS_CANCELED and cancel_reason == "save_map":
            self.get_logger().info("当前导航已取消，准备存图。")
        else:
            reason = cancel_reason or self._goal_status_name(status)
            if target is not None:
                self._mark_target_failed(target, reason)
            self.get_logger().warn("frontier 目标未成功完成：%s" % reason)

        self._reset_goal_state()

    def _send_bootstrap_spin(self) -> None:
        goal_msg = Spin.Goal()
        goal_msg.target_yaw = float(self.bootstrap_spin_target_yaw)
        goal_msg.time_allowance = RosDuration(
            seconds=self.bootstrap_spin_time_allowance
        ).to_msg()

        self.bootstrap_spin_feedback_yaw = None
        self.bootstrap_spin_send_future = self.spin_client.send_goal_async(
            goal_msg,
            feedback_callback=self._bootstrap_spin_feedback_callback,
        )
        self._set_exploration_state("BOOTSTRAP_SPIN_SEND")
        self.get_logger().info(
            "发送启动自旋：target_yaw=%.2f rad，time_allowance=%.1fs"
            % (
                self.bootstrap_spin_target_yaw,
                self.bootstrap_spin_time_allowance,
            )
        )

    def _check_bootstrap_spin_response(self) -> None:
        if (
            self.bootstrap_spin_send_future is None
            or not self.bootstrap_spin_send_future.done()
        ):
            return

        spin_future = self.bootstrap_spin_send_future
        self.bootstrap_spin_send_future = None

        try:
            goal_handle = spin_future.result()
        except Exception as exc:  # pragma: no cover - runtime guard
            self.get_logger().warn("启动自旋发送失败，继续探索：%s" % exc)
            self.bootstrap_spin_completed = True
            self._reset_bootstrap_spin_state()
            return

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("启动自旋被拒绝，继续探索。")
            self.bootstrap_spin_completed = True
            self._reset_bootstrap_spin_state()
            return

        self.bootstrap_spin_goal_handle = goal_handle
        self.bootstrap_spin_result_future = goal_handle.get_result_async()
        self._set_exploration_state("BOOTSTRAP_SPIN")

    def _check_bootstrap_spin_result(self) -> None:
        if (
            self.bootstrap_spin_result_future is None
            or not self.bootstrap_spin_result_future.done()
        ):
            return

        try:
            result = self.bootstrap_spin_result_future.result()
        except Exception as exc:  # pragma: no cover - runtime guard
            self.get_logger().warn("等待启动自旋结果时异常，继续探索：%s" % exc)
            self.bootstrap_spin_completed = True
            self._reset_bootstrap_spin_state()
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            elapsed = result.result.total_elapsed_time
            self.get_logger().info(
                "启动自旋完成：已旋转 %.2f rad，用时 %d.%09ds。"
                % (
                    self.bootstrap_spin_feedback_yaw or 0.0,
                    elapsed.sec,
                    elapsed.nanosec,
                )
            )
        else:
            self.get_logger().warn(
                "启动自旋未成功完成（%s），继续探索。"
                % self._goal_status_name(status)
            )

        self.bootstrap_spin_completed = True
        self._reset_bootstrap_spin_state()

    def _monitor_bootstrap_spin(self, now_ns: int) -> None:
        self._set_exploration_state("BOOTSTRAP_SPIN")
        if self.bootstrap_spin_feedback_yaw is None:
            return
        self._log_status(
            now_ns,
            "启动自旋进行中：已旋转 %.2f / %.2f rad。"
            % (
                self.bootstrap_spin_feedback_yaw,
                self.bootstrap_spin_target_yaw,
            ),
        )

    def _bootstrap_spin_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if hasattr(feedback, "angular_distance_traveled"):
            self.bootstrap_spin_feedback_yaw = float(
                feedback.angular_distance_traveled
            )

    def _navigate_feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        if hasattr(feedback, "distance_remaining"):
            distance_remaining = float(feedback.distance_remaining)
            self.latest_feedback_distance_remaining = distance_remaining
            if self.best_feedback_distance_remaining is None:
                self.best_feedback_distance_remaining = distance_remaining
                self.last_goal_progress_ns = self.get_clock().now().nanoseconds
            elif (
                self.best_feedback_distance_remaining - distance_remaining
                >= self.frontier_goal_progress_epsilon
            ):
                self.best_feedback_distance_remaining = distance_remaining
                self.last_goal_progress_ns = self.get_clock().now().nanoseconds
        if hasattr(feedback, "number_of_recoveries"):
            self.latest_feedback_recoveries = int(feedback.number_of_recoveries)

    def _cancel_active_goal(self, reason: str) -> None:
        if self.active_goal_handle is None:
            return
        if self.cancel_future is not None and not self.cancel_future.done():
            return

        self.active_goal_cancel_reason = reason
        self.cancel_future = self.active_goal_handle.cancel_goal_async()

    def _reset_goal_state(self) -> None:
        self.active_frontier_target = None
        self.active_goal_handle = None
        self.active_result_future = None
        self.active_goal_cancel_reason = None
        self.cancel_future = None
        self.latest_feedback_distance_remaining = None
        self.latest_feedback_recoveries = 0
        self.best_feedback_distance_remaining = None
        self.last_goal_progress_ns = 0

    def _reset_bootstrap_spin_state(self) -> None:
        self.bootstrap_spin_send_future = None
        self.bootstrap_spin_goal_handle = None
        self.bootstrap_spin_result_future = None
        self.bootstrap_spin_feedback_yaw = None

    def _navigation_busy(self) -> bool:
        return any(
            future is not None
            for future in [
                self.goal_send_future,
                self.active_goal_handle,
                self.active_result_future,
                self.bootstrap_spin_send_future,
                self.bootstrap_spin_goal_handle,
                self.bootstrap_spin_result_future,
            ]
        )

    def _bootstrap_spin_busy(self) -> bool:
        return any(
            future is not None
            for future in [
                self.bootstrap_spin_send_future,
                self.bootstrap_spin_goal_handle,
                self.bootstrap_spin_result_future,
            ]
        )

    def _build_goal_pose(self, frontier_target) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self.global_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = frontier_target["x"]
        pose.pose.position.y = frontier_target["y"]

        yaw = math.atan2(
            frontier_target["centroid_y"] - frontier_target["y"],
            frontier_target["centroid_x"] - frontier_target["x"],
        )
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose

    def _update_robot_pose(self, now_ns: int) -> None:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                self.robot_base_frame,
                Time(),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            if now_ns - self.last_tf_warning_ns >= int(self.status_log_interval * 1e9):
                self.get_logger().info(
                    "等待 TF：%s -> %s" % (self.global_frame, self.robot_base_frame)
                )
                self.last_tf_warning_ns = now_ns
            return

        self.current_position_x = float(transform.transform.translation.x)
        self.current_position_y = float(transform.transform.translation.y)
        self.current_yaw = self._yaw_from_quaternion(transform.transform.rotation)

    def _select_frontier_target(self, now_ns: int):
        if self.latest_map_stats is None or not self._has_pose():
            return None

        clusters = self.latest_map_stats["frontier_clusters"]
        residual_clusters = self.latest_map_stats.get("residual_frontier_clusters", [])
        if not clusters and not residual_clusters:
            self.latest_frontier_selection_debug = (
                "raw frontier %d 簇/%d 格，主 frontier 0 簇/0 格，残余 frontier %d 簇/%d 格。"
                % (
                    self.latest_map_stats["raw_frontier_cluster_count"],
                    self.latest_map_stats["raw_frontier_cells"],
                    self.latest_map_stats["residual_frontier_cluster_count"],
                    self.latest_map_stats["residual_frontier_cells"],
                )
            )
            return None

        distance_field = self._build_reachable_distance_field()
        if distance_field is None:
            self.latest_frontier_selection_debug = "reachable distance field 构建失败。"
            return None

        attempt_summaries = []

        if clusters:
            strict_target, strict_stats = self._select_frontier_target_with_unknown_ratio(
                clusters,
                now_ns,
                self.frontier_max_path_unknown_ratio,
                distance_field,
                min_cluster_cells=self.frontier_cluster_min_cells,
                selection_label="strict",
            )
            attempt_summaries.append(self._format_frontier_selection_stats(strict_stats))
            if strict_target is not None:
                self.latest_frontier_selection_debug = attempt_summaries[-1]
                return strict_target

        fallback_unknown_ratio = max(
            self.frontier_max_path_unknown_ratio,
            self.frontier_fallback_path_unknown_ratio,
        )
        if clusters and fallback_unknown_ratio > self.frontier_max_path_unknown_ratio:
            fallback_target, fallback_stats = self._select_frontier_target_with_unknown_ratio(
                clusters,
                now_ns,
                fallback_unknown_ratio,
                distance_field,
                min_cluster_cells=self.frontier_cluster_min_cells,
                selection_label="fallback",
            )
            attempt_summaries.append(self._format_frontier_selection_stats(fallback_stats))
            if fallback_target is not None:
                self.get_logger().info(
                    "严格 frontier 选点无可执行目标，放宽未知路径比例到 %.2f 后继续探索。"
                    % fallback_unknown_ratio
                )
                self.latest_frontier_selection_debug = attempt_summaries[-1]
                return fallback_target

        use_residual_frontiers = (
            self.frontier_residual_cluster_min_cells < self.frontier_cluster_min_cells
            and residual_clusters
        )
        if use_residual_frontiers:
            residual_target, residual_stats = self._select_frontier_target_with_unknown_ratio(
                residual_clusters,
                now_ns,
                self.frontier_fallback_path_unknown_ratio,
                distance_field,
                min_cluster_cells=self.frontier_residual_cluster_min_cells,
                selection_label="residual",
            )
            attempt_summaries.append(self._format_frontier_selection_stats(residual_stats))
            if residual_target is not None:
                self.get_logger().info(
                    "主 frontier 已不足，切换到残余 frontier 兜底：%d 簇/%d 格。"
                    % (
                        self.latest_map_stats["residual_frontier_cluster_count"],
                        self.latest_map_stats["residual_frontier_cells"],
                    )
                )
                self.latest_frontier_selection_debug = attempt_summaries[-1]
                return residual_target

        self.latest_frontier_selection_debug = " | ".join(attempt_summaries)
        return None

    def _select_frontier_target_with_unknown_ratio(
        self,
        clusters,
        now_ns: int,
        max_unknown_path_ratio: float,
        distance_field,
        min_cluster_cells: int,
        selection_label: str,
    ):
        best_target = None
        best_score = float("inf")
        selection_stats = self._make_frontier_selection_stats(
            selection_label,
            len(clusters),
            sum(cluster["size"] for cluster in clusters),
            max_unknown_path_ratio,
            min_cluster_cells,
        )
        for cluster in clusters:
            if cluster["size"] < max(1, min_cluster_cells):
                selection_stats["clusters_too_small"] += 1
                continue
            selection_stats["clusters_eligible"] += 1

            candidate = self._select_cluster_target_point(
                cluster,
                now_ns=now_ns,
                max_unknown_path_ratio=max_unknown_path_ratio,
                distance_field=distance_field,
                selection_stats=selection_stats,
            )
            if candidate is None:
                continue

            selection_stats["clusters_with_candidate"] += 1
            cluster_penalty = self._cluster_failure_penalty(cluster, now_ns)
            if cluster_penalty > 0.0:
                selection_stats["clusters_penalized"] += 1
                selection_stats["cluster_penalty_total"] += cluster_penalty
            heading_error = abs(self._target_yaw_error(candidate["x"], candidate["y"]))
            score = (
                candidate["travel_distance"] * self.frontier_distance_weight
                + heading_error * self.frontier_heading_weight
                + candidate["path_score"]
                + cluster_penalty
                - cluster["size"] * self.frontier_size_weight
            )
            if score < best_score:
                best_score = score
                best_target = {
                    "x": candidate["x"],
                    "y": candidate["y"],
                    "size": cluster["size"],
                    "centroid_x": cluster["centroid_x"],
                    "centroid_y": cluster["centroid_y"],
                    "selected_ns": now_ns,
                }

        selection_stats["target_found"] = best_target is not None
        return best_target, selection_stats

    def _select_cluster_target_point(
        self,
        cluster,
        now_ns: int,
        max_unknown_path_ratio: float,
        distance_field,
        selection_stats,
    ):
        candidate_points = cluster.get("points", [])
        if not candidate_points:
            return None

        best_candidate = None
        best_score = float("inf")
        sample_step = max(1, self.frontier_target_cell_stride)

        for point in candidate_points[::sample_step]:
            selection_stats["points_sampled"] += 1
            target_x, target_y = self._project_frontier_target(point)
            if self._is_recently_completed_target(cluster, target_x, target_y, now_ns):
                selection_stats["reject_recently_completed"] += 1
                continue
            if self._is_blocked_target(target_x, target_y, now_ns):
                selection_stats["reject_blocked"] += 1
                continue
            clearance_score = self._score_candidate_clearance(target_x, target_y)
            if math.isinf(clearance_score):
                selection_stats["reject_clearance"] += 1
                continue
            travel_distance = self._distance_from_field(
                distance_field, target_x, target_y
            )
            if math.isinf(travel_distance):
                selection_stats["reject_unreachable"] += 1
                continue
            path_score = self._score_path_to_point(
                target_x,
                target_y,
                max_unknown_path_ratio=max_unknown_path_ratio,
            )
            if math.isinf(path_score):
                selection_stats["reject_path"] += 1
                continue
            selection_stats["points_accepted"] += 1
            heading_error = abs(self._target_yaw_error(target_x, target_y))
            score = (
                travel_distance * self.frontier_distance_weight
                + heading_error * self.frontier_heading_weight
                + path_score
                + clearance_score
            )
            if score < best_score:
                best_score = score
                best_candidate = {
                    "x": target_x,
                    "y": target_y,
                    "travel_distance": travel_distance,
                    "path_score": path_score + clearance_score,
                }

        return best_candidate

    def _make_frontier_selection_stats(
        self,
        selection_label: str,
        cluster_count: int,
        frontier_cells: int,
        max_unknown_path_ratio: float,
        min_cluster_cells: int,
    ):
        return {
            "label": selection_label,
            "cluster_count": cluster_count,
            "frontier_cells": frontier_cells,
            "max_unknown_path_ratio": max_unknown_path_ratio,
            "min_cluster_cells": min_cluster_cells,
            "clusters_too_small": 0,
            "clusters_eligible": 0,
            "clusters_with_candidate": 0,
            "clusters_penalized": 0,
            "cluster_penalty_total": 0.0,
            "points_sampled": 0,
            "points_accepted": 0,
            "reject_recently_completed": 0,
            "reject_blocked": 0,
            "reject_clearance": 0,
            "reject_unreachable": 0,
            "reject_path": 0,
            "target_found": False,
        }

    def _format_frontier_selection_stats(self, selection_stats) -> str:
        return (
            "%s: clusters=%d cells=%d min=%d unknown<=%.2f eligible=%d with_candidate=%d penalized=%d "
            "sampled=%d accepted=%d rejected(recent=%d blocked=%d clearance=%d unreachable=%d path=%d) target=%s"
            % (
                selection_stats["label"],
                selection_stats["cluster_count"],
                selection_stats["frontier_cells"],
                selection_stats["min_cluster_cells"],
                selection_stats["max_unknown_path_ratio"],
                selection_stats["clusters_eligible"],
                selection_stats["clusters_with_candidate"],
                selection_stats["clusters_penalized"],
                selection_stats["points_sampled"],
                selection_stats["points_accepted"],
                selection_stats["reject_recently_completed"],
                selection_stats["reject_blocked"],
                selection_stats["reject_clearance"],
                selection_stats["reject_unreachable"],
                selection_stats["reject_path"],
                "yes" if selection_stats["target_found"] else "no",
            )
        )

    def _project_frontier_target(self, point):
        point_x = point["x"]
        point_y = point["y"]
        offset_distance = max(0.0, self.frontier_target_offset)
        resolution = self.latest_map_stats["resolution"] if self.latest_map_stats else 0.05

        direction_x = point.get("nx", 0.0)
        direction_y = point.get("ny", 0.0)
        direction_norm = math.hypot(direction_x, direction_y)
        if direction_norm < 1e-6 and self._has_pose():
            direction_x = self.current_position_x - point_x
            direction_y = self.current_position_y - point_y
            direction_norm = math.hypot(direction_x, direction_y)
        if direction_norm < 1e-6:
            return point_x, point_y

        direction_x /= direction_norm
        direction_y /= direction_norm

        candidate_offsets = []
        for offset in [
            max(0.0, offset_distance - 2.0 * resolution),
            max(0.0, offset_distance - resolution),
            offset_distance,
            offset_distance + resolution,
        ]:
            if offset not in candidate_offsets:
                candidate_offsets.append(offset)

        for offset in candidate_offsets:
            candidate_x = point_x + direction_x * offset
            candidate_y = point_y + direction_y * offset
            center_value = self._map_value_at(candidate_x, candidate_y)
            if center_value is None or center_value < 0 or center_value >= 50:
                continue
            if not math.isinf(self._score_candidate_clearance(candidate_x, candidate_y)):
                return candidate_x, candidate_y

        return (
            point_x + direction_x * offset_distance,
            point_y + direction_y * offset_distance,
        )

    def _score_candidate_clearance(self, target_x: float, target_y: float) -> float:
        if self.latest_map is None:
            return 0.0

        center_value = self._map_value_at(target_x, target_y)
        if center_value is None or center_value < 0 or center_value >= 50:
            return float("inf")

        resolution = float(self.latest_map.info.resolution)
        if resolution <= 0.0 or self.frontier_clearance_radius <= 0.0:
            return 0.0

        radius_in_cells = max(1, int(math.ceil(self.frontier_clearance_radius / resolution)))
        unknown_hits = 0
        samples = 0
        for dy in range(-radius_in_cells, radius_in_cells + 1):
            for dx in range(-radius_in_cells, radius_in_cells + 1):
                if dx * dx + dy * dy > radius_in_cells * radius_in_cells:
                    continue
                sample_x = target_x + dx * resolution
                sample_y = target_y + dy * resolution
                value = self._map_value_at(sample_x, sample_y)
                samples += 1
                if value is None or value >= 50:
                    return float("inf")
                if value < 0:
                    unknown_hits += 1

        if samples == 0:
            return 0.0
        return (unknown_hits / float(samples)) * self.frontier_unknown_path_penalty

    def _score_path_to_point(
        self,
        target_x: float,
        target_y: float,
        max_unknown_path_ratio: float,
    ) -> float:
        if self.latest_map is None or not self._has_pose():
            return 0.0

        resolution = float(self.latest_map.info.resolution)
        if resolution <= 0.0:
            return 0.0

        distance = math.hypot(
            target_x - self.current_position_x,
            target_y - self.current_position_y,
        )
        steps = max(2, int(math.ceil(distance / max(resolution * 1.5, 0.08))))
        occupied_hits = 0
        unknown_hits = 0
        ignore_tail_steps = max(
            1,
            int(math.ceil(self.frontier_target_offset / max(resolution, 1e-6))),
        )

        for step in range(1, steps + 1):
            ratio = step / float(steps)
            sample_x = self.current_position_x + (target_x - self.current_position_x) * ratio
            sample_y = self.current_position_y + (target_y - self.current_position_y) * ratio
            map_value = self._map_value_at(sample_x, sample_y)
            if map_value is None:
                continue
            if map_value >= 50:
                occupied_hits += 1
            elif map_value < 0 and step < steps - ignore_tail_steps:
                unknown_hits += 1

        unknown_ratio = unknown_hits / float(steps)
        occupied_ratio = occupied_hits / float(steps)
        if occupied_ratio > self.frontier_max_path_occupied_ratio:
            return float("inf")
        if unknown_ratio > max_unknown_path_ratio:
            return float("inf")

        return (
            unknown_ratio * self.frontier_unknown_path_penalty
            + occupied_ratio * self.frontier_blocked_path_penalty
        )

    def _build_reachable_distance_field(self):
        if self.latest_map is None or not self._has_pose():
            return None

        width = int(self.latest_map.info.width)
        height = int(self.latest_map.info.height)
        resolution = float(self.latest_map.info.resolution)
        if width <= 0 or height <= 0 or resolution <= 0.0:
            return None

        start_cell = self._nearest_free_cell(
            self.current_position_x,
            self.current_position_y,
        )
        if start_cell is None:
            return None

        data = self.latest_map.data
        distances = [-1] * (width * height)
        queue = deque()
        start_index = start_cell[1] * width + start_cell[0]
        distances[start_index] = 0
        queue.append(start_cell)

        while queue:
            cell_x, cell_y = queue.popleft()
            current_distance = distances[cell_y * width + cell_x]
            for offset_x, offset_y in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                next_x = cell_x + offset_x
                next_y = cell_y + offset_y
                if next_x < 0 or next_x >= width or next_y < 0 or next_y >= height:
                    continue
                next_index = next_y * width + next_x
                if distances[next_index] >= 0:
                    continue
                if data[next_index] < 0 or data[next_index] >= 50:
                    continue
                distances[next_index] = current_distance + 1
                queue.append((next_x, next_y))

        return {
            "width": width,
            "height": height,
            "resolution": resolution,
            "distances": distances,
        }

    def _distance_from_field(self, distance_field, world_x: float, world_y: float) -> float:
        if distance_field is None:
            return float("inf")

        cell = self._world_to_cell(world_x, world_y)
        if cell is None:
            return float("inf")

        index = cell[1] * distance_field["width"] + cell[0]
        cell_distance = distance_field["distances"][index]
        if cell_distance < 0:
            return float("inf")
        return cell_distance * distance_field["resolution"]

    def _nearest_free_cell(self, world_x: float, world_y: float):
        cell = self._world_to_cell(world_x, world_y)
        if cell is None or self.latest_map is None:
            return None

        width = int(self.latest_map.info.width)
        height = int(self.latest_map.info.height)
        data = self.latest_map.data
        cell_x, cell_y = cell

        if data[cell_y * width + cell_x] >= 0 and data[cell_y * width + cell_x] < 50:
            return cell

        for radius in range(1, 5):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    next_x = cell_x + dx
                    next_y = cell_y + dy
                    if next_x < 0 or next_x >= width or next_y < 0 or next_y >= height:
                        continue
                    value = data[next_y * width + next_x]
                    if value >= 0 and value < 50:
                        return next_x, next_y

        return None

    def _world_to_cell(self, world_x: float, world_y: float):
        if self.latest_map is None:
            return None

        resolution = float(self.latest_map.info.resolution)
        width = int(self.latest_map.info.width)
        height = int(self.latest_map.info.height)
        if resolution <= 0.0 or width <= 0 or height <= 0:
            return None

        origin_x = float(self.latest_map.info.origin.position.x)
        origin_y = float(self.latest_map.info.origin.position.y)
        cell_x = int((world_x - origin_x) / resolution)
        cell_y = int((world_y - origin_y) / resolution)
        if cell_x < 0 or cell_x >= width or cell_y < 0 or cell_y >= height:
            return None
        return cell_x, cell_y

    def _is_frontier_target_still_valid(self, target, clusters) -> bool:
        if target is None:
            return False
        point_match_radius = max(
            self.frontier_target_switch_distance * 0.6,
            self.frontier_target_offset + self.frontier_target_reached_distance,
        )
        for cluster in clusters:
            centroid_distance = math.hypot(
                cluster["centroid_x"] - target["centroid_x"],
                cluster["centroid_y"] - target["centroid_y"],
            )
            if centroid_distance <= self.frontier_target_switch_distance:
                return True
            goal_distance = math.hypot(
                cluster["centroid_x"] - target["x"],
                cluster["centroid_y"] - target["y"],
            )
            if goal_distance <= self.frontier_target_switch_distance:
                return True
            for point in cluster.get("points", []):
                if (
                    math.hypot(point["x"] - target["x"], point["y"] - target["y"])
                    <= point_match_radius
                ):
                    return True
        return False

    def _is_frontier_target_reached(self, target) -> bool:
        if not self._has_pose() or target is None:
            return False
        return (
            math.hypot(
                target["x"] - self.current_position_x,
                target["y"] - self.current_position_y,
            )
            <= self.frontier_target_reached_distance
        )

    def _map_value_at(self, world_x: float, world_y: float):
        if self.latest_map is None:
            return None

        resolution = float(self.latest_map.info.resolution)
        width = self.latest_map.info.width
        height = self.latest_map.info.height
        if resolution <= 0.0 or width <= 0 or height <= 0:
            return None

        origin_x = float(self.latest_map.info.origin.position.x)
        origin_y = float(self.latest_map.info.origin.position.y)
        cell_x = int((world_x - origin_x) / resolution)
        cell_y = int((world_y - origin_y) / resolution)
        if cell_x < 0 or cell_x >= width or cell_y < 0 or cell_y >= height:
            return None
        return self.latest_map.data[cell_y * width + cell_x]

    def _extract_map_stats(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        if width <= 0 or height <= 0 or not msg.data:
            return None

        known_cells = 0
        free_cells = 0
        occupied_cells = 0
        min_x = width
        min_y = height
        max_x = -1
        max_y = -1

        for index, value in enumerate(msg.data):
            if value < 0:
                continue

            known_cells += 1
            if value >= 50:
                occupied_cells += 1
            else:
                free_cells += 1

            x = index % width
            y = index // width
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x)
            max_y = max(max_y, y)

        resolution = float(msg.info.resolution)
        total_cells = width * height
        if known_cells == 0:
            return {
                "known_cells": 0,
                "free_cells": 0,
                "occupied_cells": 0,
                "known_area_m2": 0.0,
                "free_area_m2": 0.0,
                "occupied_area_m2": 0.0,
                "observed_area_m2": 0.0,
                "bbox_unknown_area_m2": 0.0,
                "bbox_unknown_ratio": 1.0,
                "global_known_ratio": 0.0,
                "raw_frontier_cells": 0,
                "raw_frontier_cluster_count": 0,
                "raw_frontier_clusters": [],
                "frontier_cells": 0,
                "frontier_cluster_count": 0,
                "frontier_clusters": [],
                "residual_frontier_cells": 0,
                "residual_frontier_cluster_count": 0,
                "residual_frontier_clusters": [],
                "resolution": resolution,
                "total_cells": total_cells,
            }

        bbox_area_cells = (max_x - min_x + 1) * (max_y - min_y + 1)
        bbox_unknown_cells = 0
        for y in range(min_y, max_y + 1):
            row_start = y * width + min_x
            row_end = y * width + max_x + 1
            for value in msg.data[row_start:row_end]:
                if value < 0:
                    bbox_unknown_cells += 1

        frontier_clusters_all = self._extract_frontier_clusters(msg)
        raw_frontier_cells = sum(cluster["size"] for cluster in frontier_clusters_all)
        frontier_clusters = self._filter_frontier_clusters(
            frontier_clusters_all, self.frontier_cluster_min_cells
        )
        residual_frontier_clusters = self._filter_frontier_clusters(
            frontier_clusters_all, self.frontier_residual_cluster_min_cells
        )
        frontier_cells = sum(cluster["size"] for cluster in frontier_clusters)
        residual_frontier_cells = sum(
            cluster["size"] for cluster in residual_frontier_clusters
        )

        cell_area = resolution * resolution
        return {
            "known_cells": known_cells,
            "free_cells": free_cells,
            "occupied_cells": occupied_cells,
            "known_area_m2": known_cells * cell_area,
            "free_area_m2": free_cells * cell_area,
            "occupied_area_m2": occupied_cells * cell_area,
            "observed_area_m2": bbox_area_cells * cell_area,
            "bbox_unknown_area_m2": bbox_unknown_cells * cell_area,
            "bbox_unknown_ratio": bbox_unknown_cells / float(bbox_area_cells),
            "global_known_ratio": known_cells / float(total_cells),
            "raw_frontier_cells": raw_frontier_cells,
            "raw_frontier_cluster_count": len(frontier_clusters_all),
            "raw_frontier_clusters": frontier_clusters_all,
            "frontier_cells": frontier_cells,
            "frontier_cluster_count": len(frontier_clusters),
            "frontier_clusters": frontier_clusters,
            "residual_frontier_cells": residual_frontier_cells,
            "residual_frontier_cluster_count": len(residual_frontier_clusters),
            "residual_frontier_clusters": residual_frontier_clusters,
            "resolution": resolution,
            "total_cells": total_cells,
        }

    def _extract_frontier_clusters(self, msg: OccupancyGrid):
        width = msg.info.width
        height = msg.info.height
        data = msg.data
        resolution = float(msg.info.resolution)
        origin_x = float(msg.info.origin.position.x)
        origin_y = float(msg.info.origin.position.y)

        frontier_indices = set()
        frontier_normals = {}
        for y in range(1, height - 1):
            row_offset = y * width
            for x in range(1, width - 1):
                index = row_offset + x
                value = data[index]
                if value < 0 or value >= 50:
                    continue
                if self._has_unknown_neighbor(data, width, height, x, y):
                    frontier_indices.add(index)
                    frontier_normals[index] = self._frontier_free_space_normal(
                        data, width, height, x, y
                    )

        clusters = []
        while frontier_indices:
            seed = frontier_indices.pop()
            queue = deque([seed])
            cluster = [seed]

            while queue:
                current = queue.popleft()
                cx = current % width
                cy = current // width
                for ny in range(max(1, cy - 1), min(height - 1, cy + 2)):
                    for nx in range(max(1, cx - 1), min(width - 1, cx + 2)):
                        neighbor = ny * width + nx
                        if neighbor in frontier_indices:
                            frontier_indices.remove(neighbor)
                            queue.append(neighbor)
                            cluster.append(neighbor)

            centroid_x = 0.0
            centroid_y = 0.0
            points = []
            for index in cluster:
                cell_x = index % width
                cell_y = index // width
                world_x = origin_x + (cell_x + 0.5) * resolution
                world_y = origin_y + (cell_y + 0.5) * resolution
                centroid_x += world_x
                centroid_y += world_y
                normal_x, normal_y = frontier_normals.get(index, (0.0, 0.0))
                points.append(
                    {
                        "x": world_x,
                        "y": world_y,
                        "nx": normal_x,
                        "ny": normal_y,
                    }
                )

            size = len(cluster)
            clusters.append(
                {
                    "size": size,
                    "centroid_x": centroid_x / size,
                    "centroid_y": centroid_y / size,
                    "points": points,
                }
            )

        return clusters

    def _filter_frontier_clusters(self, clusters, min_cells: int):
        return [cluster for cluster in clusters if cluster["size"] >= max(1, min_cells)]

    def _frontier_free_space_normal(
        self, data, width: int, height: int, x: int, y: int
    ):
        sum_x = 0.0
        sum_y = 0.0
        for ny in range(max(0, y - 1), min(height, y + 2)):
            for nx in range(max(0, x - 1), min(width, x + 2)):
                if nx == x and ny == y:
                    continue
                if data[ny * width + nx] < 0:
                    sum_x -= float(nx - x)
                    sum_y -= float(ny - y)

        norm = math.hypot(sum_x, sum_y)
        if norm < 1e-6:
            return 0.0, 0.0
        return sum_x / norm, sum_y / norm

    def _has_unknown_neighbor(self, data, width: int, height: int, x: int, y: int) -> bool:
        for ny in range(max(0, y - 1), min(height, y + 2)):
            for nx in range(max(0, x - 1), min(width, x + 2)):
                if nx == x and ny == y:
                    continue
                if data[ny * width + nx] < 0:
                    return True
        return False

    def _map_progressed(self, stats) -> bool:
        return (
            stats["known_area_m2"] - self.progress_baseline_known_area_m2
            >= self.map_progress_min_known_area
            or stats["observed_area_m2"] - self.progress_baseline_observed_area_m2
            >= self.map_progress_min_observed_area
            or self.progress_baseline_bbox_unknown_area_m2 - stats["bbox_unknown_area_m2"]
            >= self.map_progress_min_bbox_unknown_area_reduction
        )

    def _update_map_progress_baseline(self, stats) -> None:
        self.progress_baseline_known_area_m2 = stats["known_area_m2"]
        self.progress_baseline_free_area_m2 = stats["free_area_m2"]
        self.progress_baseline_observed_area_m2 = stats["observed_area_m2"]
        self.progress_baseline_bbox_unknown_area_m2 = stats["bbox_unknown_area_m2"]

    def _evaluate_save_reason(self, now_ns: int, elapsed: float):
        if self.max_exploration_timeout > 0.0 and elapsed >= self.max_exploration_timeout:
            return "达到硬超时 %.1fs" % self.max_exploration_timeout

        if elapsed < max(self.exploration_timeout, self.minimum_exploration_time):
            return None

        if self.latest_map_stats is None:
            self._log_status(
                now_ns,
                "软超时已到，但地图尚未可用，继续等待 SLAM 更新。",
            )
            return None

        stalled_for = self._seconds_since(self.last_map_progress_time_ns, now_ns)
        if stalled_for < self.map_stall_duration:
            self._log_status(
                now_ns,
                "软超时已到，但地图 %.1fs 内仍有增长，继续探索。"
                % stalled_for,
            )
            return None

        observed_ready = (
            self.latest_map_stats["observed_area_m2"] >= self.completion_min_observed_area
        )
        bbox_ratio_ready = (
            self.latest_map_stats["bbox_unknown_ratio"]
            <= self.completion_max_bbox_unknown_ratio
        )
        bbox_area_ready = (
            self.latest_map_stats["bbox_unknown_area_m2"]
            <= self.completion_max_bbox_unknown_area
        )
        if self.completion_require_both_bbox_metrics:
            bbox_ready = bbox_ratio_ready and bbox_area_ready
        else:
            bbox_ready = bbox_ratio_ready or bbox_area_ready
        frontier_ready = (
            self.latest_map_stats["frontier_cluster_count"]
            <= self.frontier_completion_max_clusters
            and self.latest_map_stats["frontier_cells"]
            <= self.frontier_completion_max_cells
        )
        residual_frontier_ready = True
        if self.frontier_residual_cluster_min_cells < self.frontier_cluster_min_cells:
            residual_frontier_ready = (
                self.latest_map_stats["residual_frontier_cluster_count"]
                <= self.frontier_completion_max_clusters
                and self.latest_map_stats["residual_frontier_cells"]
                <= self.frontier_completion_max_cells
            )
        if observed_ready and bbox_ready and frontier_ready and residual_frontier_ready:
            return (
                "地图已停滞 %.1fs，frontier 已耗尽，观测包围盒 %.1fm^2，包围盒未知面积 %.1fm^2"
                % (
                    stalled_for,
                    self.latest_map_stats["observed_area_m2"],
                    self.latest_map_stats["bbox_unknown_area_m2"],
                )
            )

        self._log_status(
            now_ns,
            "软超时已到，但覆盖仍不足：raw frontier %d 簇/%d 格，主 frontier %d 簇/%d 格，残余 frontier %d 簇/%d 格，观测包围盒 %.1fm^2 / 目标 %.1fm^2，继续探索。"
            % (
                self.latest_map_stats["raw_frontier_cluster_count"],
                self.latest_map_stats["raw_frontier_cells"],
                self.latest_map_stats["frontier_cluster_count"],
                self.latest_map_stats["frontier_cells"],
                self.latest_map_stats["residual_frontier_cluster_count"],
                self.latest_map_stats["residual_frontier_cells"],
                self.latest_map_stats["observed_area_m2"],
                self.completion_min_observed_area,
            ),
        )
        return None

    def _mark_target_failed(self, target, reason: str) -> None:
        now_ns = self.get_clock().now().nanoseconds
        self._mark_failed_cluster(target, reason, now_ns)
        for record in self.blocked_targets:
            if (
                math.hypot(record["x"] - target["x"], record["y"] - target["y"])
                <= self.frontier_failed_goal_radius
            ):
                record["attempts"] += 1
                backoff = max(
                    1.0,
                    self.frontier_failed_goal_backoff ** max(0, record["attempts"] - 1),
                )
                record["expires_ns"] = now_ns + int(
                    self.frontier_failed_goal_cooldown * backoff * 1e9
                )
                record["reason"] = reason
                self.get_logger().warn(
                    "frontier 目标失败，加入冷却：reason=%s attempts=%d goal=(%.2f, %.2f)"
                    % (
                        reason,
                        record["attempts"],
                        target["x"],
                        target["y"],
                    )
                )
                self._maybe_clear_costmaps(now_ns)
                return

        self.blocked_targets.append(
            {
                "x": target["x"],
                "y": target["y"],
                "attempts": 1,
                "reason": reason,
                "expires_ns": now_ns + int(self.frontier_failed_goal_cooldown * 1e9),
            }
        )
        self.get_logger().warn(
            "frontier 目标失败，加入冷却：reason=%s attempts=1 goal=(%.2f, %.2f)"
            % (reason, target["x"], target["y"])
        )
        self._maybe_clear_costmaps(now_ns)

    def _is_blocked_target(self, x: float, y: float, now_ns: int) -> bool:
        for record in self.blocked_targets:
            if record["expires_ns"] <= now_ns:
                continue
            if math.hypot(record["x"] - x, record["y"] - y) <= self.frontier_failed_goal_radius:
                if record["attempts"] >= self.frontier_failed_goal_max_retries:
                    return True
                return True
        return False

    def _prune_blocked_targets(self, now_ns: int) -> None:
        self.blocked_targets = [
            record for record in self.blocked_targets if record["expires_ns"] > now_ns
        ]

    def _prune_completed_targets(self, now_ns: int) -> None:
        self.completed_targets = [
            record for record in self.completed_targets if record["expires_ns"] > now_ns
        ]

    def _prune_failed_clusters(self, now_ns: int) -> None:
        self.failed_clusters = [
            record for record in self.failed_clusters if record["expires_ns"] > now_ns
        ]

    def _clear_blocked_targets_on_progress(self) -> None:
        retained = []
        for record in self.blocked_targets:
            if record["attempts"] >= self.frontier_failed_goal_max_retries:
                retained.append(record)
        self.blocked_targets = retained
        self.failed_clusters = []

    def _clear_completed_targets_on_progress(self) -> None:
        self.completed_targets = []

    def _mark_target_completed(self, target) -> None:
        if self.frontier_reached_goal_cooldown <= 0.0:
            return

        now_ns = self.get_clock().now().nanoseconds
        for record in self.completed_targets:
            same_goal = (
                math.hypot(record["x"] - target["x"], record["y"] - target["y"])
                <= self.frontier_reached_goal_radius
            )
            same_cluster = (
                math.hypot(
                    record["centroid_x"] - target["centroid_x"],
                    record["centroid_y"] - target["centroid_y"],
                )
                <= self.frontier_reached_cluster_radius
            )
            if same_goal or same_cluster:
                record["x"] = target["x"]
                record["y"] = target["y"]
                record["centroid_x"] = target["centroid_x"]
                record["centroid_y"] = target["centroid_y"]
                record["expires_ns"] = now_ns + int(
                    self.frontier_reached_goal_cooldown * 1e9
                )
                return

        self.completed_targets.append(
            {
                "x": target["x"],
                "y": target["y"],
                "centroid_x": target["centroid_x"],
                "centroid_y": target["centroid_y"],
                "expires_ns": now_ns + int(self.frontier_reached_goal_cooldown * 1e9),
            }
        )

    def _is_recently_completed_target(
        self, cluster, target_x: float, target_y: float, now_ns: int
    ) -> bool:
        for record in self.completed_targets:
            if record["expires_ns"] <= now_ns:
                continue
            same_goal = (
                math.hypot(record["x"] - target_x, record["y"] - target_y)
                <= self.frontier_reached_goal_radius
            )
            same_cluster = (
                math.hypot(
                    record["centroid_x"] - cluster["centroid_x"],
                    record["centroid_y"] - cluster["centroid_y"],
                )
                <= self.frontier_reached_cluster_radius
            )
            if same_goal or same_cluster:
                return True
        return False

    def _mark_failed_cluster(self, target, reason: str, now_ns: int) -> None:
        if reason not in {"goal_no_progress", "STATUS_ABORTED"}:
            return

        cluster_radius = max(
            self.frontier_target_switch_distance,
            self.frontier_failed_goal_radius * 2.0,
        )
        cluster_backoff_multiplier = max(1.0, self.frontier_failed_goal_backoff)
        for record in self.failed_clusters:
            if (
                math.hypot(
                    record["centroid_x"] - target["centroid_x"],
                    record["centroid_y"] - target["centroid_y"],
                )
                <= cluster_radius
            ):
                record["attempts"] += 1
                record["expires_ns"] = now_ns + int(
                    self.frontier_failed_goal_cooldown
                    * (cluster_backoff_multiplier ** max(0, record["attempts"] - 1))
                    * 1e9
                )
                record["reason"] = reason
                return

        self.failed_clusters.append(
            {
                "centroid_x": target["centroid_x"],
                "centroid_y": target["centroid_y"],
                "attempts": 1,
                "reason": reason,
                "expires_ns": now_ns + int(self.frontier_failed_goal_cooldown * 1e9),
            }
        )

    def _cluster_failure_penalty(self, cluster, now_ns: int) -> float:
        penalty_radius = max(
            self.frontier_target_switch_distance,
            self.frontier_failed_goal_radius * 2.0,
        )
        penalty_unit = max(
            self.frontier_target_switch_distance,
            self.frontier_target_reached_distance * 2.0,
            0.75,
        )
        highest_attempts = 0
        for record in self.failed_clusters:
            if record["expires_ns"] <= now_ns:
                continue
            if (
                math.hypot(
                    record["centroid_x"] - cluster["centroid_x"],
                    record["centroid_y"] - cluster["centroid_y"],
                )
                <= penalty_radius
            ):
                highest_attempts = max(highest_attempts, record["attempts"])
        return penalty_unit * float(highest_attempts)

    def _maybe_clear_costmaps(self, now_ns: int) -> None:
        if not self.clear_costmap_on_failure:
            return
        if now_ns - self.last_costmap_clear_ns < int(self.costmap_clear_cooldown * 1e9):
            return

        cleared_services = []
        for service_name, client in self.clear_costmap_clients.items():
            if client.wait_for_service(timeout_sec=0.1):
                client.call_async(ClearEntireCostmap.Request())
                cleared_services.append(service_name)

        if cleared_services:
            self.last_costmap_clear_ns = now_ns
            self.get_logger().info(
                "已请求清理 Nav2 costmap：%s" % ", ".join(cleared_services)
            )

    def _request_map_save(self) -> None:
        if self.save_requested:
            return

        save_map_client = self._get_ready_save_map_client()
        if save_map_client is None:
            self.get_logger().warn(
                "等待存图服务就绪，已尝试：%s"
                % ", ".join(self.save_map_service_candidates)
            )
            return

        request = SaveMap.Request()
        request.map_topic = self.map_topic
        request.map_url = self.save_map_url
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        self.save_future = save_map_client.call_async(request)
        self.save_requested = True
        self.save_retry_deadline_ns = self.get_clock().now().nanoseconds + int(20.0 * 1e9)
        self.get_logger().info("已通过服务 %s 发起存图请求。" % self.active_save_map_service)

    def _check_save_result(self) -> None:
        if self.save_future is None:
            return

        if self.save_future.done():
            try:
                response = self.save_future.result()
            except Exception as exc:  # pragma: no cover - runtime guard
                self.get_logger().error("存图调用抛出异常：%s" % exc)
                self.save_requested = False
                self.save_future = None
                return

            if response is not None and response.result:
                self.get_logger().info("地图已成功保存到 %s" % self.save_map_url)
                rclpy.shutdown()
                return

            self.get_logger().error("存图失败，%.1f 秒后重试。" % self.save_retry_interval)
            self.save_requested = False
            self.save_future = None
            self.save_retry_cooldown_until_ns = self.get_clock().now().nanoseconds + int(
                self.save_retry_interval * 1e9
            )
            return

        if self.get_clock().now().nanoseconds >= self.save_retry_deadline_ns:
            self.get_logger().warn("存图请求超时，准备重试。")
            self.save_requested = False
            self.save_future = None
            self.save_retry_cooldown_until_ns = self.get_clock().now().nanoseconds + int(
                self.save_retry_interval * 1e9
            )

    def _get_ready_save_map_client(self):
        for service_name, client in self.save_map_clients.items():
            if client.wait_for_service(timeout_sec=0.2):
                self.active_save_map_service = service_name
                return client
        return None

    def _normalize_map_save_url(self, save_map_url: str) -> str:
        root, ext = os.path.splitext(save_map_url)
        if ext.lower() in [".yaml", ".pgm"]:
            self.get_logger().warn(
                "检测到存图路径包含后缀 %s，已自动改为基路径 %s" % (ext, root)
            )
            return root
        return save_map_url

    def _set_exploration_state(self, new_state: str) -> None:
        if self.exploration_state != new_state:
            self.get_logger().info(
                "探索状态切换：%s -> %s" % (self.exploration_state, new_state)
            )
            self.exploration_state = new_state

    def _log_status(self, now_ns: int, message: str) -> None:
        if now_ns - self.last_status_log_ns >= int(self.status_log_interval * 1e9):
            self.get_logger().info(message)
            self.last_status_log_ns = now_ns

    def _seconds_since(self, timestamp_ns: int, now_ns: int) -> float:
        if timestamp_ns <= 0:
            return 0.0
        return (now_ns - timestamp_ns) / 1e9

    def _goal_status_name(self, status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: "STATUS_UNKNOWN",
            GoalStatus.STATUS_ACCEPTED: "STATUS_ACCEPTED",
            GoalStatus.STATUS_EXECUTING: "STATUS_EXECUTING",
            GoalStatus.STATUS_CANCELING: "STATUS_CANCELING",
            GoalStatus.STATUS_SUCCEEDED: "STATUS_SUCCEEDED",
            GoalStatus.STATUS_CANCELED: "STATUS_CANCELED",
            GoalStatus.STATUS_ABORTED: "STATUS_ABORTED",
        }
        return names.get(status, "STATUS_%d" % status)

    def _target_yaw_error(self, target_x: float, target_y: float) -> float:
        target_yaw = math.atan2(
            target_y - self.current_position_y,
            target_x - self.current_position_x,
        )
        return self._normalize_angle(target_yaw - self.current_yaw)

    def _has_pose(self) -> bool:
        return (
            self.current_position_x is not None
            and self.current_position_y is not None
            and self.current_yaw is not None
        )

    def _yaw_from_quaternion(self, quaternion) -> float:
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _normalize_angle(self, angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutoExploreAndSave()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
