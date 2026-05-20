import heapq
import math
import os
from collections import deque

from ament_index_python.packages import PackageNotFoundError
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid, Path
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import yaml


FREE = 0
BLOCKED = 1


class CoveragePlanner(Node):
    def __init__(self):
        super().__init__("coverage_planner")

        self.declare_parameter("map_yaml", self._default_map_yaml())
        self.declare_parameter("use_map_topic", False)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("path_topic", "/coverage_path")
        self.declare_parameter("waypoints_topic", "/coverage_waypoints")
        self.declare_parameter("line_spacing", 0.30)
        self.declare_parameter("wall_margin", 0.20)
        self.declare_parameter("obstacle_margin", 0.20)
        self.declare_parameter("min_segment_length", 0.20)
        self.declare_parameter("waypoint_spacing", 0.30)
        self.declare_parameter("turn_margin", 0.25)
        self.declare_parameter("publish_period", 2.0)
        self.declare_parameter("frame_id", "map")

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.path_pub = self.create_publisher(
            Path, self.get_parameter("path_topic").value, qos
        )
        self.waypoints_pub = self.create_publisher(
            PoseArray, self.get_parameter("waypoints_topic").value, qos
        )

        self._path = None
        self._waypoints = None
        self._planned_from_topic = False

        if self.get_parameter("use_map_topic").value:
            self.create_subscription(
                OccupancyGrid,
                self.get_parameter("map_topic").value,
                self._on_map,
                qos,
            )
            self.get_logger().info(
                f"Waiting for OccupancyGrid on {self.get_parameter('map_topic').value}"
            )
        else:
            map_yaml = self.get_parameter("map_yaml").value
            try:
                grid = self._load_grid_from_yaml(map_yaml)
                self._plan_and_store(grid)
            except Exception as exc:
                self.get_logger().error(f"Failed to load map yaml '{map_yaml}': {exc}")

        period = float(self.get_parameter("publish_period").value)
        self.create_timer(max(period, 0.1), self._publish_plan)

    def _default_map_yaml(self):
        try:
            bringup_dir = get_package_share_directory("cleaning_robot_bringup")
            return os.path.join(bringup_dir, "maps", "room_map.yaml")
        except PackageNotFoundError:
            return ""

    def _on_map(self, msg):
        if self._planned_from_topic:
            return
        self._planned_from_topic = True
        self._plan_and_store(msg)

    def _plan_and_store(self, grid_msg):
        planner = GridCoveragePlanner(
            grid_msg,
            line_spacing=float(self.get_parameter("line_spacing").value),
            wall_margin=float(self.get_parameter("wall_margin").value),
            obstacle_margin=float(self.get_parameter("obstacle_margin").value),
            min_segment_length=float(self.get_parameter("min_segment_length").value),
            waypoint_spacing=float(self.get_parameter("waypoint_spacing").value),
            turn_margin=float(self.get_parameter("turn_margin").value),
        )
        cells = planner.plan()
        path_msg, waypoints_msg = self._cells_to_messages(grid_msg, cells)
        self._path = path_msg
        self._waypoints = waypoints_msg
        self.get_logger().info(
            "Generated coverage path with "
            f"{len(path_msg.poses)} path poses and {len(waypoints_msg.poses)} waypoints"
        )
        if planner.skipped_segments:
            self.get_logger().warn(
                f"Skipped {planner.skipped_segments} unreachable coverage segments"
            )
        self._publish_plan()

    def _cells_to_messages(self, grid_msg, cells):
        frame_id = self.get_parameter("frame_id").value or grid_msg.header.frame_id or "map"
        stamp = self.get_clock().now().to_msg()

        path = Path()
        path.header.frame_id = frame_id
        path.header.stamp = stamp

        waypoints = PoseArray()
        waypoints.header = path.header

        last_yaw = 0.0
        for index, cell in enumerate(cells):
            x, y = self._cell_to_world(grid_msg, cell)
            if index + 1 < len(cells):
                nx, ny = self._cell_to_world(grid_msg, cells[index + 1])
                last_yaw = math.atan2(ny - y, nx - x)

            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation = yaw_to_quaternion(last_yaw)

            stamped = PoseStamped()
            stamped.header = path.header
            stamped.pose = pose
            path.poses.append(stamped)
            waypoints.poses.append(pose)

        return path, waypoints

    def _cell_to_world(self, grid_msg, cell):
        mx, my = cell
        info = grid_msg.info
        yaw = quaternion_to_yaw(info.origin.orientation)
        local_x = (mx + 0.5) * info.resolution
        local_y = (my + 0.5) * info.resolution
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        world_x = info.origin.position.x + local_x * cos_yaw - local_y * sin_yaw
        world_y = info.origin.position.y + local_x * sin_yaw + local_y * cos_yaw
        return world_x, world_y

    def _publish_plan(self):
        if self._path is None:
            return
        stamp = self.get_clock().now().to_msg()
        self._path.header.stamp = stamp
        self._waypoints.header.stamp = stamp
        for pose in self._path.poses:
            pose.header.stamp = stamp
        self.path_pub.publish(self._path)
        self.waypoints_pub.publish(self._waypoints)

    def _load_grid_from_yaml(self, yaml_path):
        if not yaml_path:
            raise ValueError("map_yaml parameter is empty")
        with open(yaml_path, "r", encoding="utf-8") as yaml_file:
            meta = yaml.safe_load(yaml_file)

        image_path = meta["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)

        width, height, pixels = read_pgm(image_path)
        negate = int(meta.get("negate", 0))
        mode = str(meta.get("mode", "trinary")).lower()
        occupied_thresh = float(meta.get("occupied_thresh", 0.65))
        free_thresh = float(meta.get("free_thresh", 0.25))

        data = [-1] * (width * height)
        for image_y in range(height):
            map_y = height - image_y - 1
            for x in range(width):
                gray = pixels[image_y * width + x]
                data[map_y * width + x] = pgm_gray_to_occupancy(
                    gray,
                    mode=mode,
                    negate=negate,
                    occupied_thresh=occupied_thresh,
                    free_thresh=free_thresh,
                )

        grid = OccupancyGrid()
        grid.header.frame_id = self.get_parameter("frame_id").value
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = float(meta["resolution"])
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(meta["origin"][0])
        grid.info.origin.position.y = float(meta["origin"][1])
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation = yaw_to_quaternion(float(meta["origin"][2]))
        grid.data = data
        return grid


class GridCoveragePlanner:
    def __init__(
        self,
        grid_msg,
        line_spacing,
        wall_margin,
        obstacle_margin,
        min_segment_length,
        waypoint_spacing,
        turn_margin,
    ):
        self.grid_msg = grid_msg
        self.width = grid_msg.info.width
        self.height = grid_msg.info.height
        self.resolution = grid_msg.info.resolution
        self.line_spacing_cells = max(1, int(round(line_spacing / self.resolution)))
        self.wall_margin_cells = max(0, int(math.ceil(wall_margin / self.resolution)))
        self.obstacle_margin_cells = max(
            0, int(math.ceil(obstacle_margin / self.resolution))
        )
        self.min_segment_cells = max(1, int(round(min_segment_length / self.resolution)))
        self.waypoint_spacing_cells = max(1, int(round(waypoint_spacing / self.resolution)))
        self.turn_margin_cells = max(0, int(math.ceil(turn_margin / self.resolution)))
        self.skipped_segments = 0
        self.raw_free = self._build_raw_free()
        self.free = self._inflate_blocked_cells()

    def plan(self):
        row_segments = self._scan_segments()
        if not row_segments:
            return []

        cells = self._decompose_cells(row_segments)
        if not cells:
            return []

        ordered_paths = []
        remaining = list(range(len(cells)))
        current = None

        while remaining:
            choice = self._choose_next_cell(cells, remaining, current)
            if choice is None:
                self.skipped_segments += len(remaining)
                break

            cell_index, cell_path, connector = choice
            remaining.remove(cell_index)

            if connector:
                ordered_paths.extend(connector[1:] if ordered_paths else connector)

            ordered_paths.extend(cell_path if not ordered_paths else cell_path[1:])
            current = cell_path[-1]

        return self._make_waypoints(self._dedupe_consecutive(ordered_paths))

    def _build_raw_free(self):
        free = [[False for _ in range(self.width)] for _ in range(self.height)]
        for y in range(self.height):
            row_offset = y * self.width
            for x in range(self.width):
                free[y][x] = self.grid_msg.data[row_offset + x] == 0
        return free

    def _inflate_blocked_cells(self):
        if self.obstacle_margin_cells == 0:
            return [row[:] for row in self.raw_free]

        free = [row[:] for row in self.raw_free]
        blocked = deque()
        visited = [[False for _ in range(self.width)] for _ in range(self.height)]

        for y in range(self.height):
            for x in range(self.width):
                if not self.raw_free[y][x]:
                    blocked.append((x, y, 0))
                    visited[y][x] = True

        while blocked:
            x, y, distance = blocked.popleft()
            if distance > self.obstacle_margin_cells:
                continue
            if self._in_bounds(x, y):
                free[y][x] = False
            if distance == self.obstacle_margin_cells:
                continue
            for nx, ny in self._neighbors4((x, y)):
                if not visited[ny][nx]:
                    visited[ny][nx] = True
                    blocked.append((nx, ny, distance + 1))

        return free

    def _scan_segments(self):
        rows = []
        start_y = self.wall_margin_cells
        end_y = self.height - self.wall_margin_cells
        for y in range(start_y, max(start_y, end_y), self.line_spacing_cells):
            segments = []
            x = self.wall_margin_cells
            while x < self.width - self.wall_margin_cells:
                while x < self.width - self.wall_margin_cells and not self.free[y][x]:
                    x += 1
                start_x = x
                while x < self.width - self.wall_margin_cells and self.free[y][x]:
                    x += 1
                end_x = x - 1
                if end_x < start_x:
                    continue

                start_x, end_x = self._trim_segment_for_turning(start_x, end_x)
                if end_x >= start_x and end_x - start_x + 1 >= self.min_segment_cells:
                    segments.append(
                        {
                            "start_x": start_x,
                            "end_x": end_x,
                            "y": y,
                        }
                    )
            if segments:
                rows.append({"y": y, "segments": segments})
        return rows

    def _trim_segment_for_turning(self, start_x, end_x):
        if self.turn_margin_cells <= 0:
            return start_x, end_x

        if end_x - start_x + 1 <= self.min_segment_cells + 2 * self.turn_margin_cells:
            return start_x, end_x

        return start_x + self.turn_margin_cells, end_x - self.turn_margin_cells

    def _segment_cells(self, start, end, reverse):
        sx, sy = start
        ex, _ = end
        if reverse:
            sx, ex = ex, sx
        step = 1 if ex >= sx else -1
        return [(x, sy) for x in range(sx, ex + step, step)]

    def _decompose_cells(self, row_segments):
        cells = []
        active_cells = []

        for row in row_segments:
            current_segments = row["segments"]
            overlaps_by_active = {active["cell_index"]: [] for active in active_cells}
            overlaps_by_segment = {
                segment_index: [] for segment_index in range(len(current_segments))
            }

            for active in active_cells:
                for segment_index, segment in enumerate(current_segments):
                    if self._segments_overlap(active["segment"], segment):
                        overlaps_by_active[active["cell_index"]].append(segment_index)
                        overlaps_by_segment[segment_index].append(active["cell_index"])

            next_active_cells = []
            for segment_index, segment in enumerate(current_segments):
                parent_cells = overlaps_by_segment[segment_index]
                continued_cell = None
                if len(parent_cells) == 1:
                    candidate = parent_cells[0]
                    if len(overlaps_by_active[candidate]) == 1:
                        continued_cell = candidate

                if continued_cell is None:
                    continued_cell = len(cells)
                    cells.append({"segments": []})

                cells[continued_cell]["segments"].append(segment)
                next_active_cells.append(
                    {"cell_index": continued_cell, "segment": segment}
                )

            active_cells = next_active_cells

        return [cell for cell in cells if cell["segments"]]

    def _segments_overlap(self, first, second):
        return max(first["start_x"], second["start_x"]) <= min(
            first["end_x"], second["end_x"]
        )

    def _choose_next_cell(self, cells, remaining, current):
        if current is None:
            first_index = min(remaining, key=lambda index: self._cell_sort_key(cells[index]))
            first_path = self._build_cell_path(cells[first_index])
            if not first_path:
                return None
            return first_index, first_path, []

        best_choice = None
        best_cost = float("inf")
        for cell_index in remaining:
            base_path = self._build_cell_path(cells[cell_index])
            if not base_path:
                continue

            for cell_path in (base_path, list(reversed(base_path))):
                connector = self._astar(current, cell_path[0])
                if not connector:
                    continue
                connector_cost = self._path_cost(connector)
                if connector_cost < best_cost:
                    best_cost = connector_cost
                    best_choice = (cell_index, cell_path, connector)

        return best_choice

    def _build_cell_path(self, cell):
        segments = cell["segments"]
        if not segments:
            return []

        path = []
        current = None
        reverse = False

        for segment_info in segments:
            start = (segment_info["start_x"], segment_info["y"])
            end = (segment_info["end_x"], segment_info["y"])
            segment = self._segment_cells(start, end, reverse)
            if current is not None and current != segment[0]:
                connector = self._astar(current, segment[0])
                if connector:
                    path.extend(connector[1:] if path else connector)
                else:
                    self.skipped_segments += 1
                    reverse = not reverse
                    continue
            path.extend(segment if not path else segment[1:])
            current = segment[-1]
            reverse = not reverse

        return path

    def _cell_sort_key(self, cell):
        first_segment = cell["segments"][0]
        return first_segment["y"], first_segment["start_x"]

    def _path_cost(self, path):
        if len(path) <= 1:
            return 0.0
        cost = 0.0
        for first, second in zip(path[:-1], path[1:]):
            cost += self._grid_distance(first, second)
        return cost

    def _astar(self, start, goal):
        if not self._is_free(start) or not self._is_free(goal):
            return []

        open_set = [(0.0, start)]
        came_from = {}
        cost_so_far = {start: 0.0}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                return reconstruct_path(came_from, current)

            for neighbor in self._neighbors8(current):
                if not self._is_free(neighbor):
                    continue
                if self._cuts_corner(current, neighbor):
                    continue
                move_cost = math.hypot(neighbor[0] - current[0], neighbor[1] - current[1])
                new_cost = cost_so_far[current] + move_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + self._heuristic(neighbor, goal)
                    heapq.heappush(open_set, (priority, neighbor))
                    came_from[neighbor] = current

        return []

    def _heuristic(self, cell, goal):
        return math.hypot(goal[0] - cell[0], goal[1] - cell[1])

    def _neighbors4(self, cell):
        x, y = cell
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if self._in_bounds(nx, ny):
                yield nx, ny

    def _neighbors8(self, cell):
        x, y = cell
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if self._in_bounds(nx, ny):
                    yield nx, ny

    def _is_free(self, cell):
        x, y = cell
        return self._in_bounds(x, y) and self.free[y][x]

    def _cuts_corner(self, current, neighbor):
        dx = neighbor[0] - current[0]
        dy = neighbor[1] - current[1]
        if abs(dx) != 1 or abs(dy) != 1:
            return False
        return not (
            self._is_free((current[0] + dx, current[1]))
            and self._is_free((current[0], current[1] + dy))
        )

    def _in_bounds(self, x, y):
        return 0 <= x < self.width and 0 <= y < self.height

    def _dedupe_consecutive(self, cells):
        deduped = []
        for cell in cells:
            if not deduped or deduped[-1] != cell:
                deduped.append(cell)
        return deduped

    def _make_waypoints(self, cells):
        if len(cells) <= 2:
            return cells

        waypoints = [cells[0]]
        previous_direction = self._direction(cells[0], cells[1])
        distance_since_waypoint = 0.0

        for index in range(1, len(cells) - 1):
            current_direction = self._direction(cells[index], cells[index + 1])
            distance_since_waypoint += self._grid_distance(
                cells[index - 1], cells[index]
            )

            if current_direction != previous_direction:
                if waypoints[-1] != cells[index]:
                    waypoints.append(cells[index])
                distance_since_waypoint = 0.0
                previous_direction = current_direction
            elif distance_since_waypoint >= self.waypoint_spacing_cells:
                if waypoints[-1] != cells[index]:
                    waypoints.append(cells[index])
                distance_since_waypoint = 0.0

        if waypoints[-1] != cells[-1]:
            waypoints.append(cells[-1])
        return waypoints

    def _direction(self, first, second):
        return sign(second[0] - first[0]), sign(second[1] - first[1])

    def _grid_distance(self, first, second):
        return math.hypot(second[0] - first[0], second[1] - first[1])


def read_pgm(path):
    with open(path, "rb") as pgm_file:
        magic = pgm_file.readline().strip()
        if magic != b"P5":
            raise ValueError(f"Only binary PGM P5 maps are supported, got {magic!r}")
        width, height = read_pgm_values(pgm_file, 2)
        max_value = read_pgm_values(pgm_file, 1)[0]
        if max_value > 255:
            raise ValueError("Only 8-bit PGM maps are supported")
        pixels = list(pgm_file.read(width * height))
        if len(pixels) != width * height:
            raise ValueError("PGM image ended before all pixels were read")
    return width, height, pixels


def read_pgm_values(pgm_file, count):
    values = []
    while len(values) < count:
        line = pgm_file.readline()
        if not line:
            raise ValueError("Unexpected end of PGM header")
        line = line.split(b"#", 1)[0].strip()
        if not line:
            continue
        values.extend(int(value) for value in line.split())
    return values


def pgm_gray_to_occupancy(gray, mode, negate, occupied_thresh, free_thresh):
    if mode == "trinary":
        if negate:
            if gray <= 5:
                return 0
            if gray >= 250:
                return 100
        else:
            if gray >= 250:
                return 0
            if gray <= 5:
                return 100
        return -1

    color = gray / 255.0
    occ = color if negate else 1.0 - color
    if occ > occupied_thresh:
        return 100
    if occ < free_thresh:
        return 0
    return -1


def reconstruct_path(came_from, current):
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    path.reverse()
    return path


def sign(value):
    if value > 0:
        return 1
    if value < 0:
        return -1
    return 0


def quaternion_to_yaw(quaternion):
    siny_cosp = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy_cosp = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    quaternion = Quaternion()
    quaternion.z = math.sin(yaw * 0.5)
    quaternion.w = math.cos(yaw * 0.5)
    return quaternion


def main():
    rclpy.init()
    node = CoveragePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
