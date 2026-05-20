# cleaning_robot_coverage

Phase 3 coverage path planning for the cleaning robot.

This package only generates and publishes a coverage path. It does not send Nav2
goals and does not implement a cleaning task state machine.

## Topics

- Publishes `nav_msgs/Path` on `/coverage_path`
- Publishes `geometry_msgs/PoseArray` on `/coverage_waypoints`
- Optionally subscribes to `nav_msgs/OccupancyGrid` on `/map`

## Run

Build and source the workspace:

```bash
colcon build --packages-select cleaning_robot_coverage
source install/setup.bash
```

Generate a path from the saved `room_map.yaml`:

```bash
ros2 launch cleaning_robot_coverage coverage.launch.py
```

Use the live `/map` topic instead:

```bash
ros2 launch cleaning_robot_coverage coverage.launch.py use_map_topic:=true
```

Tune spacing and clearance:

```bash
ros2 launch cleaning_robot_coverage coverage.launch.py \
  line_spacing:=0.25 \
  wall_margin:=0.20 \
  obstacle_margin:=0.25 \
  waypoint_spacing:=0.30
```

## Parameters

- `map_yaml`: occupancy map yaml to load. Defaults to
  `cleaning_robot_bringup/maps/room_map.yaml`.
- `use_map_topic`: when `true`, wait for `/map` instead of loading `map_yaml`.
- `line_spacing`: distance between boustrophedon sweep lines, in meters.
- `wall_margin`: clearance from walls and map boundaries, in meters.
- `obstacle_margin`: clearance from occupied and unknown cells, in meters.
- `min_segment_length`: minimum free sweep segment length, in meters.
- `waypoint_spacing`: approximate spacing between published poses on long
  straight sweep lines, in meters.
- `publish_period`: period for republishing the latched visualization topics.

## Algorithm

1. Load an occupancy grid from `room_map.yaml`, or receive it from `/map`.
2. Treat occupied and unknown cells as blocked.
3. Inflate blocked cells by the configured clearance margin.
4. Slice the remaining free cells into horizontal sweep lines separated by
   `line_spacing`.
5. Keep only contiguous free-space line segments.
6. Visit segments in alternating left-to-right and right-to-left order.
7. Connect separated segments with A* over the inflated free-space grid.
8. Downsample the cell path into turn points and spaced waypoints for cleaner
   visualization and later execution.

The published path is therefore constrained to free cells after inflation. The
first version is intentionally simple: it produces a practical boustrophedon
coverage pattern for visualization and later execution, but it leaves goal
dispatch and task orchestration to later phases.

For `mode: trinary` maps saved by ROS map tools, white cells are treated as
free, black cells as occupied, and gray cells as unknown/blocked. If an isolated
sweep segment cannot be connected through inflated free space, it is skipped
instead of drawing a straight line through an obstacle.

## RViz Check

1. Launch the existing navigation stack or RViz with the saved map loaded.
2. Launch this package.
3. In RViz, add a `Path` display and set the topic to `/coverage_path`.
4. Optionally add a `PoseArray` display for `/coverage_waypoints`.
5. Verify that the path stays inside white free space and does not cross black
   walls or gray unknown regions.
