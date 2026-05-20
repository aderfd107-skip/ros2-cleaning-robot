# Nav2 已知地图导航验证

本文档只验证阶段 1：在已保存的单房间地图上稳定执行 Nav2 目标点导航。
不包含覆盖清扫、任务状态机或自动路径规划。

## 启动导航

```bash
cd ~/cleaning_robot/cleaning_robot_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch cleaning_robot_bringup nav.launch.py
```

默认启动 Gazebo、Nav2、AMCL、map server 和 RViz，并使用：

- 地图：`src/cleaning_robot_bringup/maps/room_map.yaml`
- 参数：`src/cleaning_robot_bringup/config/nav2_params.yaml`
- RViz 配置：`src/cleaning_robot_bringup/config/nav.rviz`

## RViz 操作

1. 等待地图、机器人模型、激光雷达点云和 TF 都显示出来。
2. 点击 `2D Pose Estimate`，把机器人初始位姿放到 Gazebo 生成位置，通常在地图中心附近，朝向 +X。
3. 确认 `/amcl_pose` 靠近机器人真实位置，并且激光雷达点云和房间墙体对齐。
4. 点击 `Nav2 Goal`，在空闲区域选择一个可达目标点。
5. 在房间不同位置重复发送目标点，至少包含一个靠近但不贴住静态方块障碍物的目标。

## 验收标准

阶段 1 导航稳定化通过时，应满足：

- Nav2 能连续到达 3 个手动 `Nav2 Goal` 目标点。
- 全局路径保持在已知空闲区域内，不穿过未知区域。
- 机器人与墙体、静态方块障碍物保持可见安全距离。
- 接近目标点时机器人会减速收敛，不持续原地抖动或来回旋转。
- 移动后 AMCL 仍与地图对齐，没有持续性的地图/激光偏移。

## 排查建议

- 如果机器人初始位置不对，先重新发送 `2D Pose Estimate`，再发送目标点。
- 如果 Nav2 拒绝目标点，选择离墙、障碍物和未知区域更远的位置。
- 如果定位漂移，检查 `/scan`、`/odom`、`/tf` 和 `/map` 是否正常发布，并确认 RViz fixed frame 是 `map`。
- 如果机器人显得过于保守，先确认地图质量，再考虑放宽 costmap 膨胀半径或速度限制。
