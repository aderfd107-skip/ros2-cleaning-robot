# cleaning_robot_bringup

这个包负责当前 ROS 2 Humble 扫地机器人项目的系统启动与基础验证流程。

## 当前职责

- 通过 `cleaning_robot_simulation` 启动可切换 world 的 Gazebo 验证环境
- 启动 SLAM、自动探索与自动存图
- 启动基于已知地图的 Nav2 + AMCL 导航
- 存放 bringup 共用配置、RViz 配置与基线地图

当前阶段仍然以“小而稳”的基础链路为主，后续阶段再逐步增强安全避障、卡住恢复、状态机和覆盖式清扫。

## 启动入口

### 1. SLAM 基线验证

```bash
cd ~/cleaning_robot/cleaning_robot_ws
source install/setup.bash
ros2 launch cleaning_robot_bringup slam.launch.py
```

作用：

- 启动可切换 world 的 Gazebo 仿真
- 启动 `slam_toolbox`
- 启动自动探索节点
- 在任务结束时保存占据栅格地图
- 打开 SLAM 专用 RViz 配置

关键 launch 参数：

- `map_output`：保存地图的绝对基路径，不带 `.yaml` 或 `.pgm` 后缀
- `world_name`：内置仿真场景名，支持 `empty_room`、`single_obstacle`、`multi_obstacle`、`narrow_passage`、`multi_room`
- `world`：自定义 world 绝对路径，设置后会覆盖 `world_name`
- `exploration_timeout`：探索超时时间，单位秒
- `use_sim_time`：是否使用仿真时间
- `enable_rviz`：是否启动 RViz
- `rviz_config`：RViz 配置文件路径
- `auto_explore_start_delay`：自动探索节点延迟启动时间

### 2. 已知地图导航验证

```bash
cd ~/cleaning_robot/cleaning_robot_ws
source install/setup.bash
ros2 launch cleaning_robot_bringup nav.launch.py
```

作用：

- 启动可切换 world 的 Gazebo 仿真
- 加载已保存地图并启动 Nav2
- 启动 AMCL 定位
- 打开用于 2D Goal 验证的 RViz

关键 launch 参数：

- `map`：待加载地图 yaml 的绝对路径
- `world_name`：内置仿真场景名，支持 `empty_room`、`single_obstacle`、`multi_obstacle`、`narrow_passage`、`multi_room`
- `world`：自定义 world 绝对路径，设置后会覆盖 `world_name`
- `use_sim_time`：是否使用仿真时间
- `enable_rviz`：是否启动 RViz
- `rviz_config`：RViz 配置文件路径

## 基线接口

### Topics

- 输入：`/scan`
- 输入：`/odom`
- 输出：`/cmd_vel`
- 输出：`/map`

### Services

- `/map_saver/save_map`

### TF 关系

- `map -> odom -> base_footprint -> base_link -> lidar_link`

## 阶段 5 仿真回归

- 详细说明见 [docs/phase5_simulation_regression.md](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/docs/phase5_simulation_regression.md)
- 轻量 smoke 脚本：`ros2 run cleaning_robot_bringup phase5_world_smoke.sh 12`
- 兼容旧入口：`cleaning_robot_simulation/worlds/room.world` 保留为单障碍物基线别名

## 阶段 0 验收清单

在继续做更复杂能力前，当前项目至少应满足以下基线要求：

1. `slam.launch.py` 在单房间世界中可以直接启动，不需要手改源码。
2. TF 链对 SLAM 和导航是完整可用的：
   - `odom -> base_footprint`
   - `base_footprint -> base_link`
   - `base_link -> lidar_link`
3. 自动探索时机器人可以稳定发布 `/cmd_vel` 并产生移动。
4. 在设定超时时间内能够成功存图。
5. `nav.launch.py` 可以直接加载默认地图进入导航流程。
6. RViz 可以通过 launch 参数开关。

## 当前验证环境

- 世界文件预设：
  - `cleaning_robot_simulation/worlds/empty_room.world`
  - `cleaning_robot_simulation/worlds/single_obstacle.world`
  - `cleaning_robot_simulation/worlds/multi_obstacle.world`
  - `cleaning_robot_simulation/worlds/narrow_passage.world`
  - `cleaning_robot_simulation/worlds/multi_room.world`
- 兼容别名：`cleaning_robot_simulation/worlds/room.world`
- 基线地图：`maps/room_map.yaml`
- 机器人模型：`cleaning_robot_description/urdf/robot.urdf.xacro`

在安全避障、卡住恢复等能力稳定前，`single_obstacle` 仍然作为默认回归环境。
