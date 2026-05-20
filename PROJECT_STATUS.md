# ROS 2 扫地机器人项目状态

本文档记录阶段 0 的基线验证与项目整理状态。当前范围只覆盖
`cleaning_robot_description`、`cleaning_robot_simulation`、`cleaning_robot_bringup`
三个包，不包含覆盖清扫、任务状态机或复杂环境扩展。

## 当前包职责

- `cleaning_robot_description`：机器人 URDF/Xacro 描述，包含底盘、轮子、脚轮、雷达与 IMU；提供 `display.launch.py` 用于模型显示检查。
- `cleaning_robot_simulation`：Gazebo 单房间仿真环境；提供 `sim.launch.py` 启动 Gazebo、`robot_state_publisher` 并生成机器人实体。
- `cleaning_robot_bringup`：系统启动入口与基线配置；提供 SLAM 自动探索存图流程、已知地图 Nav2 验证流程、RViz 配置、地图与参数文件。

## 当前已完成能力

- 单房间 Gazebo 基线环境：`room.world`，包含四面墙和一个静态障碍物。
- 机器人基础模型：圆形底盘、左右轮、前后脚轮、2D LiDAR、IMU。
- Gazebo 运动与里程计：通过 `libgazebo_ros_planar_move.so` 接收 `/cmd_vel`，发布 `/odom` 和 odom TF。
- 传感器输出：LiDAR 发布 `/scan`，IMU 发布 `/imu`。
- SLAM：`slam_toolbox` 使用 `/scan`、`odom`、`base_footprint` 生成 `/map`。
- 自动探索与存图：`auto_explore_and_save.py` 基于激光雷达和里程计发布速度命令，并在超时后调用 map saver 保存地图。
- 已知地图导航：`nav.launch.py` 使用默认 `maps/room_map.yaml` 启动 Nav2 + AMCL，可在 RViz 中进行 2D Goal 基线验证。

## 启动方式

先构建并 source 工作区：

```bash
cd ~/cleaning_robot/cleaning_robot_ws
colcon build
source install/setup.bash
```

显示机器人模型：

```bash
ros2 launch cleaning_robot_description display.launch.py
```

启动单房间仿真：

```bash
ros2 launch cleaning_robot_simulation sim.launch.py
```

启动 SLAM 基线验证并自动存图：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py
```

常用参数：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py enable_rviz:=false exploration_timeout:=90.0
ros2 launch cleaning_robot_bringup slam.launch.py map_output:=/tmp/cleaning_robot_map/room_map
```

启动已知地图导航验证：

```bash
ros2 launch cleaning_robot_bringup nav.launch.py
```

常用参数：

```bash
ros2 launch cleaning_robot_bringup nav.launch.py enable_rviz:=false
ros2 launch cleaning_robot_bringup nav.launch.py map:=/absolute/path/to/room_map.yaml
```

## Topics

核心输入输出：

- `/cmd_vel`：速度命令输入，`geometry_msgs/msg/Twist`。
- `/odom`：Gazebo 里程计输出，`nav_msgs/msg/Odometry`。
- `/scan`：2D LiDAR 输出，`sensor_msgs/msg/LaserScan`。
- `/imu`：IMU 输出。
- `/map`：SLAM 或 map server 发布的占据栅格地图。

Nav2 启动后还会出现规划、控制、costmap、行为树等 Nav2 标准 topics。

## TF

当前基线 TF 链：

```text
map -> odom -> base_footprint -> base_link -> lidar_link
                                      `-> imu_link
```

- `odom -> base_footprint`：Gazebo planar move 插件发布。
- `base_footprint -> base_link`、`base_link -> lidar_link`、`base_link -> imu_link`：`robot_state_publisher` 根据 URDF 发布。
- `map -> odom`：SLAM 或 AMCL 发布，取决于启动流程。

## Services / Actions

已明确使用的服务：

- `/map_saver/save_map`：`nav2_map_server` 的存图服务。

导航模式下由 Nav2 提供标准 lifecycle services、costmap services 与导航 actions，例如 `navigate_to_pose`。

## 文件一致性检查

- `cleaning_robot_description/CMakeLists.txt` 安装 `launch` 和 `urdf`，与 `display.launch.py`、`robot.urdf.xacro` 一致。
- `cleaning_robot_simulation/CMakeLists.txt` 安装 `launch` 和 `worlds`，与 `sim.launch.py` 默认世界 `worlds/room.world` 一致。
- `cleaning_robot_bringup/CMakeLists.txt` 安装 `launch`、`config`、`maps` 和 README，并安装 `scripts/auto_explore_and_save.py` 到包可执行目录。
- `slam.launch.py` 默认读取 `config/slam_toolbox.yaml`、`config/auto_explore.yaml`、`config/slam.rviz`，默认将地图保存到源码目录 `src/cleaning_robot_bringup/maps/room_map`。
- `nav.launch.py` 默认读取已安装包内的 `maps/room_map.yaml`、`config/nav2_params.yaml`、`config/nav.rviz`。
- `maps/room_map.yaml` 指向同目录 `room_map.pgm`，与地图文件命名一致。

## 阶段 0 验收边界

本阶段只确认项目可以构建、启动、说明清楚且目录整洁。以下内容不在阶段 0 中实现：

- 覆盖式清扫路径规划。
- 清扫任务状态机。
- 多房间或复杂动态环境。
- 更复杂的脱困、回充、语义地图或任务调度。

## 后续路线

1. 阶段 1：稳定单房间 SLAM 与地图回归，完善基础避障参数和失败案例记录。
2. 阶段 2：引入覆盖清扫规划，但保持环境简单，先验证可重复性。
3. 阶段 3：加入任务状态机，包括空闲、建图、导航、清扫、暂停、异常恢复等状态。
4. 阶段 4：扩展复杂环境、更多障碍物、回归脚本和验收指标。
5. 阶段 5：视需要接入真实机器人接口或更高保真的仿真模型。

## 阶段 0 结论

从文件结构和配置关系看，三个核心包职责清晰，build/install 规则与 launch/config/maps 基本一致。阶段 0 已完成三包 `colcon build` 验证，并通过 `ros2 launch ... --show-args` 确认 `display.launch.py`、`sim.launch.py`、`slam.launch.py`、`nav.launch.py` 均可被 ROS 2 解析。完整 Gazebo/RViz 运行验证留到后续阶段或人工验收时执行。
