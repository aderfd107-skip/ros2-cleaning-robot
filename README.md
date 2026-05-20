# ROS 2 扫地机器人仿真项目

基于 ROS 2 Humble、Gazebo 与 Nav2 的扫地机器人仿真项目。

这个仓库已经打通了一个面向已知地图场景的基础清扫闭环：机器人可在 Gazebo 中完成建模与运动仿真，使用 SLAM 建图，切换到 AMCL + Nav2 已知地图导航，生成覆盖路径，逐点下发导航目标，并基于覆盖率统计结束清扫任务。


## 当前能力

- 机器人模型描述：URDF/Xacro 定义底盘、轮子、LiDAR 和 IMU。
- Gazebo 仿真：支持单房间与多种障碍场景世界。
- SLAM 建图：基于 `slam_toolbox` 进行建图与地图保存。
- 已知地图导航：基于 Nav2 + AMCL 进行定位与导航验证。
- 自动探索：`auto_explore_and_save.py` 可执行基础探索和存图。
- 覆盖清扫闭环：生成覆盖路径、逐点发送 `NavigateToPose` 目标、统计覆盖率并按阈值结束任务。
- 基础恢复机制：支持 waypoint 失败后的重试、代价地图清理与跳点继续。

## 系统链路

```text
Gazebo world
  -> SLAM / saved map
  -> AMCL + Nav2
  -> coverage planner
  -> cleaning task manager
  -> coverage progress publisher
  -> cleaning completed / failed
```

## 仓库结构

```text
src/
├── cleaning_robot_description/   # 机器人模型与 TF 结构
├── cleaning_robot_simulation/    # Gazebo 世界与仿真启动
├── cleaning_robot_bringup/       # SLAM、Nav2、地图、清扫任务入口
└── cleaning_robot_coverage/      # 覆盖规划、进度统计与任务管理
PROJECT_STATUS.md                 # 基线状态与阶段边界
conclusion.md                     # 项目阶段总结
docs/zero-basics-guide.md         # 零基础详细讲解版说明书
```

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic
- Nav2
- `slam_toolbox`

## 快速开始

先构建并加载工作区：

```bash
cd ~/cleaning_robot/cleaning_robot_ws
colcon build
source install/setup.bash
```

显示机器人模型：

```bash
ros2 launch cleaning_robot_description display.launch.py
```

启动 Gazebo 仿真：

```bash
ros2 launch cleaning_robot_simulation sim.launch.py
```

启动 SLAM 建图与自动探索：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py
```

启动已知地图导航：

```bash
ros2 launch cleaning_robot_bringup nav.launch.py
```

启动完整清扫闭环：

```bash
ros2 launch cleaning_robot_bringup cleaning.launch.py
```

## 关键入口

- `src/cleaning_robot_description/urdf/robot.urdf.xacro`：机器人模型总入口
- `src/cleaning_robot_simulation/launch/sim.launch.py`：Gazebo 仿真入口
- `src/cleaning_robot_bringup/launch/slam.launch.py`：SLAM 建图入口
- `src/cleaning_robot_bringup/launch/nav.launch.py`：已知地图导航入口
- `src/cleaning_robot_bringup/launch/cleaning.launch.py`：完整清扫任务入口
- `src/cleaning_robot_bringup/scripts/auto_explore_and_save.py`：自动探索与存图逻辑
- `src/cleaning_robot_coverage/cleaning_robot_coverage/coverage_planner.py`：覆盖路径规划
- `src/cleaning_robot_coverage/cleaning_robot_coverage/cleaning_task_manager.py`：清扫任务状态机与 Nav2 目标调度
- `src/cleaning_robot_coverage/cleaning_robot_coverage/coverage_progress_publisher.py`：覆盖进度统计与覆盖图发布


## 后续提交

日常更新可以直接使用：

```bash
git status
git add .
git commit -m "Describe your change"
git push
```
