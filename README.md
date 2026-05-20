# ROS 2 扫地机器人仿真项目

基于 ROS 2 Humble、Gazebo 与 Nav2 的扫地机器人教学型仿真项目。

这个仓库当前已经具备机器人建模、Gazebo 仿真、SLAM 建图、已知地图导航，以及一个自动探索存图脚本和覆盖规划原型；但它还不是完整的商用扫地机器人系统，清扫任务状态机、覆盖路径执行闭环和更复杂环境验证仍待完善。

## 当前能力

- 机器人模型描述：URDF/Xacro 定义底盘、轮子、LiDAR 和 IMU。
- Gazebo 仿真：支持单房间与多种障碍场景世界。
- SLAM 建图：基于 `slam_toolbox` 进行建图与地图保存。
- 已知地图导航：基于 Nav2 + AMCL 进行定位与导航验证。
- 自动探索：`auto_explore_and_save.py` 可执行基础探索和存图。
- 覆盖规划原型：提供覆盖路径规划与任务管理方向的初步代码。

## 仓库结构

```text
src/
├── cleaning_robot_description/   # 机器人模型与 TF 结构
├── cleaning_robot_simulation/    # Gazebo 世界与仿真启动
├── cleaning_robot_bringup/       # SLAM、Nav2、地图、脚本和参数
└── cleaning_robot_coverage/      # 覆盖规划与清扫任务原型
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

启动覆盖规划原型：

```bash
ros2 launch cleaning_robot_coverage coverage.launch.py
```

## 关键入口

- `src/cleaning_robot_description/urdf/robot.urdf.xacro`：机器人模型总入口
- `src/cleaning_robot_simulation/launch/sim.launch.py`：Gazebo 仿真入口
- `src/cleaning_robot_bringup/launch/slam.launch.py`：SLAM 建图入口
- `src/cleaning_robot_bringup/launch/nav.launch.py`：已知地图导航入口
- `src/cleaning_robot_bringup/scripts/auto_explore_and_save.py`：自动探索与存图逻辑
- `src/cleaning_robot_coverage/cleaning_robot_coverage/coverage_planner.py`：覆盖规划原型


## 当前边界

目前仓库更适合用来学习和扩展 ROS 2 机器人系统骨架，而不是直接当作“完整扫地机器人产品”使用。以下能力仍需要继续实现或加强：

- 正式清扫任务状态机
- 覆盖路径到导航目标的执行闭环
- 更复杂环境和更完整的回归测试
- 异常恢复、回充、任务调度等完整任务系统

## 后续提交

日常更新可以直接使用：

```bash
git status
git add .
git commit -m "Describe your change"
git push
```
