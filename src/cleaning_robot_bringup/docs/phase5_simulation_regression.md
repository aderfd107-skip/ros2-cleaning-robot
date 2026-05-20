# 阶段 5：复杂仿真环境与回归测试

本文档只覆盖阶段 5 的仿真场景、启动入口和轻量回归流程。
不修改覆盖规划、清扫任务管理或其他算法核心逻辑。

## 准备

```bash
cd ~/cleaning_robot/cleaning_robot_ws
mkdir -p /tmp/phase5_maps
colcon build --symlink-install
source install/setup.bash
```

`cleaning_robot_simulation` 现在支持两种 world 选择方式：

- `world_name:=<preset>`：使用内置场景预设
- `world:=/abs/path/to/custom.world`：直接指定自定义 world，优先级高于 `world_name`

内置预设：

- `empty_room`
- `single_obstacle`
- `multi_obstacle`
- `narrow_passage`
- `multi_room`

兼容旧场景名：

- `room`

## 轻量 smoke 回归

如果只想先确认 5 个 Gazebo 场景都能被启动，可运行：

```bash
ros2 run cleaning_robot_bringup phase5_world_smoke.sh 12
```

判定通过：

- 5 个 world 都能进入 Gazebo 启动流程
- 脚本最终输出 `All world smoke launches completed.`
- 过程中没有 world 文件缺失、launch 参数解析失败或机器人生成失败

说明：

- 这是启动级 smoke test，不替代功能验收
- 若命令因 `timeout` 结束，属于脚本预期行为

## 场景回归矩阵

### 1. empty_room

用途：

- 验证最简单无障碍基线
- 观察 SLAM、导航和覆盖路径是否在开阔区域内稳定
- 作为后续复杂场景的对照组

启动命令：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py \
  world_name:=empty_room \
  map_output:=/tmp/phase5_maps/empty_room
```

建议观察：

- 机器人起步后是否能快速扫出完整矩形边界
- `/map` 轮廓是否平直，没有明显缺口或大面积漂移
- RViz 中激光点云是否与四周墙体对齐

通过标准：

- 外墙四边都被稳定建图出来
- 机器人在开阔区域内连续运动，不长时间原地打转
- 在超时前成功保存 `/tmp/phase5_maps/empty_room.yaml` 和 `.pgm`

### 2. single_obstacle

用途：

- 作为默认回归基线
- 验证单个静态障碍物对建图、定位和路径规划的影响
- 对齐现有 `room_map.yaml` 的已知地图导航流程

启动命令：

```bash
ros2 launch cleaning_robot_bringup nav.launch.py \
  world_name:=single_obstacle \
  map:=/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/maps/room_map.yaml
```

建议观察：

- `2D Pose Estimate` 后 AMCL 是否快速贴合真实位置
- 多次发送 `Nav2 Goal` 时，全局路径是否绕开右上区域方块障碍物
- 接近障碍物边缘时机器人是否保持可见安全距离

通过标准：

- 连续 3 个可达目标点都能到达
- 路径不穿过静态方块障碍物
- 机器人不与墙体或方块发生碰撞

### 3. multi_obstacle

用途：

- 验证多障碍密集环境下的建图完整性
- 检查自动探索是否会遗漏被多个障碍分割出的自由区域
- 为后续覆盖回归提供更复杂的已知地图

启动命令：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py \
  world_name:=multi_obstacle \
  map_output:=/tmp/phase5_maps/multi_obstacle
```

建议观察：

- 中央圆柱和四周箱体是否都能在地图上成形
- 机器人穿行障碍之间时 `/scan` 与地图是否持续对齐
- 没有出现明显穿障、卡死或局部区域长期探索不到

通过标准：

- 地图中能辨认出中心障碍和四周主要障碍布局
- 机器人能在多个障碍间持续移动至少一个完整探索周期
- 在超时前成功保存 `/tmp/phase5_maps/multi_obstacle.yaml` 和 `.pgm`

### 4. narrow_passage

用途：

- 验证狭窄通道对定位和局部避障的压力
- 观察机器人通过窄通道时是否出现明显抖动、贴墙或反复退让
- 检查建图是否会把通道错误封死

启动命令：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py \
  world_name:=narrow_passage \
  map_output:=/tmp/phase5_maps/narrow_passage
```

说明：

- 如果你想手动指定出生点，传入 `spawn_x`、`spawn_y`、`spawn_yaw`。

建议观察：

- 地图中央通道是否被正确保留为可通行区域
- 机器人穿越狭窄区域时是否仍能保持连续前进
- 如果短暂停顿，是否能恢复，而不是无限原地旋转

通过标准：

- 窄通道在保存地图中保持连通
- 机器人至少成功穿越一次中央狭窄区域
- 全程没有与两侧障碍发生接触碰撞

### 5. multi_room

用途：

- 验证多房间连通结构中的探索覆盖能力
- 检查门洞、拐角和分区对 SLAM 拓扑的影响
- 为多区域清扫任务提供更接近真实家的回归场景

启动命令：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py \
  world_name:=multi_room \
  map_output:=/tmp/phase5_maps/multi_room
```

建议观察：

- 机器人是否会穿过多个门洞进入不同子房间
- 地图是否能形成中心通道和上下左右分区结构
- 进入分区后能否再回到中心区域继续探索

通过标准：

- 至少 3 个独立子区域被成功探索并反映到地图中
- 门洞位置在地图中保持连通，而不是整段被误判成墙
- 在超时前成功保存 `/tmp/phase5_maps/multi_room.yaml` 和 `.pgm`

## 已知地图回归入口

当某个场景通过 SLAM 存图后，可继续用同名地图做导航或清扫回归。

导航回归：

```bash
ros2 launch cleaning_robot_bringup nav.launch.py \
  world_name:=multi_room \
  map:=/tmp/phase5_maps/multi_room.yaml
```

清扫回归：

```bash
ros2 launch cleaning_robot_bringup cleaning.launch.py \
  world_name:=multi_room \
  map:=/tmp/phase5_maps/multi_room.yaml
```

推荐检查项：

- `nav.launch.py`：目标点可达性、绕障效果、AMCL 对齐稳定性
- `cleaning.launch.py`：覆盖路径是否落在已知自由区域，任务是否持续推进而不是早停
