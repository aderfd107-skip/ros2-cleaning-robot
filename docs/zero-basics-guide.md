# ROS 2 扫地机器人项目零基础说明书

这份 README 是给“刚学到 Gazebo 和 RViz，还没有真正做过导航、建图、自动清扫”的你准备的。

目标不是只告诉你“怎么运行”，而是让你真的明白：

- 这个项目现在做到了什么
- 每个包分别负责什么
- ROS 2 里的节点、话题、TF、地图在这里是怎么配合的
- 为什么 `slam.launch.py` 和 `nav.launch.py` 能把整个系统串起来
- 这个项目还缺什么，为什么说它“还未完成”
- 如果你想自己继续写“阶段 4”，应该从哪里下手改代码

## 1. 先用一句话说清这个项目

这是一个基于 ROS 2 Humble 的扫地机器人仿真项目。

它目前已经具备这些基础能力：

- 机器人模型描述
- Gazebo 单房间仿真
- 激光雷达和 IMU 传感器仿真
- 基于 SLAM 的建图
- 基于已知地图的 Nav2 导航
- 一个简单的“自动探索并存图”脚本
- 一个“覆盖路径规划”原型节点

但它还没有完全变成一个真正完整的“扫地机器人系统”。

它还缺少这些关键能力：

- 真正的清扫任务状态机
- 把覆盖路径逐个变成导航目标并执行
- 更复杂环境下的回归验证
- 更完整的异常恢复和任务调度

所以，当前项目更准确的定位是：

**一个已经搭好基础骨架、但还没有做完最后几层功能的扫地机器人教学项目。**

## 2. 你现在最需要建立的总图

你可以把整个项目理解成 4 层。

### 第 1 层：机器人长什么样

对应包：

- `src/cleaning_robot_description`

这一层解决的问题是：

- 机器人有哪些部件
- 这些部件之间怎么连接
- 雷达和 IMU 装在哪里
- 机器人在 TF 树里有哪些坐标系

### 第 2 层：机器人在哪个世界里动

对应包：

- `src/cleaning_robot_simulation`

这一层解决的问题是：

- Gazebo 里有什么房间、墙、障碍物
- 机器人怎么被放进世界里
- `/cmd_vel` 为什么能让机器人动起来

### 第 3 层：机器人怎么建图和导航

对应包：

- `src/cleaning_robot_bringup`

这一层解决的问题是：

- 怎么启动 SLAM
- 怎么保存地图
- 怎么启动 Nav2
- 怎么在已知地图上做定位和路径规划

### 第 4 层：机器人怎么“像扫地机器人那样工作”

对应包：

- `src/cleaning_robot_coverage`

这一层本来应该解决的问题是：

- 怎么规划覆盖清扫路径
- 怎么按照清扫顺序把区域扫完
- 怎么处理任务流程

但目前这里只完成了“生成覆盖路径”，还没有完成“执行整个清扫任务”。

## 3. 仓库结构总览

```text
cleaning_robot_ws/
├── README.md                          <- 你现在看的这份总说明
├── PROJECT_STATUS.md                  <- 旧的项目状态说明
└── src/
    ├── cleaning_robot_description/    <- 机器人本体模型
    ├── cleaning_robot_simulation/     <- Gazebo 仿真世界
    ├── cleaning_robot_bringup/        <- 系统启动、SLAM、Nav2、自动探索
    └── cleaning_robot_coverage/       <- 覆盖路径规划原型
```

如果你以后忘了“某个功能在哪”，就按这个思路找：

- 机器人长什么样：`description`
- 世界长什么样：`simulation`
- 整套系统怎么启动：`bringup`
- 清扫路径怎么生成：`coverage`

## 4. 先补上几个 ROS 2 基础概念

如果你是纯新手，这几个词一定要先看懂。

### 4.1 Node 节点

ROS 2 里一个独立运行的程序，就叫一个节点。

比如这个项目里：

- `robot_state_publisher` 是节点
- `slam_toolbox` 是节点
- `amcl` 是节点
- `coverage_planner` 是节点
- `auto_explore_and_save.py` 也是节点

你可以把节点理解成“系统里的一个功能模块进程”。

### 4.2 Topic 话题

Topic 是节点之间传数据的“广播频道”。

比如：

- `/scan`：激光雷达数据
- `/odom`：里程计
- `/cmd_vel`：速度控制命令
- `/map`：地图

一个节点发布，另一个节点订阅。

### 4.3 TF 坐标变换

TF 是 ROS 里非常核心的一套坐标系关系。

在这个项目里最重要的一条 TF 链是：

```text
map -> odom -> base_footprint -> base_link -> lidar_link
                                      -> imu_link
```

它表示：

- `map`：全局地图坐标系
- `odom`：里程计坐标系
- `base_footprint`：机器人底盘在地面上的基准点
- `base_link`：机器人主体
- `lidar_link`：激光雷达
- `imu_link`：IMU

如果这条链不通，SLAM、定位、导航都会出问题。

### 4.4 Launch 文件

`.launch.py` 文件的作用不是“写算法”，而是“把很多节点一起启动起来，并把参数接好”。

所以你以后看到 launch 文件，要先问：

- 它启动了哪些节点
- 给这些节点传了哪些参数
- 它 include 了哪个别的 launch 文件

### 4.5 Xacro / URDF

- URDF：描述机器人结构的 XML
- Xacro：带变量和宏的 URDF，写起来更方便

这个项目不是直接手写一大坨 URDF，而是把模型拆成多个 xacro 文件，再在总入口里组合。

### 4.6 Occupancy Grid 占据栅格地图

SLAM 和导航里常见的地图，本质上就是：

- 白色：空闲
- 黑色：障碍物
- 灰色：未知

在这个项目里，地图文件保存在：

- `src/cleaning_robot_bringup/maps/room_map.yaml`
- `src/cleaning_robot_bringup/maps/room_map.pgm`

## 5. 这个项目现在到底完成到哪了

如果只看“能不能跑”，你可能会觉得已经差不多了。

但如果从“完整扫地机器人系统”来判断，当前状态更适合这样理解：

### 已完成

- 机器人模型定义
- Gazebo 房间仿真
- 激光雷达和 IMU 仿真
- 基础里程计和速度控制
- SLAM 建图
- 地图保存
- Nav2 已知地图导航
- 自动探索原型
- 覆盖路径规划原型

### 半完成

- 自动探索

原因是它已经能动、能避障、能存图，但仍然是“规则驱动的简单脚本”，不是成熟的探索系统。

- 覆盖清扫

原因是它已经能生成覆盖路径，但还没有把路径送给 Nav2 去真正执行，也没有清扫任务状态机。

### 还没完成

- 清扫任务总状态机
- 覆盖路径执行器
- 复杂环境验证
- 回归测试流程
- 更完整的异常恢复机制

你以后做“阶段 4”，主要就是围绕这些未完成部分继续扩展。

## 6. 四个包逐个讲清楚

---

## 6.1 `cleaning_robot_description`：机器人本体模型

这个包就是“机器人说明书”。

关键文件：

- `src/cleaning_robot_description/urdf/robot.urdf.xacro`
- `src/cleaning_robot_description/urdf/base.xacro`
- `src/cleaning_robot_description/urdf/sensors.xacro`
- `src/cleaning_robot_description/urdf/materials.xacro`
- `src/cleaning_robot_description/launch/display.launch.py`

### 6.1.1 `robot.urdf.xacro` 是总入口

这个文件做了两件大事：

1. 把 `materials.xacro`、`base.xacro`、`sensors.xacro` 包含进来
2. 给 Gazebo 挂上平面运动插件 `libgazebo_ros_planar_move.so`

这个插件非常关键，因为它让机器人具备了：

- 订阅 `cmd_vel`
- 发布 `odom`
- 发布 `odom -> base_footprint` 的 TF

也就是说，机器人之所以能“接收速度命令然后动起来”，很大程度就是因为这个插件。

### 6.1.2 `base.xacro` 描述了底盘和轮子

你可以把它理解成“纯机械结构部分”。

里面定义了：

- `base_footprint`
- `base_link`
- 左轮和右轮
- 前后两个脚轮

几个重要点：

- `base_footprint` 到 `base_link` 是固定连接
- 左右轮是 `continuous` 关节，表示可以一直转
- 前后脚轮是固定连接，这里更像简化支撑结构
- 文件里还给底盘、轮子、脚轮设置了质量、惯量、摩擦参数

这说明这个项目不是只画了个外形，它至少把基础物理属性也补上了。

### 6.1.3 `sensors.xacro` 描述传感器

这个文件定义了两个核心传感器：

- LiDAR
- IMU

LiDAR 相关重点：

- link 名字：`lidar_link`
- 安装在 `base_link` 上方
- 使用 `libgazebo_ros_ray_sensor.so`
- 输出 `/scan`

IMU 相关重点：

- link 名字：`imu_link`
- 使用 `libgazebo_ros_imu_sensor.so`
- 输出 `/imu`

所以你以后如果看到：

- SLAM 为什么能工作
- Nav2 为什么能感知障碍物

根源都可以追溯到这里的雷达输出 `/scan`。

### 6.1.4 `display.launch.py` 做什么

这个 launch 文件很适合新手单独练习模型理解。

它会启动：

- `robot_state_publisher`
- `joint_state_publisher`
- `rviz2`

用途是：

- 不开 Gazebo，只在 RViz 看机器人模型和 TF

如果你以后改了 URDF，却发现导航全乱了，先回到这个 launch 验证模型是很好的习惯。

---

## 6.2 `cleaning_robot_simulation`：Gazebo 仿真世界

关键文件：

- `src/cleaning_robot_simulation/worlds/room.world`
- `src/cleaning_robot_simulation/launch/sim.launch.py`

### 6.2.1 `room.world` 是一个很简单的单房间环境

这个世界里有：

- 东南西北四面墙
- 一个静态方块障碍物
- 地面和太阳光源

这个环境非常简单，但它有教学上的好处：

- 地图容易建
- 导航问题容易定位
- 你不会一开始就被复杂场景干扰

### 6.2.2 `sim.launch.py` 是怎么启动仿真的

这个 launch 文件做了这些事：

1. 找到机器人 xacro
2. 找到 Gazebo 世界文件
3. 启动 Gazebo
4. 启动 `robot_state_publisher`
5. 启动 `joint_state_publisher`
6. 延迟一会儿后，用 `spawn_entity.py` 把机器人生成到 Gazebo 里

你可以特别注意这几个参数：

- `world`
- `entity_name`
- `spawn_x`
- `spawn_y`
- `spawn_z`
- `spawn_yaw`
- `spawn_delay`

以后你要做更复杂环境，最先会改的通常就是：

- 世界文件
- 出生点参数

### 6.2.3 为什么这里要有 `robot_state_publisher`

很多新手会以为“Gazebo 里有机器人了，就不需要 `robot_state_publisher` 了”。

其实不是。

Gazebo 负责物理仿真，但 ROS 侧很多节点仍然需要 TF 和机器人描述。

`robot_state_publisher` 的作用就是根据 URDF 把：

- `base_footprint -> base_link`
- `base_link -> lidar_link`
- `base_link -> imu_link`

这些 TF 发出来。

---

## 6.3 `cleaning_robot_bringup`：把整个系统串起来

这是目前项目里最重要的包。

关键目录：

- `launch/`
- `config/`
- `maps/`
- `scripts/`
- `docs/`

这个包不是“写机器人算法最多”的包，但它是“把所有东西接起来”的核心包。

### 6.3.1 `slam.launch.py`：建图流程总入口

这个 launch 文件启动的是：

1. `cleaning_robot_simulation/launch/sim.launch.py`
2. `slam_toolbox`
3. `nav2_map_server` 里的 `map_saver_server`
4. `nav2_lifecycle_manager`
5. `auto_explore_and_save.py`
6. RViz

也就是说，一条命令：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py
```

背后其实是在做：

- 启动仿真
- 让机器人开始跑
- 用雷达边跑边建图
- 时间到后自动保存地图

### 6.3.2 `slam_toolbox.yaml`：SLAM 配置

这个文件非常短，但已经足够搭起基础建图。

里面最关键的是：

- `odom_frame: odom`
- `map_frame: map`
- `base_frame: base_footprint`
- `scan_topic: /scan`
- `mode: mapping`
- `resolution: 0.05`

你可以这样理解：

- 输入：`/scan` 和 `odom`
- 输出：`/map` 和 `map -> odom`

也就是说，SLAM 把“雷达看到什么”和“机器人走了多远”结合起来，拼出地图。

### 6.3.3 `auto_explore_and_save.py`：自动探索脚本

这是当前项目里最值得你认真读的 Python 文件之一。

文件位置：

- `src/cleaning_robot_bringup/scripts/auto_explore_and_save.py`

它做的事情可以概括成：

- 订阅 `/scan`
- 订阅 `/odom`
- 发布 `/cmd_vel`
- 到时间后调用 `/map_saver/save_map`

但是它不只是“往前走直到撞墙”那么简单。

#### 它内部已经做了这些逻辑

- 前方扇区距离判断
- 减速区判断
- 避障区判断
- 急停区判断
- 左右转向选择
- 随机游走转向
- 卡住检测
- 倒车 + 转向恢复
- 超时后自动存图
- 存图失败后的重试

#### 你可以把它理解成一个简单状态机

虽然项目里还没有正式的大状态机框架，但这个脚本内部已经有“状态”的味道了。

它会在这些运动状态之间切换：

- `WAIT_FOR_SCAN`
- `CRUISE`
- `SLOWDOWN`
- `AVOID`
- `EMERGENCY_STOP`
- `TURNING`
- `WANDER`
- `SAVE_MAP`
- `SAVE_RETRY_WAIT`
- `RECOVERY_STOP`
- `RECOVERY_REVERSE`
- `RECOVERY_TURN`
- `RECOVERY_HOLD`
- `RECOVERY_ABORT`

这对你以后做阶段 3 或阶段 4 很重要，因为你已经有一个可以继续抽象成正式状态机的雏形了。

#### 它是怎么根据激光雷达做决策的

脚本会把雷达切成几个角度区域：

- 正前方
- 前左
- 前右
- 左侧
- 右侧

然后分别取这些扇区里的最小距离。

再根据距离阈值决定：

- 很近：急停并转向
- 比较近：避障转向
- 有点近：减速前进
- 很空：正常巡航

所以它本质上是一个**基于规则的反应式避障脚本**。

这也正是它的优点和局限：

- 优点：简单，容易看懂，容易调参数
- 局限：不聪明，不保证探索最优，也不保证复杂环境鲁棒性

#### 卡住恢复是怎么做的

脚本不仅看“命令发了什么”，还看“机器人实际上有没有动”。

如果满足以下特征，它会认为可能卡住：

- 已经持续发出前进或转向命令
- 但几秒内位置变化很小
- 或朝向变化也很小

一旦卡住，就会进入恢复流程：

1. 先停
2. 倒车
3. 再朝更空的一侧转向
4. 稍微停一下
5. 重新回到探索

如果连续恢复多次都失败，就停止探索并尝试存图。

这是一种很典型的“工程上先做个能用的版本”的思路。

#### `auto_explore.yaml` 是参数集中地

如果你想练习“调行为”，最适合改的文件之一就是：

- `src/cleaning_robot_bringup/config/auto_explore.yaml`

里面有这些参数：

- 速度大小
- 减速距离
- 避障距离
- 急停距离
- 各个扇区角度
- 自动转向时间
- 卡住检测阈值
- 恢复动作时长

这是新手最适合入门调参的地方，因为：

- 不容易把系统结构改坏
- 改完马上就能在仿真里看到效果

### 6.3.4 `maps/`：地图产物目录

当前默认地图在：

- `src/cleaning_robot_bringup/maps/room_map.yaml`
- `src/cleaning_robot_bringup/maps/room_map.pgm`

`room_map.yaml` 不是地图本体，它更像“地图说明文件”。

里面记录了：

- 实际图片文件名
- 分辨率
- 原点
- 阈值

真正像素地图在 `.pgm` 文件里。

### 6.3.5 `nav.launch.py`：已知地图导航入口

这个 launch 文件做的是另一条链路：

1. 启动仿真
2. 启动 Nav2 官方 bringup
3. 加载默认地图 `room_map.yaml`
4. 启动 RViz

所以这个流程和 `slam.launch.py` 的区别是：

- `slam.launch.py` 是边走边建图
- `nav.launch.py` 是地图已经有了，直接导航

### 6.3.6 `nav2_params.yaml`：导航系统参数

这个文件很重要，但第一次读的时候容易被吓到。

你不用一次全懂。先把它拆成 6 块看：

#### 1. `amcl`

作用：

- 已知地图定位

你可以理解成：

- 地图已经有了
- 雷达也有了
- AMCL 用雷达和里程计估计“机器人现在在地图上的哪里”

#### 2. `planner_server`

作用：

- 生成全局路径

当前用的是：

- `NavfnPlanner`

它负责从起点到目标点规划一条大路线。

#### 3. `controller_server`

作用：

- 跟踪路径，把路径变成速度命令

当前用的是：

- `DWBLocalPlanner`

它负责“路径已经有了，现在每一小步该怎么走”。

#### 4. `bt_navigator`

作用：

- 管理导航行为树

你可以先粗略理解成：

- 它像一个流程控制器
- 负责调用“规划、跟踪、恢复”等动作

#### 5. `local_costmap`

作用：

- 管机器人附近的局部障碍物地图

它主要依赖：

- `/scan`

所以机器人是否能在局部避障，很大程度看这里。

#### 6. `global_costmap`

作用：

- 管整个地图范围内的全局代价地图

它结合：

- 静态地图
- 雷达障碍物
- 膨胀层

#### 新手最值得先关注的参数

第一次调 Nav2，不要试图全部看懂。优先关注这些：

- `robot_radius`
- `inflation_radius`
- `max_vel_x`
- `max_vel_theta`
- `xy_goal_tolerance`
- `yaw_goal_tolerance`

因为这些参数最直接影响：

- 会不会撞墙
- 会不会过于保守
- 会不会走得太慢
- 到点后会不会抖

### 6.3.7 `docs/nav2_known_map_validation.md`

这是一个很实用的小文档。

它告诉你：

- 启动导航后在 RViz 里怎么操作
- 如何发送 `2D Pose Estimate`
- 如何发送 `Nav2 Goal`
- 导航通过的验收标准是什么

这个文档特别适合你在“第一次真正理解 Nav2 工作流程”时配合使用。

---

## 6.4 `cleaning_robot_coverage`：覆盖路径规划原型

这个包很重要，因为它代表项目正在往“真正清扫”迈进。

关键文件：

- `src/cleaning_robot_coverage/cleaning_robot_coverage/coverage_planner.py`
- `src/cleaning_robot_coverage/launch/coverage.launch.py`
- `src/cleaning_robot_coverage/README.md`

### 6.4.1 这个包现在到底做了什么

它现在会：

- 读取已有地图，或者订阅 `/map`
- 生成一条覆盖路径
- 发布 `/coverage_path`
- 发布 `/coverage_waypoints`

它**不会**做这些事：

- 不会自己调用 Nav2
- 不会逐点导航
- 不会管理清扫任务状态

所以它现在是“路径规划器”，还不是“完整清扫执行器”。

### 6.4.2 它的算法思路是什么

这个规划器用的是一种很经典、也很适合扫地机入门的思路：

- Boustrophedon 风格来回扫描

你可以把它想象成“割草机路径”。

大致流程是：

1. 读取占据栅格地图
2. 把障碍物和未知区域视为不可走
3. 对障碍物做膨胀，留安全边距
4. 按固定间距切出一条条水平扫描线
5. 找出每一条扫描线上连续可走的线段
6. 一行从左到右，下一行从右到左，来回覆盖
7. 如果两段之间不连通，就用 A* 找连接路径
8. 最后把过密的栅格点简化成更干净的 waypoints

### 6.4.3 为什么这个包对你写阶段 4 很重要

因为它已经提供了“清扫区域路径”的核心原型。

你后面如果继续做项目，很自然的下一步就是：

- 把 `/coverage_waypoints` 变成一串 Nav2 目标点
- 加一个任务状态机控制执行顺序
- 失败时重新规划或跳过

也就是说，这个包虽然还不完整，但它已经给阶段 4 提供了一个很好的出发点。

## 7. 系统运行链路，用大白话串起来

如果你想真正懂项目，最好把完整数据流背下来。

### 7.1 仅看机器人模型

```text
display.launch.py
-> 读取 xacro
-> robot_state_publisher 发布 TF
-> RViz 显示机器人
```

### 7.2 启动 Gazebo 仿真

```text
sim.launch.py
-> 读取 xacro
-> Gazebo 启动世界
-> robot_state_publisher 发布 TF
-> spawn_entity 把机器人生成到 Gazebo
-> Gazebo 插件接收 /cmd_vel，发布 /odom、/scan、/imu
```

### 7.3 启动 SLAM 建图

```text
slam.launch.py
-> include sim.launch.py
-> slam_toolbox 订阅 /scan 和 odom
-> 发布 /map 和 map->odom
-> auto_explore_and_save.py 发布 /cmd_vel 让机器人移动
-> map_saver_server 提供存图服务
-> 超时后调用存图服务保存 room_map
```

### 7.4 启动已知地图导航

```text
nav.launch.py
-> include sim.launch.py
-> map_server 读取 room_map.yaml
-> AMCL 用 /scan + /odom 在地图上定位
-> planner_server 规划全局路径
-> controller_server 生成速度控制
-> Gazebo 接收 /cmd_vel 后移动机器人
```

### 7.5 启动覆盖路径规划

```text
coverage.launch.py
-> coverage_planner 读取地图
-> 生成 coverage_path 和 coverage_waypoints
-> RViz 可视化查看路径
```

### 7.6 你以后自己加 Python 节点时，为什么有时能运行，有时不能运行

这是新手后面非常容易卡住的一点。

这个项目里其实用了两种不同的“让 Python 节点变成 ROS 可执行文件”的方式。

#### 方式 1：像 `cleaning_robot_bringup` 一样，直接安装脚本

对应文件：

- `src/cleaning_robot_bringup/CMakeLists.txt`

这里用了：

```cmake
install(PROGRAMS
  scripts/auto_explore_and_save.py
  DESTINATION lib/${PROJECT_NAME}
)
```

意思是：

- 这个 Python 文件本身就会被当成可执行程序安装
- 所以 launch 文件里可以直接写可执行名 `auto_explore_and_save.py`

#### 方式 2：像 `cleaning_robot_coverage` 一样，用 `setup.py` 的 `console_scripts`

对应文件：

- `src/cleaning_robot_coverage/setup.py`

这里用了：

```python
entry_points={
    "console_scripts": [
        "coverage_planner = cleaning_robot_coverage.coverage_planner:main",
    ],
}
```

意思是：

- `ros2 run cleaning_robot_coverage coverage_planner`
- 最后会去执行 `coverage_planner.py` 里的 `main()`

#### 这两种方式你以后怎么选

如果你后面写阶段 4 的新节点：

- 想继续沿用 `bringup` 的风格：就在 `scripts/` 下新建 `.py`，然后在 `CMakeLists.txt` 里加到 `install(PROGRAMS ...)`
- 想继续沿用 `coverage` 的风格：就在 Python 包目录里加模块，然后在 `setup.py` 的 `console_scripts` 里注册

#### 你最需要记住的一句话

**代码文件写出来，不等于 ROS 2 就能找到它。**

你必须同时处理：

- 文件放在哪里
- 它有没有执行权限
- `CMakeLists.txt` 或 `setup.py` 有没有把它注册出去
- 改完后有没有重新 `colcon build` 并重新 `source install/setup.bash`

## 8. 你最应该亲自读懂的 10 个文件

如果你现在想从“小白”进步到“能自己改代码”，优先读这 10 个文件。

### 第 1 组：先搞懂系统骨架

1. `src/cleaning_robot_description/urdf/robot.urdf.xacro`
2. `src/cleaning_robot_simulation/launch/sim.launch.py`
3. `src/cleaning_robot_bringup/launch/slam.launch.py`
4. `src/cleaning_robot_bringup/launch/nav.launch.py`

目标：

- 搞懂系统是怎么被启动和串联起来的

### 第 2 组：再搞懂行为逻辑

5. `src/cleaning_robot_bringup/scripts/auto_explore_and_save.py`
6. `src/cleaning_robot_bringup/config/auto_explore.yaml`
7. `src/cleaning_robot_bringup/config/nav2_params.yaml`

目标：

- 搞懂机器人为什么这么走
- 搞懂导航参数是怎么影响行为的

### 第 3 组：最后搞懂“未完成部分”

8. `src/cleaning_robot_coverage/cleaning_robot_coverage/coverage_planner.py`
9. `src/cleaning_robot_coverage/launch/coverage.launch.py`
10. `PROJECT_STATUS.md`

目标：

- 搞懂项目下一步准备往哪里扩展

## 9. 你应该怎样一步一步学这个项目

我建议你不要直接冲着“写阶段 4”去。

更稳的学习顺序是：

### 第一步：先只看模型和 TF

运行：

```bash
ros2 launch cleaning_robot_description display.launch.py
```

你要搞懂：

- 机器人有哪些 link
- 哪些是固定连接，哪些是轮子
- LiDAR 在哪里
- `base_footprint` 和 `base_link` 的区别

### 第二步：只看 Gazebo 里机器人怎么动

运行：

```bash
ros2 launch cleaning_robot_simulation sim.launch.py
```

你要搞懂：

- 机器人是怎么被 spawn 进去的
- Gazebo 插件为什么能把 `/cmd_vel` 变成运动
- `/odom` 和 `/scan` 是哪里来的

### 第三步：看 SLAM 是怎么工作的

运行：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py
```

你要观察：

- 机器人怎么自动跑
- RViz 里地图怎么一点点长出来
- 为什么 TF 一旦错了地图就会漂

### 第四步：看已知地图导航

运行：

```bash
ros2 launch cleaning_robot_bringup nav.launch.py
```

你要搞懂：

- 为什么要先 `2D Pose Estimate`
- `Nav2 Goal` 发出去后，哪些节点开始工作
- 全局路径和局部控制分别是谁负责

### 第五步：看覆盖路径规划

运行：

```bash
ros2 launch cleaning_robot_coverage coverage.launch.py
```

你要搞懂：

- 为什么路径会一行一行来回扫
- 为什么要对障碍物做膨胀
- 为什么生成路径不等于“已经会扫地”

## 10. 如果你要自己改代码，先从哪里练手

新手不要一上来改大结构。

最好的练手机会有 4 类。

### 练手 1：改机器人外形和尺寸

改这些文件：

- `src/cleaning_robot_description/urdf/base.xacro`
- `src/cleaning_robot_description/urdf/sensors.xacro`

可以尝试：

- 改底盘半径
- 改轮子位置
- 改雷达高度

你会学到：

- 结构参数如何影响碰撞和导航
- 为什么 `robot_radius` 也要一起调整

### 练手 2：改自动探索行为

改这些文件：

- `src/cleaning_robot_bringup/config/auto_explore.yaml`
- `src/cleaning_robot_bringup/scripts/auto_explore_and_save.py`

可以尝试：

- 提高前进速度
- 调大急停距离
- 改转向策略
- 改卡住恢复时长

你会学到：

- 规则式机器人控制怎么写
- 参数和行为的关系

### 练手 3：改导航参数

改这个文件：

- `src/cleaning_robot_bringup/config/nav2_params.yaml`

可以尝试：

- 调大 `inflation_radius`
- 调小 `max_vel_x`
- 调整目标容差

你会学到：

- 为什么同一个地图，参数不同，机器人表现差很多

### 练手 4：改世界环境

改这个文件：

- `src/cleaning_robot_simulation/worlds/room.world`

可以尝试：

- 多加几个障碍物
- 改障碍物位置
- 做一个更狭窄的通道

你会学到：

- 为什么简单算法在复杂环境下容易暴露问题

## 10.5 新手最常用的排查命令

以后你调项目时，不要只盯着 RViz 画面看。

这几条命令很常用：

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic echo /cmd_vel --once
```

你可以这样理解它们的用途：

- `ros2 node list`：现在到底启动了哪些节点
- `ros2 topic list`：系统里现在有哪些数据通道
- `ros2 service list`：有没有存图服务之类的接口
- `ros2 action list`：Nav2 的动作接口有没有起来
- `ros2 topic echo ... --once`：这个 topic 到底有没有数据

一个很好用的排查思路是：

1. 机器人不动，先看有没有 `/cmd_vel`
2. SLAM 不出图，先看 `/scan`、`/odom`、`/tf`
3. Nav2 不工作，先看 AMCL、map server、planner、controller 是否都起来了

## 11. 阶段 4 到底应该理解成什么

现有文档里的阶段编号并不完全统一，所以你不要死记“阶段 2、阶段 3、阶段 4”的名字。

更建议你按“功能成熟度”理解。

### 按代码现状来理解，当前大概是这样

#### 基础骨架已经有了

- 模型
- 仿真
- SLAM
- Nav2

#### 自动行为有了原型

- 自动探索脚本
- 覆盖路径规划

#### 但完整清扫闭环还没做完

- 覆盖路径执行
- 任务状态机
- 复杂环境验证

所以如果你说“我想自己写阶段 4”，我建议你把阶段 4 理解成：

**把当前单房间基础项目，推进到更接近“完整扫地机器人任务系统”的那一步。**

## 12. 如果让你自己做阶段 4，最合理的任务拆分

下面这个拆分非常适合你后续自己接着做。

### 任务 A：扩展复杂环境

目标：

- 不再只在一个简单房间里测试

建议改动：

- 新建更多 `.world` 文件
- 增加多个障碍物
- 增加狭窄通道
- 增加类似家具布局的结构

主要修改位置：

- `src/cleaning_robot_simulation/worlds/`
- `src/cleaning_robot_simulation/launch/sim.launch.py`

### 任务 B：让覆盖路径真正能执行

目标：

- 不只是发布 `/coverage_waypoints`
- 而是把这些 waypoint 依次交给 Nav2 去走

你大概率需要新建一个节点，做这些事：

1. 订阅 `/coverage_waypoints`
2. 调用 Nav2 的导航 action
3. 等待当前目标完成
4. 再发下一个目标
5. 失败时重试、跳过或恢复

推荐新包或新脚本位置：

- 可以放进 `cleaning_robot_coverage`
- 或新建一个 `cleaning_robot_tasks`

### 任务 C：加正式状态机

目标：

- 不再靠多个 launch 文件和单个脚本松散配合
- 而是有统一任务流程

可以设计的状态例如：

- `IDLE`
- `MAPPING`
- `SAVE_MAP`
- `LOCALIZE`
- `COVERAGE_PLAN`
- `COVERAGE_EXECUTE`
- `PAUSE`
- `RECOVERY`
- `DONE`
- `ERROR`

这一步做完后，项目就会从“功能拼起来能跑”，变成“像一个系统”。

### 任务 D：做回归验证

目标：

- 每次改参数或改算法后，能快速验证有没有退化

最简单的做法包括：

- 固定几个世界
- 固定几个起点
- 固定几个导航目标
- 记录是否成功、耗时、是否碰撞

这一步会让你的项目质量提升很多。

## 13. 阶段 4 最值得你改的具体文件

如果你后面真的开始写，我建议优先盯住这些位置。

### 和复杂环境有关

- `src/cleaning_robot_simulation/worlds/room.world`
- 新建更多 world 文件
- `src/cleaning_robot_simulation/launch/sim.launch.py`

### 和任务调度有关

- 新建状态机节点
- 新增 launch 文件统一启动“建图模式 / 导航模式 / 清扫模式”

最可能的新文件方向：

- `src/cleaning_robot_bringup/launch/cleaning.launch.py`
- `src/cleaning_robot_bringup/scripts/mission_manager.py`

### 和覆盖执行有关

- `src/cleaning_robot_coverage/cleaning_robot_coverage/coverage_planner.py`
- 新建 waypoint 执行节点

最可能的新文件方向：

- `src/cleaning_robot_coverage/cleaning_robot_coverage/coverage_executor.py`

### 和参数优化有关

- `src/cleaning_robot_bringup/config/nav2_params.yaml`
- `src/cleaning_robot_bringup/config/auto_explore.yaml`

## 14. 你要能“自己写阶段 4”，至少要真正懂这几件事

请把下面这几件事当成必须掌握，而不是“最好掌握”。

### 1. 你要知道每个 topic 从哪里来，到哪里去

最重要的是：

- `/cmd_vel`
- `/scan`
- `/odom`
- `/map`
- `/coverage_path`
- `/coverage_waypoints`

### 2. 你要知道 TF 链为什么不能断

最关键的是：

- `map -> odom`
- `odom -> base_footprint`
- `base_footprint -> base_link`
- `base_link -> lidar_link`

### 3. 你要知道 launch 文件是在“接线”，不是在“写算法”

以后你改 launch 时，要优先从“这个文件到底启动了谁”去看。

### 4. 你要知道 YAML 多半是在调参，不是在写逻辑

以后你看到：

- `nav2_params.yaml`
- `auto_explore.yaml`
- `slam_toolbox.yaml`

第一反应应该是：

- 这里大概率是参数，不是主算法实现

### 5. 你要知道真正决定行为的 Python 代码在哪

目前最核心的行为代码在：

- `auto_explore_and_save.py`
- `coverage_planner.py`

## 15. 我对这个项目的直观判断

从教学角度看，这个项目其实搭得不错，因为它没有一上来就上非常复杂的框架，而是先把基本链路打通了。

它的优点是：

- 包划分清楚
- 文件数量不算夸张
- 从模型到仿真到导航这条线很完整
- 自动探索和覆盖规划都已经有原型，适合继续扩展

它当前的主要短板是：

- 还没有统一任务状态机
- 覆盖路径还没接到 Nav2 执行
- 阶段编号和部分文档描述略有不一致
- 复杂环境验证还不够

这意味着它非常适合你拿来学习和继续开发，因为：

- 它不是一个空壳
- 但它又没有复杂到让新手完全无从下手

## 16. 给你的最实在建议

如果你的目标是“以后能自己写阶段 4”，那你现在不要追求一下子懂全部 Nav2 内部原理。

你最应该先做到的是：

1. 能说清四个包分别干什么
2. 能说清 `slam.launch.py` 和 `nav.launch.py` 的区别
3. 能说清 `/scan`、`/odom`、`/cmd_vel`、`/map` 的数据流
4. 能自己改 `auto_explore.yaml` 并看懂行为变化
5. 能自己改 `room.world` 加障碍物
6. 能自己读懂 `coverage_planner.py` 的主流程

如果这 6 件事你都做到了，你就已经不是“纯小白”了，而是进入了“可以接着自己扩功能”的阶段。

## 17. 一个适合你的短期学习路线

你接下来 1 到 2 周可以按这个顺序练：

### 第 1 天到第 2 天

- 只读 `description` 和 `simulation`
- 目标：搞懂机器人和世界

### 第 3 天到第 4 天

- 跑 `slam.launch.py`
- 对照 `auto_explore_and_save.py` 看机器人为什么这样动

### 第 5 天到第 6 天

- 跑 `nav.launch.py`
- 改 `nav2_params.yaml` 的几个关键参数看效果

### 第 7 天到第 8 天

- 跑 `coverage.launch.py`
- 对照 `coverage_planner.py` 画出算法流程图

### 第 9 天以后

- 自己尝试做一个最小版本的“coverage waypoint 执行器”

如果你能写出这个执行器，你离“自己做阶段 4”就已经很近了。

## 18. 最后一段总结

这个项目的本质不是“已经完成的扫地机器人”，而是“一个已经完成基础骨架、正在向完整扫地任务系统发展的项目”。

你现在最该做的，不是急着把所有概念一次吃完，而是先把下面这条主线牢牢记住：

```text
机器人模型
-> Gazebo 仿真
-> 传感器输出 /scan /odom
-> SLAM 建图或 AMCL 定位
-> Nav2 规划和控制
-> 更上层的自动探索 / 覆盖规划 / 任务状态机
```

当你真正能把这条主线讲顺的时候，你就已经开始具备“自己改代码、自己续写阶段 4”的能力了。
