# conclusion.md

这份文档是给“现在已经把项目跑起来了，但很多地方还不懂”的你准备的。

目标不是让你一夜之间变成机器人专家，而是帮你做到下面 4 件事：

1. 读完后，至少能从系统层面理解这个项目 80% 左右。
2. 能把这个项目讲给老师、同学、面试官听，而不是只会背命令。
3. 能知道每个包、每个 launch、每个关键脚本到底负责什么。
4. 能知道这个项目现在的上限、问题、缺口，以及下一步怎么继续学。

如果你现在还是“纯小白”也没关系，这份文档会尽量用最朴素的方式来讲。

---

# 1. 先用一句话讲清楚这个项目

这是一个基于 **ROS 2 Humble** 的扫地机器人仿真项目。

它目前已经打通了下面几条主链路：

- 机器人模型描述
- Gazebo 仿真环境
- 激光雷达和 IMU 仿真
- 基于 `slam_toolbox` 的建图
- 基于 `AMCL + Nav2` 的已知地图导航
- 自动探索并自动保存地图
- 弓字形覆盖路径生成
- 覆盖路径执行的基础任务链路

但它还不是一个完全成熟的“商用扫地机器人系统”，原因是：

- 定位稳定性还不够强
- 自动探索结束条件还需要继续调优
- 覆盖路径规划仍然是原型级实现
- 有些世界里仍然会出现提前结束、路径低效、定位跳变等问题

所以更准确地说：

**这是一个已经跑通核心机器人软件链路、并且做了不少工程调试的研究生级仿真项目。**

---

# 2. 你最需要建立的“全局脑图”

把整个项目想成 4 层。

## 第 1 层：机器人本体

负责“机器人长什么样、有什么传感器、有哪些坐标系”。

对应包：

- `src/cleaning_robot_description`

## 第 2 层：仿真世界

负责“机器人被放到哪里跑、世界里有什么墙和障碍、Gazebo 怎么启动”。

对应包：

- `src/cleaning_robot_simulation`

## 第 3 层：系统启动与导航建图

负责“怎么把 Gazebo、SLAM、Nav2、AMCL、RViz、自动探索都启动起来”。

对应包：

- `src/cleaning_robot_bringup`

## 第 4 层：覆盖清扫任务

负责“怎么生成弓字形路径、怎么执行清扫任务、怎么统计覆盖进度”。

对应包：

- `src/cleaning_robot_coverage`

你后面如果忘了某个功能在哪，就按这个规则找：

- 机器人长什么样：`description`
- 世界长什么样：`simulation`
- 系统怎么启动：`bringup`
- 清扫路径和任务：`coverage`

---

# 3. 仓库结构怎么读

项目根目录是：

```text
cleaning_robot_ws/
├── README.md
├── PROJECT_STATUS.md
├── conclusion.md
└── src/
    ├── cleaning_robot_description/
    ├── cleaning_robot_simulation/
    ├── cleaning_robot_bringup/
    └── cleaning_robot_coverage/
```

你可以先只把下面这些目录记住：

```text
src/cleaning_robot_description
src/cleaning_robot_simulation
src/cleaning_robot_bringup
src/cleaning_robot_coverage
```

这 4 个目录就是你这个项目的主战场。

---

# 4. ROS 2 新手必须先懂的概念

如果你这部分不熟，那项目再复杂你也会觉得像黑盒。

## 4.1 节点 Node

ROS 2 里一个独立运行的程序，就叫一个节点。

在你的项目里，下面这些都是节点：

- `robot_state_publisher`
- `joint_state_publisher`
- `slam_toolbox`
- `amcl`
- `planner_server`
- `controller_server`
- `bt_navigator`
- `auto_explore_and_save.py`
- `coverage_planner`
- `cleaning_task_manager`

你可以把一个节点理解成“一个单独的功能模块进程”。

## 4.2 话题 Topic

节点之间要传数据，就会通过 Topic。

常见话题：

- `/scan`：激光雷达数据
- `/odom`：里程计
- `/cmd_vel`：速度指令
- `/map`：地图
- `/amcl_pose`：AMCL 估计的机器人位姿
- `/coverage_waypoints`：覆盖规划生成的路径点

## 4.3 TF 坐标变换

TF 是机器人系统里最重要的东西之一。

在你的项目里最关键的链路是：

```text
map -> odom -> base_footprint -> base_link -> lidar_link
                                      -> imu_link
```

这条链的意思是：

- `map`：全局地图坐标系
- `odom`：里程计坐标系
- `base_footprint`：机器人在地面上的基准点
- `base_link`：机器人主体
- `lidar_link`：激光雷达
- `imu_link`：IMU

如果这条链不通，建图、定位、导航都会出问题。

## 4.4 Launch 文件

`.launch.py` 文件不是算法文件，它是“系统编排文件”。

它负责：

- 启动哪些节点
- 每个节点传什么参数
- 是否 include 其他 launch 文件
- 是否打开 RViz / Gazebo GUI

所以你读 launch 文件时，要先问：

- 它启动了谁？
- 它给谁传了什么参数？
- 它和别的 launch 文件是什么关系？

## 4.5 Xacro / URDF

- URDF：机器人结构描述格式
- Xacro：带宏和变量的 URDF

你的项目不是直接写一大坨 URDF，而是用多个 `.xacro` 文件拼起来的。

## 4.6 Occupancy Grid 栅格地图

SLAM 和 Nav2 里用的地图，本质上是二维栅格：

- 白色：空闲区域
- 黑色：障碍物
- 灰色：未知区域

常见地图文件是：

- `.yaml`：地图元信息
- `.pgm`：实际灰度图

---

# 5. 你的项目到底完成到了什么程度

这是你后面面试时必须讲清楚的。

## 已经完成的部分

- 机器人模型定义
- Gazebo 多 world 仿真
- 激光雷达 / IMU 仿真
- 基础里程计与 `/cmd_vel` 控制
- SLAM 建图
- 自动保存地图
- 已知地图导航
- 覆盖路径生成
- 覆盖任务执行基础链路

## 半完成的部分

- 自动探索
- 多场景调参
- 覆盖路径效率优化
- AMCL 定位稳定性

### 为什么这 4 项叫“半完成”

“半完成”不是说它们没做，也不是说它们做错了。

这里的意思是：

- 主链路已经有了
- 关键功能已经能跑
- 你已经开始围绕真实问题调试
- 但它们还没有达到“稳定、泛化、工程上可放心复用”的成熟程度

换句话说，它们都已经从 `0` 走到了 `1`，但还没有从“能跑”走到“稳、好、通用”。

下面我把这 4 项分别拆开讲。

### 1. 自动探索

#### 现在已经做到什么

- 已经有独立的自动探索脚本 `auto_explore_and_save.py`
- 能读取当前地图并寻找 frontier
- 能基于一定规则选择目标 frontier
- 能把目标交给 Nav2 去执行
- 能在探索结束后自动保存地图
- 你还围绕它做过多次针对性修正，比如：
  - 启动自旋
  - 重复目标冷却
  - 提前结束判定收紧

这说明自动探索不是“没有”，而是已经形成了可运行的任务链路。

#### 为什么它还只能算半完成

因为它虽然能跑，但你已经亲眼遇到过这些真实问题：

- 明显没探索完就提前保存
- 某些场景会反复选择相同或很近的 frontier
- 某些小残留区域补不干净
- 不同 world 下对结束条件的敏感性不同
- 一些场景里 frontier 能识别到，但实际导航过去很困难

这说明当前自动探索的主要问题不是“有没有”，而是：

- 结束判定还不够稳
- 选点策略还不够强
- 对多场景的鲁棒性还不够高

#### 如果以后继续补，下一步该做什么

- 继续细化完成条件，把“未知比例、未知面积、frontier 数量、停滞时间”联合考虑
- 让 frontier 目标选择更聪明，而不是主要依赖距离和简单打分
- 增加对“窄缝附近目标”“高失败率目标”的更强惩罚
- 加入更清晰的探索阶段切换逻辑，例如：
  - 初始观察阶段
  - 主探索阶段
  - 残余补图阶段

#### 你面试时可以怎么说

> 自动探索部分已经打通了 frontier-based exploration 主链路，能够自动建图并保存地图，但在不同场景下仍存在提前结束、重复选点和残余区域补图不稳定的问题，所以我把它定义为“半完成”，因为它已经能用，但还没达到鲁棒、泛化的程度。

### 2. 多场景调参

#### 现在已经做到什么

- 项目已经不只支持单一 world
- 你已经实际跑过多个场景，比如：
  - `narrow_passage`
  - `multi_obstacle`
  - `multi_room`
- 已经引入 world profile 思路
- 不同 world 可以有不同的自动探索参数
- 你已经根据场景差异调过：
  - exploration timeout
  - completion 条件
  - frontier cluster 阈值
  - 目标偏移与 clearance

这说明你不是只会把代码跑在一个 demo 场景里，而是已经开始做“多场景适配”。

#### 为什么它还只能算半完成

因为现在更像是：

- 针对几个已知场景做了经验性适配

而不是：

- 已经抽象出一套更通用、更稳定的调参方法论

你目前还没有做到的包括：

- 清晰总结“哪类场景需要哪类参数变化”
- 建立统一的参数迁移规律
- 用定量指标评估不同参数组合的效果
- 让新 world 只需极少修改就能稳定工作

所以它不是“完成了场景泛化”，而是“已经开始做场景化配置”。

#### 如果以后继续补，下一步该做什么

- 总结场景类型和参数关系，例如：
  - 对称窄通道类
  - 多障碍空旷类
  - 多房间连接类
- 建立一张“场景特征 -> 推荐参数”的经验表
- 对每个 world 固定记录：
  - 建图完成时间
  - 提前结束次数
  - 地图残缺比例
  - 导航异常次数
- 尽量减少“纯手调”感，让参数选择更有规律

#### 你面试时可以怎么说

> 多场景部分已经实现了基于 world profile 的参数区分，不同环境可以加载不同探索策略，但当前仍然偏向经验性调参，还没有形成一套高度泛化的参数体系，所以我认为这部分属于半完成。

### 3. 覆盖路径效率优化

#### 现在已经做到什么

- 覆盖清扫主链路已经有了
- 系统已经能基于已知地图生成弓字形清扫路径
- 你已经意识到原始方案存在明显低效问题
- 也已经开始尝试改进，不再满足于“能扫就行”
- 你已经从“整图扫描线串联”的思路，开始往“区域/cell 优先”的方向思考和修改

这说明你已经从“复现功能”进步到“审视规划质量”了。

#### 为什么它还只能算半完成

因为目前这部分更像是：

- 原型级覆盖规划

而不是：

- 更成熟的区域分解 + 路径优化系统

你自己已经观察到的问题就说明了这一点：

- 同一水平线上被障碍打断后，会左中右来回切换
- 狭窄连接区域穿越次数太多
- 路径虽然能完成，但空驶距离偏大
- 还不够像成熟扫地机器人的区域优先策略

所以目前是：

- 已经发现问题
- 已经开始修正
- 但还没有完成系统级优化

#### 如果以后继续补，下一步该做什么

- 更明确地做区域分解，而不是只靠扫描线段
- 给“跨区域切换”增加代价
- 给“穿过狭窄通道”增加惩罚
- 增加路径总长度、转向次数、通道穿越次数等评价指标
- 最终让 coverage 更接近“先扫完整块，再去下一区域”的策略

#### 你面试时可以怎么说

> 覆盖路径部分已经能生成并执行弓字形路径，但原始方案偏向全局扫描线串联，在复杂障碍场景中会出现较低效的往返切换，所以我把它归类为半完成。后续优化方向是区域分解、切换代价建模和路径重排。

### 4. AMCL 定位稳定性

#### 现在已经做到什么

- 已知地图导航主链路已经打通
- `map_server + AMCL + Nav2` 能正常启动
- 大多数情况下机器人可以在已知地图中完成基本定位和导航
- 说明 AMCL 不是完全不可用，也不是链路没搭起来

这一步非常重要，因为它说明：

- 你已经不仅会建图
- 还把“建图后的定位与导航闭环”跑起来了

#### 为什么它还只能算半完成

因为你已经在真实运行中观察到：

- 某些时刻定位会突然跳
- 对称环境更容易出问题
- 地图质量会影响 AMCL 匹配效果
- 后期恢复行为可能放大误定位
- RViz 中的机器人位姿有时会和 Gazebo 实体解释不一致

这些现象说明：

- AMCL 这部分不是没实现
- 而是鲁棒性还不够强

也就是说，它能工作，但不是“在各种情况下都稳定可靠”。

#### 如果以后继续补，下一步该做什么

- 系统分析地图质量对 AMCL 的影响
- 继续优化对称环境下的定位稳定性
- 检查并对比：
  - 激光数据质量
  - 里程计质量
  - 初始位姿设置
  - 恢复行为触发前后的定位变化
- 建立“什么时候是 `odom` 稳、`map->odom` 跳”的诊断方法

#### 你面试时可以怎么说

> AMCL 已经支持已知地图定位与导航，但在对称环境和任务后期仍可能出现定位跳变，因此我把它定义为半完成。它已经具备功能性，但定位鲁棒性和稳定性还需要继续提升。

### 这 4 项的共同特点

你可以把它们统一理解成一句话：

> 它们都已经不再是“没有做”，而是“已经打通主链路、已经开始围绕真实问题优化，但还没有达到高度稳定、可泛化、可工程复用的成熟状态”。

这就是“半完成”最准确的含义。

## 还不够成熟的部分

- 覆盖路径的全局最优性
- 定位鲁棒性
- 系统化的定量评估
- 真机部署与真机验证
- 更强的异常恢复机制

### 为什么这 5 项叫“还不够成熟”

前面的“半完成”更强调的是：

- 功能已经有了
- 主链路已经打通了
- 但还需要继续优化

而这里的“还不够成熟”强调的是更高一个层次的问题：

- 不是系统能不能运行
- 而是系统是否已经达到更像工程产品、或者更像高质量研究项目的状态

换句话说，这 5 项不是“功能缺个按钮”的问题，而是：

- 优化够不够深入
- 稳定性够不够强
- 证据够不够充分
- 能不能脱离仿真进入真实场景
- 异常发生后是否能更聪明地恢复

所以它们不属于“没做”，而属于“离成熟工程还有明显距离”。

下面我也把这 5 项逐个拆开讲。

### 1. 覆盖路径的全局最优性

#### 现在已经做到什么

- 已经实现了覆盖任务主链路
- 已经能从地图中提取可清扫区域
- 已经能生成弓字形风格的覆盖路径
- 已经能把覆盖路径交给 Nav2 去执行
- 你也已经开始意识到路径效率问题，并尝试做局部优化

这说明“覆盖能跑起来”这件事已经不是问题。

#### 为什么它还不够成熟

因为目前更像是：

- 能生成一条可执行路径

而不是：

- 能生成全局意义上更优的覆盖路径

所谓“全局最优性”不一定要求你真的做到数学上的绝对最优，但至少应该更重视这些问题：

- 总路径长度是否尽量短
- 总转向次数是否尽量少
- 跨区域切换次数是否可控
- 狭窄通道是否被过度反复穿越
- 重复清扫和空驶距离是否被明显压缩

你现在这部分更像“有覆盖规划”，但还不是“有明显优化过的高质量覆盖规划”。

#### 如果以后继续补，下一步该做什么

- 做更明确的区域分解和区域排序
- 引入跨区域切换代价
- 引入狭窄通道穿越惩罚
- 记录总路径长度、转向次数、重复覆盖率等指标
- 尝试让路径更接近“先扫完整块，再切下一块”

#### 你面试时可以怎么说

> 覆盖路径部分已经实现了可执行的弓字形清扫，但目前更偏向原型级路径生成，还没有在全局长度、转向次数、跨区域切换代价等方面做更系统的优化，所以我认为它还不够成熟。

### 2. 定位鲁棒性

#### 现在已经做到什么

- 已知地图导航已经跑通
- `AMCL + Nav2` 基本链路已经工作
- 大多数情况下可以完成基础定位和导航
- 你已经能通过现象区分“机器人真实运动”和“RViz 中位姿解释”的差异

这说明定位链路是建立起来的，而不是完全缺失的。

#### 为什么它还不够成熟

因为“能定位”和“定位鲁棒”是两回事。

鲁棒性强调的是：

- 换一个起始位姿还能不能稳
- 对称环境中会不会轻易跳解
- 地图稍微有误差时是否还能保持稳定
- 恢复行为触发后是否还能重新收敛
- 长时间运行后是否会累积出明显问题

你前面遇到过的后期跳变、对称场景不稳、地图质量影响定位这些现象，说明当前定位还处在“可用但脆弱”的阶段。

#### 如果以后继续补，下一步该做什么

- 针对对称场景做专门稳定性测试
- 系统对比不同地图质量下的 AMCL 表现
- 记录定位跳变出现的时刻、地点、触发条件
- 重点分析恢复行为前后 `map->odom` 的变化
- 必要时引入更强的定位辅助策略或更好的重定位机制

#### 你面试时可以怎么说

> 当前定位链路已经可用，但鲁棒性还不够成熟，尤其在对称环境、地图存在误差或任务后期时，仍可能出现跳变或收敛不稳的问题，因此这部分我归类为“还不够成熟”。

### 3. 系统化的定量评估

#### 现在已经做到什么

- 你已经积累了大量现象级观察
- 你已经能判断哪些场景跑得好、哪些场景有问题
- 你也已经能根据现象反向调参数和改逻辑

这说明你不是盲目运行，而是在做工程判断。

#### 为什么它还不够成熟

因为目前这些判断更多还是：

- 经验性的
- 现象驱动的
- 依赖人工观察的

而成熟一点的项目会希望有更系统的指标体系，比如：

- 建图完成时间
- 建图成功率
- 地图残缺比例
- 导航成功率
- 覆盖完成时间
- 路径总长度
- 恢复行为触发次数
- AMCL 跳变频次
- 不同 world 间的性能对比

没有这些量化数据时，你可以说“我觉得变好了”，但不容易非常有说服力地证明“它确实变好了多少”。

#### 如果以后继续补，下一步该做什么

- 为每个场景建立统一测试流程
- 固定记录关键指标
- 做修改前后对比实验
- 形成一张结果表或实验图
- 在 README 或报告里把结论量化呈现出来

#### 你面试时可以怎么说

> 当前项目已经有比较丰富的现象观察和调试经验，但还缺少系统化的定量评估体系，因此在“证明优化效果”这一点上还不够成熟。后续我会补建图、导航、覆盖和定位稳定性的指标化评估。

### 4. 真机部署与真机验证

#### 现在已经做到什么

- 仿真系统已经比较完整
- 多条核心链路已经打通
- 你已经在仿真中做了很多真实的排障和调试

这已经比很多只停留在单个 demo 的项目强了。

#### 为什么它还不够成熟

因为仿真和真机之间永远有一层鸿沟。

真机会带来很多仿真里不明显的问题，比如：

- 地面打滑
- 电机误差
- 轮子左右不一致
- 激光/IMU 噪声更复杂
- 传感器安装偏差
- 计算资源和通信时延限制
- 真实碰撞和摩擦特性

所以只在仿真里验证过，说明项目已经很适合做研究原型和系统集成练习，但还不能直接说它已经完成了工程落地验证。

#### 如果以后继续补，下一步该做什么

- 先做最基础的真机底盘控制验证
- 再验证传感器 TF 和实际安装是否一致
- 再验证建图和定位链路在真机上能否运行
- 最后再验证导航和覆盖任务
- 过程中记录仿真与真机的差异来源

#### 你面试时可以怎么说

> 当前项目已经在仿真中打通了主要功能链路，但还没有完成真机部署和真实环境验证，因此它更接近高完成度仿真系统，而不是真正完成落地验证的机器人系统。

### 5. 更强的异常恢复机制

#### 现在已经做到什么

- 系统已经能使用 Nav2 自带的一些恢复动作
- 例如旋转、后退、清除代价地图等
- 自动探索层面也已经有一些失败目标冷却和重试逻辑

这说明你的系统不是“出错就完全停摆”。

#### 为什么它还不够成熟

因为现在的恢复能力仍然更多是：

- 通用恢复
- 局部补救

而不是：

- 针对任务语义设计过的更智能恢复体系

成熟一点的系统会进一步考虑：

- 某个 frontier 连续失败后是否自动降权或跳区
- coverage 在某区域多次失败后是否自动局部重规划
- 定位可疑时是否先主动重定位
- 长时间无进展时是否进入任务级故障处理
- 多次恢复仍失败时是否切换成安全退出或人工接管

你当前已经有“恢复动作”，但还没有“更强的恢复策略设计”。

#### 如果以后继续补，下一步该做什么

- 为不同失败类型建立分类：
  - 规划失败
  - 控制失败
  - 定位可疑
  - frontier 无效
  - coverage 局部卡死
- 针对不同失败类型设计不同恢复策略
- 记录恢复行为是否有效
- 建立“什么时候继续尝试、什么时候换策略、什么时候终止任务”的规则

#### 你面试时可以怎么说

> 当前系统已经具备基础恢复能力，但恢复策略更多依赖 Nav2 的通用机制，还没有建立面向自动探索和覆盖任务的更强异常恢复体系，所以我认为这部分仍然不够成熟。

### 这 5 项的共同特点

你可以把它们统一理解成一句话：

> 它们都不属于“主链路没跑通”的问题，而属于“项目离更成熟的工程系统和更有说服力的研究项目还有多远”的问题。

所以把它们放在“还不够成熟的部分”，不是贬低项目，而是帮助你更准确地判断：

- 这个项目已经做到了什么
- 它距离更高水平还差在哪
- 你后面如果继续做，最值得往哪里发力

所以你这个项目不是“课程实验玩具”，但也还不是“成熟机器人产品”。

---

# 6. 四个包分别是干什么的

## 6.1 `cleaning_robot_description`

这是机器人本体描述包。

它的核心任务是：

- 描述机器人结构
- 定义机器人链路
- 定义传感器位置
- 定义 Gazebo 中的运动插件

### 关键文件

- `src/cleaning_robot_description/urdf/robot.urdf.xacro`
- `src/cleaning_robot_description/urdf/base.xacro`
- `src/cleaning_robot_description/urdf/sensors.xacro`

### 你必须知道的最关键点

在 [robot.urdf.xacro](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_description/urdf/robot.urdf.xacro:1) 里，挂了一个关键 Gazebo 插件：

```xml
<plugin name="gazebo_ros_planar_move" filename="libgazebo_ros_planar_move.so">
```

它的作用是：

- 订阅 `/cmd_vel`
- 在 Gazebo 里驱动机器人移动
- 发布 `/odom`
- 发布 `odom -> base_footprint` 这段 TF

这件事非常关键，因为它说明：

**你的里程计不是自己手写出来的，而是仿真插件在提供。**

也就是说：

- 导航里的 `odom`
- 建图里的里程计参考
- AMCL 的底层运动参考

都和这个插件有关。

如果面试官问你：

“你这个机器人是怎么在 Gazebo 里被驱动起来的？”

你就可以回答：

“通过 `gazebo_ros_planar_move` 插件订阅 `/cmd_vel`，在仿真里做二维平面运动，同时发布 `/odom` 和 `odom -> base_footprint` TF。”

---

## 6.2 `cleaning_robot_simulation`

这是仿真环境包。

它负责：

- 提供 world 文件
- 启动 Gazebo
- 把机器人生成到世界里

### 关键文件

- `src/cleaning_robot_simulation/launch/sim.launch.py`
- `src/cleaning_robot_simulation/worlds/*.world`

### 关键世界文件

现在项目里主要有：

- `empty_room.world`
- `single_obstacle.world`
- `multi_obstacle.world`
- `narrow_passage.world`
- `multi_room.world`

### `sim.launch.py` 的核心逻辑

[sim.launch.py](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_simulation/launch/sim.launch.py:1) 主要做 3 件事：

1. 解析 `world_name`
2. 启动 Gazebo
3. 通过 `spawn_entity.py` 把机器人放进去

它有一个 `WORLD_PRESETS` 字典：

```python
WORLD_PRESETS = {
    "empty_room": "empty_room.world",
    "single_obstacle": "single_obstacle.world",
    "multi_obstacle": "multi_obstacle.world",
    "narrow_passage": "narrow_passage.world",
    "multi_room": "multi_room.world",
    "room": "room.world",
}
```

这说明 `world_name:=xxx` 本质上就是在选不同的 world 文件。

### 这层你最该理解的点

- Gazebo 负责“物理世界”
- world 文件负责“墙和障碍”
- `spawn_entity.py` 负责“把机器人放进去”
- 机器人能动，不是 launch 本身在动，而是 robot description 里的 Gazebo 插件在动

---

## 6.3 `cleaning_robot_bringup`

这是整个系统最重要的集成包。

它负责：

- 启动 SLAM
- 启动 Nav2
- 启动 AMCL
- 启动自动探索
- 启动地图保存
- 启动 RViz
- 组织不同运行模式

### 你必须重点看懂的文件

- `src/cleaning_robot_bringup/launch/slam.launch.py`
- `src/cleaning_robot_bringup/launch/nav.launch.py`
- `src/cleaning_robot_bringup/launch/cleaning.launch.py`
- `src/cleaning_robot_bringup/scripts/auto_explore_and_save.py`
- `src/cleaning_robot_bringup/config/slam_toolbox.yaml`
- `src/cleaning_robot_bringup/config/nav2_params.yaml`
- `src/cleaning_robot_bringup/config/auto_explore.yaml`
- `src/cleaning_robot_bringup/config/auto_explore/*.yaml`

### 这个包的三种主要运行模式

#### 模式 1：SLAM 建图

命令：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py
```

它会做：

- 启动仿真
- 启动 `slam_toolbox`
- 启动 Nav2 的导航核心
- 启动自动探索节点
- 自动保存地图

#### 模式 2：已知地图导航

命令：

```bash
ros2 launch cleaning_robot_bringup nav.launch.py
```

它会做：

- 启动仿真
- 启动 `map_server`
- 启动 `AMCL`
- 启动 `Nav2`
- 启动 RViz

#### 模式 3：覆盖清扫

命令：

```bash
ros2 launch cleaning_robot_bringup cleaning.launch.py
```

它会做：

- 先 include `nav.launch.py`
- 然后再启动 coverage 规划与执行相关节点

这就说明：

**`cleaning.launch.py` = `nav.launch.py` + coverage 任务层。**

---

## 6.4 `cleaning_robot_coverage`

这是清扫路径和任务逻辑包。

它负责：

- 生成弓字形路径
- 生成覆盖路径点
- 发布覆盖进度
- 执行覆盖任务管理

### 关键节点

- `coverage_planner`
- `coverage_progress_publisher`
- `cleaning_task_manager`

### 关键文件

- `src/cleaning_robot_coverage/cleaning_robot_coverage/coverage_planner.py`

### 你必须理解的点

这个包不是“底层导航器”，它是“任务层”。

它不负责：

- 直接做里程计
- 直接做定位
- 直接做建图

它负责的是：

- 根据地图生成清扫路线
- 把清扫路线交给导航层去执行

---

# 7. 三条主运行链路到底怎么串起来

这是整个项目最核心的部分。

## 7.1 SLAM 建图链路

当你运行：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py world_name:=multi_obstacle
```

系统大致按这个顺序工作：

1. `slam.launch.py` 启动
2. `sim.launch.py` 被 include
3. Gazebo 打开 world
4. `robot_state_publisher` / `joint_state_publisher` 启动
5. `spawn_entity.py` 把机器人放入世界
6. `gazebo_ros_planar_move` 开始发布 `/odom`
7. `slam_toolbox` 订阅 `/scan` 和 TF，开始建图
8. Nav2 导航链启动
9. `auto_explore_and_save.py` 启动
10. 自动探索节点不断找 frontier，调用 Nav2 去跑
11. 地图达到结束条件后，调用 `save_map` 服务保存 `.yaml` 和 `.pgm`

### 你必须记住

SLAM 模式下：

- 地图由 `slam_toolbox` 实时生成
- 机器人移动由 Nav2 + 自动探索控制
- 保存地图由 `auto_explore_and_save.py` 最后调用服务完成

---

## 7.2 已知地图导航链路

当你运行：

```bash
ros2 launch cleaning_robot_bringup nav.launch.py map:=... world_name:=...
```

系统顺序大致是：

1. `nav.launch.py` 启动
2. `sim.launch.py` 启动仿真
3. `map_server` 加载已有地图
4. `amcl` 用已有地图做定位
5. `bringup_launch.py` 启动 Nav2 核心节点
6. `publish_initial_pose.py` 给 AMCL 发布初始位姿
7. RViz 打开，等待你发目标点或别的任务层节点发目标

### 这里最关键的区别

SLAM 模式和导航模式的最大区别是：

- SLAM 模式：地图是边跑边建出来的
- 导航模式：地图已经固定好了，系统重点变成“定位 + 规划 + 跟踪”

---

## 7.3 覆盖清扫链路

当你运行：

```bash
ros2 launch cleaning_robot_bringup cleaning.launch.py map:=... world_name:=...
```

系统顺序大致是：

1. `cleaning.launch.py` include `nav.launch.py`
2. 仿真、地图、AMCL、Nav2 都先起来
3. `coverage_progress_publisher` 启动
4. `coverage_planner` 启动
5. `cleaning_task_manager` 启动
6. 规划器生成 `/coverage_waypoints`
7. 任务管理器按顺序把这些路径点交给 Nav2 执行
8. 覆盖进度节点不断根据机器人位置标记已覆盖区域

### 你必须记住

`cleaning.launch.py` 不是另起炉灶。

它是在“导航系统已经能跑”的基础上，再加一层“清扫任务逻辑”。

---

# 8. 关键 launch 文件逐个拆开讲

## 8.1 `slam.launch.py`

[slam.launch.py](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/launch/slam.launch.py:1) 是建图总入口。

### 它的关键职责

- 加载世界
- 启动 `slam_toolbox`
- 启动 Nav2 导航核心
- 启动自动探索
- 启动地图保存器

### 你要重点理解的地方

#### 1. `world_name` 和 `world`

- `world_name`：选预设场景
- `world`：直接给绝对路径，覆盖 `world_name`

#### 2. `map_output`

它是地图保存的基路径，不带后缀。

比如：

```bash
map_output:=src/cleaning_robot_bringup/maps/multi_obstacle
```

最终会生成：

- `multi_obstacle.yaml`
- `multi_obstacle.pgm`

#### 3. 世界参数覆盖

这个 launch 文件会根据 `world_name` 去找：

```text
config/auto_explore/<world_name>.yaml
```

比如：

- `multi_obstacle.yaml`
- `multi_room.yaml`
- `narrow_passage.yaml`

这意味着不同世界可以有不同的探索参数。

这就是你前面调 `narrow_passage` 和 `multi_obstacle` 时能按场景分别处理的原因。

---

## 8.2 `nav.launch.py`

[nav.launch.py](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/launch/nav.launch.py:1) 是已知地图导航总入口。

### 它的关键职责

- 启动仿真
- 启动 `map_server`
- 启动 `AMCL`
- 启动 `Nav2`
- 发布初始位姿
- 打开导航专用 RViz

### 你要重点理解的地方

#### 1. `map`

这是导航模式下最重要的输入。

例子：

```bash
map:=/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/maps/narrow_passage.yaml
```

#### 2. `publish_initial_pose.py`

这是一个你很容易忽视、但实际上非常重要的节点。

它负责：

- 根据 `spawn_x / spawn_y / spawn_yaw`
- 给 AMCL 发布初始位姿
- 帮助定位系统尽快收敛

如果没有它，AMCL 可能不知道机器人初始位置，定位会漂得更厉害。

---

## 8.3 `cleaning.launch.py`

[cleaning.launch.py](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/launch/cleaning.launch.py:1) 是清扫任务入口。

### 它的关键职责

- 先 include `nav.launch.py`
- 再启动 coverage 相关节点

### 它为什么重要

它说明你这个项目已经从“建图/导航系统”往“任务执行系统”迈了一步。

这是一个很能在面试里讲的点：

> 我不是只做了 SLAM 或 Nav2 演示，而是把导航层继续向上接到了覆盖清扫任务层。

---

# 9. 自动探索脚本 `auto_explore_and_save.py` 怎么理解

这是整个项目里最值得你花时间理解的脚本之一。

文件：

- [auto_explore_and_save.py](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/scripts/auto_explore_and_save.py:1)

## 9.1 它到底做什么

这个脚本本质上是在做一个“基于 frontier 的自动探索状态机”。

它会：

- 订阅 `/map`
- 从地图里找 frontier
- 选择一个合适的 frontier 目标
- 调用 Nav2 的 `NavigateToPose`
- 监控导航是否成功
- 失败时清理 costmap / 重新选点
- 最后在满足结束条件后保存地图

## 9.2 它依赖哪些东西

- `/map`
- TF
- Nav2 action：`/navigate_to_pose`
- 可选的 `/spin`
- `SaveMap` 服务
- `ClearEntireCostmap` 服务

## 9.3 你要重点理解的 5 个思想

### 1. 它不是“乱走”

它不是传统意义上的随机游走。

它更像：

- 从当前地图中提取 frontier
- 评估哪些 frontier 可达
- 按代价函数选一个最合适的目标
- 调用 Nav2 去走

### 2. 它有状态

脚本里会在这些状态之间切换：

- `WAIT_FOR_MAP`
- `WAIT_FOR_TF`
- `WAIT_FOR_NAV2`
- `SEND_GOAL`
- `NAVIGATING`
- `WAIT_FOR_FRONTIER`
- `SAVE_MAP`

所以它本质上是一个简单任务状态机。

### 3. 它的“地图是否还在增长”是会被监控的

脚本会记录：

- 已知面积
- 观测包围盒面积
- 包围盒未知面积

如果这些值在一段时间内不再明显变化，就认为地图进入停滞阶段。

### 4. 它有“失败目标冷却”

如果某个目标失败太多次，不会立刻重复撞上去。

脚本里有：

- `blocked_targets`
- `failed_clusters`
- `completed_targets`

这几个结构，就是你前面调“反复选择同一个目标”时的关键。

### 5. 它的结束不是只靠时间

结束条件不只是 timeout。

脚本还会看：

- 观测面积是否足够
- 未知面积是否够少
- frontier 是否基本耗尽
- 是否已经长时间停滞

所以它会出现你前面碰到的现象：

- 时间还没到，但它觉得“够了”，于是保存
- 或者地图明显没扫完，却被误判为完成

这就是为什么你后来专门调了：

- `completion_max_bbox_unknown_ratio`
- `completion_max_bbox_unknown_area`
- `frontier_completion_max_clusters`
- `frontier_completion_max_cells`
- `completion_require_both_bbox_metrics`

## 9.4 这个脚本为什么很适合拿来面试讲

因为它不是单纯“调用现成库”，而是体现了你对机器人任务流程的理解。

你可以讲：

- 我用 Nav2 负责底层导航
- 用自定义脚本负责上层探索逻辑
- 脚本根据 frontier 和地图增长情况决定探索目标与任务结束条件

这比只说“我用了 slam_toolbox”有内容得多。

---

# 10. 覆盖路径规划 `coverage_planner.py` 怎么理解

文件：

- [coverage_planner.py](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_coverage/cleaning_robot_coverage/coverage_planner.py:1)

## 10.1 它的目标是什么

给一张已经建好的地图，生成一条“尽量覆盖整个可通行区域”的弓字形路线。

输出主要是：

- `/coverage_path`
- `/coverage_waypoints`

## 10.2 它大致怎么工作

### 旧思路

一开始它更偏向：

- 按扫描线一行一行切
- 同一条水平线上所有线段依次拼接

问题是：

- 会出现左一段、中间一段、右一段反复切换
- 效率低

### 现在的改进思路

你后来已经把它改成了“cell 优先”：

- 先按扫描线切段
- 再把上下有重叠的线段归成一个 cell
- 先在 cell 内做弓字形覆盖
- 再在 cell 之间做连接

这比原来更像真实机器人会采用的区域优先思路。

## 10.3 它的核心参数是什么

- `line_spacing`
- `wall_margin`
- `obstacle_margin`
- `waypoint_spacing`
- `turn_margin`

含义大概是：

- 扫描线隔多远
- 离墙多远
- 离障碍多远
- 路径点间距多大
- 转弯时两端收缩多少

## 10.4 为什么它还是原型，不是成熟商用品质

因为它虽然已经具备：

- 地图读取
- 弓字形生成
- 障碍膨胀
- cell 分组
- A* 连接

但它还没有：

- 更高级的房间分割
- 更强的全局路径排序优化
- 门口/窄通道的专门建模
- 更鲁棒的覆盖执行闭环

所以它是一个很好的“项目展示模块”，但不是最终工业级算法。

---

# 11. 关键配置文件到底在控制什么

## 11.1 `slam_toolbox.yaml`

文件：

- [slam_toolbox.yaml](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/config/slam_toolbox.yaml:1)

你要重点理解这些参数族：

- 帧相关：
  - `odom_frame`
  - `map_frame`
  - `base_frame`
- 传感器相关：
  - `scan_topic`
  - `min_laser_range`
  - `max_laser_range`
- 匹配相关：
  - `use_scan_matching`
  - `minimum_travel_distance`
  - `minimum_travel_heading`
- 回环相关：
  - `do_loop_closing`
  - `loop_search_maximum_distance`
- 惩罚项：
  - `distance_variance_penalty`
  - `angle_variance_penalty`

你前面在 `narrow_passage` 漂移排查里，本质上就是在利用这些参数让 `slam_toolbox` 在对称走廊里更依赖稳定里程计，而不是过度自由漂移。

## 11.2 `nav2_params.yaml`

文件：

- [nav2_params.yaml](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/config/nav2_params.yaml:1)

你最该理解这些模块：

- `amcl`
- `planner_server`
- `controller_server`
- `local_costmap`
- `global_costmap`
- `behavior_server`

### `amcl`

负责：

- 已知地图定位
- 广播 `map -> odom`

这是你前面“机器人最后突然跳位”的高概率根源。

### `planner_server`

负责：

- 全局路径规划

你这里用的是：

- `NavfnPlanner`
- `use_astar: true`

### `controller_server`

负责：

- 局部轨迹跟踪
- 把全局路径变成实际运动控制

你这里用的是：

- `DWBLocalPlanner`

### `behavior_server`

负责：

- recovery 行为
- `spin`
- `backup`
- `wait`

这就是为什么你日志里会看到机器人在失败后：

- 先后退
- 再旋转
- 再重试

## 11.3 `auto_explore.yaml` 与 world profile

这些文件控制“自动探索策略”：

- [auto_explore.yaml](/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/config/auto_explore.yaml:1)
- `config/auto_explore/multi_obstacle.yaml`
- `config/auto_explore/multi_room.yaml`
- `config/auto_explore/narrow_passage.yaml`

你前面的大量调试，基本都集中在这里。

比如：

- `narrow_passage`：更强调 bootstrap spin、防止沿走廊漂移、完成条件更严
- `multi_obstacle`：更晚结束、追更小 frontier
- `multi_room`：覆盖路径与探索细节适配房间结构

这说明你项目已经从“单一默认参数”走向“多世界 profile”了，这点在面试里是非常加分的。

---

# 12. 这项目里最重要的几个坐标系是谁发的

这是高频面试题。

## `odom -> base_footprint`

来源：

- Gazebo 的 `gazebo_ros_planar_move`

作用：

- 提供仿真里程计

## `base_footprint -> base_link -> lidar_link / imu_link`

来源：

- URDF + `robot_state_publisher`

作用：

- 传感器安装位姿
- 机器人结构 TF

## `map -> odom`

来源分两种：

- SLAM 模式：`slam_toolbox`
- 导航模式：`AMCL`

这是最关键的一点。

如果你搞不懂 `map -> odom` 谁在发，你就会很难看懂为什么：

- SLAM 模式会漂
- 导航模式会跳
- RViz 里机器人位置和 Gazebo 里不一致

---

# 13. 你前面已经遇到过哪些“真实机器人软件问题”

这部分就是你以后面试最值钱的材料。

## 13.1 `narrow_passage` 里 SLAM 漂移

现象：

- 地图整体沿通道方向错位

本质：

- 对称环境
- scan matching 退化
- 初始探索策略放大漂移

处理思路：

- 调 `slam_toolbox`
- 加 bootstrap spin
- 按场景单独 profile
  
## 13.2 自动探索提前结束

现象：

- 明显还没扫干净就保存了

本质：

- 探索结束条件太宽松
- frontier 剩余被误判可忽略

处理思路： 

- 收紧 `completion_*`
- 收紧 `frontier_completion_*`
- 启用 `completion_require_both_bbox_metrics`

## 13.3 自动探索反复选择同一目标

现象：

- 到达过一个目标后又立刻重选同一个地方

本质：

- 最近完成目标没有短时冷却

处理思路：

- 加 `completed_targets`
- 对最近完成目标/簇做冷却

## 13.4 覆盖路径低效

现象：

- 同一水平线左中右反复穿插扫

本质：

- 全局扫描线优先
- 没有区域/cell 概念

处理思路：

- 改成 cell 优先

## 13.5 导航末期定位跳变

现象：

- 机器人在 RViz 里突然跳位置

本质更像：

- `AMCL` 定位跳解
- `map -> odom` 突然改了

而不是：

- 底层 TF 发布器彻底断掉

---

# 14. 你现在已经具备的真实能力是什么

不要低估这一点。

你现在已经做过：

- 启动完整机器人仿真系统
- 切换不同 world
- 自动探索建图
- 保存地图
- 已知地图导航
- 覆盖路径规划与执行
- 定位跳变排查
- SLAM 漂移排查
- world-specific 参数调优
- launch / config / 脚本联动调试

这已经不是“只会运行官方例子”的水平了。

你现在缺的不是“完全没有能力”，而是：

- 需要把理解系统化
- 需要把项目表述专业化

---

# 15. 你怎么才能把这个项目真正“学成自己的”

我的建议是：不要试图一下子把全仓库每一行代码都看懂。

你应该按“80/20 原则”去学。

## 第一优先级：必须看懂

- `slam.launch.py`
- `nav.launch.py`
- `cleaning.launch.py`
- `sim.launch.py`
- `robot.urdf.xacro`
- `auto_explore_and_save.py`
- `coverage_planner.py`
- `slam_toolbox.yaml`
- `nav2_params.yaml`

## 第二优先级：看懂 world profile

- `multi_obstacle.yaml`
- `multi_room.yaml`
- `narrow_passage.yaml`

## 第三优先级：再看执行层

- `cleaning_task_manager`
- `coverage_progress_publisher`
- `publish_initial_pose.py`

## 第四优先级：最后再补底层细节

- DWB critic 细节
- AMCL 粒子滤波细节
- slam_toolbox 更底层的求解机制

---

# 16. 读完本文后，你应该至少能回答这些问题

- 这个项目的四个包分别干什么？  
- `slam.launch.py` 和 `nav.launch.py` 有什么区别？
- Gazebo 中是谁在发布 `/odom`？
- `map -> odom` 在 SLAM 和导航模式下分别是谁发的？
- 为什么自动探索会提前结束？
- 为什么 `narrow_passage` 会漂移？
- 覆盖路径为什么会效率低？
- 你是怎么把覆盖路径从“全局扫描线优先”改成“cell 优先”的？
- 为什么 AMCL 可能在末期突然跳位置？

如果这些你能讲清楚，你对项目的理解已经比很多“只会跑命令”的人强得多了。

---

# 17. 面试时你可以怎么诚实地描述这个项目

你完全可以这样说：

> 这个项目是在 ROS 2 Humble 上实现的扫地机器人仿真系统，包含机器人模型、Gazebo 多场景仿真、SLAM 建图、AMCL+Nav2 已知地图导航，以及覆盖式清扫路径规划与执行基础链路。我重点参与了系统联调、自动探索流程调优、多场景 world profile 参数设计、覆盖路径策略改进，以及定位与建图异常问题排查。

这句话的优点是：

- 不夸大
- 也不把自己说得很弱
- 能准确反映你做过的事情

---

# 18. 100 道进阶面试题

下面这 100 道题，故意设计得比普通实习面试稍微难一点。

你不用一次全答出来，但如果你能认真把这 100 题想清楚，你对这个项目的理解会明显提升，而且很多题还能迁移到别的机器人项目。

我把它们分成 10 组，每组 10 题。

---

## A. 基础概念题（1-10）

1. ROS 2 中“节点、话题、服务、Action、TF”分别是什么？它们在你的项目里各对应哪些具体例子？
2. 为什么机器人项目里不能只靠 Topic，而一定会大量用到 TF？
3. `base_footprint` 和 `base_link` 的区别是什么？为什么很多导航系统喜欢用 `base_footprint`？
4. 什么是 Occupancy Grid？为什么地图里会同时存在空闲、障碍和未知三种状态？
5. 什么是 `map`、`odom`、`base_footprint` 这三个坐标系？三者为什么不能混为一谈？
6. 为什么 launch 文件不是算法文件，但在机器人项目里仍然非常重要？
7. 为什么一个机器人项目往往会拆成 `description / simulation / bringup / task` 这种多个包？
8. `cmd_vel` 是什么？为什么发布速度指令并不等于机器人一定会按理想轨迹运动？
9. 为什么机器人定位、建图、导航往往要依赖同一套传感器，却又是三个不同问题？
10. 你怎么向一个完全不懂 ROS 的人解释这个项目的系统结构？

---

## B. 仿真与模型题（11-20）

11. 机器人在 Gazebo 中是如何被 spawn 出来的？请按节点和 launch 角度说明流程。
12. `gazebo_ros_planar_move` 在你的项目里扮演什么角色？它做了哪些事？
13. 如果 `gazebo_ros_planar_move` 不发布 `/odom`，项目里哪些功能会最先失效？
14. world 文件和 URDF/Xacro 文件分别描述什么？二者的边界是什么？
15. 为什么说 `robot_state_publisher` 负责的是“结构 TF”，而不是“运动 TF”？
16. 如果你要把机器人底盘半径增大，最有可能需要改哪些文件？为什么？
17. 传感器安装位置为什么会影响导航与建图效果？
18. 为什么你的项目里可以只做二维平面运动，而不必上完整差速驱动动力学？
19. 仿真中 `/odom` 很稳定，为什么定位仍然可能跳？
20. 如果将来要迁移到真机，仿真层里哪些东西最可能不能直接复用？

---

## C. SLAM 题（21-30）

21. `slam_toolbox` 在你的项目里输入什么、输出什么？
22. 为什么 `slam_toolbox` 能在 SLAM 模式下提供 `map -> odom`？
23. 在对称环境里，为什么 SLAM 更容易漂移？
24. `use_scan_matching`、`do_loop_closing` 这类参数分别会影响什么行为？
25. 为什么 `minimum_travel_distance` 和 `minimum_travel_heading` 太大或太小都不好？
26. 什么是回环检测？它为什么能修正地图误差？
27. 为什么你在 `narrow_passage` 里要让 scan matching 稍微更偏向 odometry？
28. `min_laser_range`、`max_laser_range` 设置不合适时会造成哪些问题？
29. 如果 SLAM 地图整体“横向平移”但系统没崩，这更像是哪个层面的问题？
30. 你会如何区分“TF 链断了”和“SLAM / 定位解跳了”？

---

## D. Nav2 与定位题（31-40）

31. `nav.launch.py` 里为什么要同时启动 `map_server`、`amcl` 和 Nav2？
32. 已知地图导航模式下，`map -> odom` 为什么由 `AMCL` 而不是 `slam_toolbox` 发布？
33. `publish_initial_pose.py` 为什么重要？如果没有它会发生什么？
34. `AMCL` 的本质思想是什么？为什么它会用粒子而不是一个单一位姿？
35. 为什么在场景对称时，`AMCL` 也可能发生跳解？
36. `planner_server`、`controller_server`、`bt_navigator` 分别负责什么？
37. 全局路径规划和局部路径跟踪在概念上有什么区别？
38. `DWBLocalPlanner` 和全局规划器之间是什么关系？
39. 为什么日志里会出现 `backup`、`spin`、`wait` 这些恢复行为？
40. 如果 RViz 中机器人位置突然跳到角落，而 Gazebo 中机器人实体没瞬移，这通常意味着什么？

---

## E. 自动探索题（41-50）

41. 什么是 frontier？为什么 frontier 能驱动探索？
42. 自动探索脚本为什么要自己做“选点逻辑”，而不是让 Nav2 自己决定去哪？
43. `auto_explore_and_save.py` 里“可达 frontier”和“不可达 frontier”是怎么区分的？
44. 为什么探索任务需要记录 `blocked_targets`、`failed_clusters` 和 `completed_targets`？
45. 自动探索反复选择同一目标的根因是什么？你是怎么修的？
46. 为什么自动探索不能只按“离我最近”选 frontier？
47. `frontier_target_offset` 和 `frontier_clearance_radius` 这两个参数分别影响什么？
48. 为什么自动探索的结束不应该只看“时间到了没”？
49. `completion_max_bbox_unknown_ratio` 和 `completion_max_bbox_unknown_area` 各自反映了什么？
50. 在 `multi_obstacle` 中你为什么需要让系统去追更小的 residual frontier？

---

## F. 覆盖路径题（51-60）

51. 弓字形覆盖路径的基本思想是什么？它为什么适合扫地机器人？
52. 为什么“按整图扫描线全局串联”会在障碍物较多的地图里很低效？
53. 你现在的 `coverage_planner` 是如何从地图中提取可扫区域的？
54. 为什么要对障碍物和未知区域做膨胀？
55. `line_spacing`、`wall_margin`、`obstacle_margin` 分别决定什么？
56. 为什么要把路径端点向内收缩一个 `turn_margin`？
57. 什么叫“cell 优先”的覆盖策略？它相比原来的全局扫描线方式改进了什么？
58. `coverage_planner` 里的 A* 连接解决了什么问题？
59. 为什么覆盖路径生成和覆盖任务执行要分成两个模块？
60. 如果未来要进一步提升覆盖效率，你觉得应该优先改“区域分解”还是“局部控制”？为什么？

---

## G. 调参与排障题（61-70）

61. 当你看到 `frame does not exist` 时，第一反应应该检查什么？
62. 当你看到地图明显还没扫完但系统已经保存退出时，优先怀疑哪类参数？
63. 为什么 `exploration_timeout` 和 `max_exploration_timeout` 都存在？二者区别是什么？
64. 为什么把 `exploration_timeout` 设大了，系统仍然可能提前结束？
65. 为什么地图保存成功不等于探索完整？
66. 如果某个 world 总是在相似完成度下结束，这说明什么？
67. 为什么你会优先使用 world-specific profile，而不是一套全局默认参数到处用？
68. 在 `multi_obstacle` 里你收紧了哪些参数？每一类参数的直觉含义是什么？
69. 定位跳变问题为什么不能简单粗暴地归结为“TF 链不稳定”？
70. 如果导航末期路径越来越怪、恢复行为越来越多，你会优先看哪个层：定位、全局规划还是局部控制？为什么？

---

## H. 工程设计题（71-80）

71. 为什么你把项目拆成多个包，而不是所有东西都写在一个包里？
72. 为什么不同 world 要有独立的 yaml 参数 profile？
73. `slam.launch.py`、`nav.launch.py`、`cleaning.launch.py` 这种分层入口设计有什么好处？
74. 如果以后还要加入 `visual_slam.launch.py` 或真机入口，你会如何保持架构清晰？
75. 你这个项目里哪些代码属于“系统编排层”，哪些属于“算法层”，哪些属于“任务层”？
76. 为什么 launch 参数设计得清晰，对后续调试和回归验证很重要？
77. 如果你要让别人复现实验，项目里哪些材料必须准备齐全？
78. 你会如何设计一个更像工业项目的“配置管理”方式？
79. 为什么说“能跑通”和“可维护”是两件不同的事？
80. 如果面试官让你评价这个项目目前的工程成熟度，你会怎么诚实地回答？

---

## I. 进阶扩展题（81-90）

81. 如果你要把这个项目从纯仿真迁移到真机，最先要替换哪几层？
82. 真机上为什么不能再依赖 `gazebo_ros_planar_move`？
83. 如果要把 2D 激光导航升级成视觉+激光融合，你会从哪些模块切入？
84. 如果要把 coverage 规划做得更像商用品质，下一步最值得引入什么：房间分割、拓扑图、TSP 排序还是学习方法？
85. 如果要把自动探索改成更先进的探索器，你觉得可以参考哪些思路？
86. 这个项目的定位跳变问题，如果从算法上解决，你会优先研究 `AMCL`、回环建图、传感器融合还是地图质量评估？
87. 如果要支持动态障碍物，这个系统哪些模块最先受影响？
88. 如果要让清扫任务支持断点续扫，你觉得要新增哪些状态信息？
89. 如果要做实验对比，哪些指标最能衡量覆盖规划改进是否有效？
90. 如果把这个项目变成毕业方向，你会把论文问题定义成什么？

---

## J. 迁移与泛化题（91-100）

91. 这个项目里你学到的哪些能力可以迁移到无人车、巡检机器人、仓储机器人？
92. 如果把 world 从室内房间换成长走廊办公区，哪一类问题会被放大？
93. 如果把机器人换成更大尺寸底盘，为什么不仅要改 URDF，还要改导航参数？
94. 如果地图分辨率从 `0.05` 改成 `0.02`，哪些模块会受到连锁影响？
95. 如果局部规划器从 `DWB` 换成 `TEB` 或 MPPI，你预计会带来哪些变化？
96. 这个项目里哪些部分是“机器人项目通用知识”，哪些是“你这个项目的特定实现”？
97. 如果面试官问你“你最大的技术成长是什么”，你会从哪个问题讲起最有说服力？
98. 如果面试官问“这个项目最弱的一点是什么”，你会怎么答既诚实又不显得太弱？
99. 如果让你从零重新做一次这个项目，你会保留哪些设计，推翻哪些设计？
100. 如果让你带一个比你更新的同学快速理解这个项目，你会按什么顺序教他？

---

# 19. 如何使用这 100 道题

建议你不要一口气全做。

更实际的节奏是：

## 第 1 轮

先做 1-30 题。

目标：

- 先建立系统骨架理解

## 第 2 轮

做 31-60 题。

目标：

- 真正搞懂导航、探索、覆盖三条链路

## 第 3 轮

做 61-80 题。

目标：

- 学会像工程师一样分析问题

## 第 4 轮

做 81-100 题。

目标：

- 开始具备扩展项目、评价系统、迁移思维的能力

如果你真的把这 100 题认真想过一遍，你对这个项目的掌握程度绝对会大幅上升。

---

# 20. 最后给你的一个很重要的结论

你现在并不是“什么都不懂”。

你只是：

- 代码最初大量借助了 AI
- 但你已经真实地把系统跑起来了
- 也已经开始在真实问题上做观察、质疑、调整和改进

这意味着你真正缺的不是“能力从零到一”，而是：

- 把零散理解整理成系统理解
- 把“会跑”升级成“会讲、会解释、会复盘”

如果你能把这份文档吃下去，再认真思考那 100 道题，你会从“依赖 AI 跑项目”逐渐变成“能把这个项目真正讲成自己的项目”。

这一步非常关键。

也是你从“会用工具的人”变成“能做机器人工程的人”的开始。

---

# 21. 用“数据流”再讲一遍整个项目

前面我们已经从“包”和“功能”两个角度讲过项目了。

但很多初学者真正卡住的地方不是“这个文件叫啥”，而是：

- 数据到底从哪里来？
- 又流到哪里去？
- 哪个节点在订阅？
- 哪个节点在发布？
- 出问题时到底该先怀疑谁？

所以这一节我们完全不讲抽象定义，只讲“数据流”。

## 21.1 最基础的一条数据流：仿真传感器数据怎么走

以激光雷达为例。

你可以把它想成：

1. Gazebo 世界里有一台机器人。
2. 机器人身上挂着一个激光雷达插件。
3. 插件根据周围障碍物，模拟出一圈距离测量值。
4. 这些测量值被发布成 ROS 2 话题 `/scan`。
5. 后面的 SLAM、AMCL、RViz 都会去用 `/scan`。

所以最短路径是：

`Gazebo world -> 雷达插件 -> /scan -> SLAM / AMCL / RViz`

这说明：

- 如果 `/scan` 没有，SLAM 一定做不了。
- 如果 `/scan` 有但 TF 不对，SLAM 也做不好。
- 如果 `/scan` 数据不稳定，AMCL 也可能跳。

## 21.2 里程计数据怎么走

在你的项目里，`odom -> base_footprint` 这一段主要来自 Gazebo 侧的平面移动插件。

它大致流程是：

1. 控制器收到速度命令 `/cmd_vel`。
2. Gazebo 里的机器人在仿真中移动。
3. 平面移动插件根据机器人的移动，计算一个里程计估计。
4. 插件发布 `/odom` 和 `odom -> base_footprint`。

最短路径可以记成：

`/cmd_vel -> Gazebo robot motion -> /odom + odom->base_footprint`

这说明：

- 如果 `/cmd_vel` 在发但 Gazebo 机器人不动，优先怀疑运动插件或碰撞/卡住。
- 如果 Gazebo 机器人在动但 RViz 机器人不动，优先怀疑 TF 或 `/odom`。
- 如果 `odom` 很稳定但地图位置突然跳，那大概率是 `map -> odom` 变了，不是底层轮式运动真的瞬移了。

## 21.3 SLAM 时地图怎么长出来

SLAM 的核心输入主要是两样：

- `/scan`
- TF，尤其是机器人姿态相关坐标变换

`slam_toolbox` 会不断做一件事：

1. 读取当前一帧激光。
2. 根据已有地图和历史轨迹，估计“这帧激光最可能来自哪里”。
3. 用这个估计姿态把当前激光投到地图上。
4. 地图一点一点长出来。
5. 同时它会维护 `map -> odom`。

你可以把它记成：

`/scan + TF -> slam_toolbox -> /map + map->odom`

所以：

- 地图歪了，先查 `/scan`、TF、场景几何对称性、SLAM 参数。
- 地图没长，先看 `/scan` 有没有、机器人动没动、SLAM 节点是不是活着。

## 21.4 已知地图导航时定位怎么工作

当你不再建图，而是加载一张现成地图时，系统要做的是“我现在在这张地图的哪里”。

这时主要是 `AMCL` 上场。

AMCL 大致做：

1. 读取一张静态地图。
2. 维护很多“粒子”，每个粒子代表机器人可能所在的位置。
3. 根据 `/odom` 预测机器人可能移动到了哪里。
4. 根据 `/scan` 比较“如果机器人在这里，会不会看到这样的激光”。
5. 给更合理的粒子更高权重。
6. 最后输出一个最可能的定位结果。

所以可以记成：

`static map + /odom + /scan -> AMCL -> /amcl_pose + map->odom`

你一定要意识到：

- AMCL 不创造地图。
- AMCL 是“拿现成地图做定位”。
- 它也会影响 `map -> odom`。

这就是为什么你后来在导航阶段看到机器人“在 RViz 里跳了”，但 Gazebo 里的实体并没有真的瞬移。

## 21.5 Nav2 是怎么拿着目标点让机器人动起来的

Nav2 的大体工作流程可以粗略理解成三段：

1. 全局规划：从当前点到目标点，先算出一条大致路径。
2. 局部控制：根据当前局部障碍和路径，算出此刻速度命令。
3. 恢复行为：如果走不通，尝试旋转、后退、清空局部代价地图等。

数据流可以记成：

`goal -> planner_server -> global path -> controller_server -> /cmd_vel`

如果失败：

`planner/controller failure -> behavior_server -> spin/backup/clear costmap`

这说明：

- 有全局路径但不动，常看 controller。
- 连全局路径都出不来，常看 planner、costmap、定位。
- 反复恢复行为，多半不是“恢复行为有病”，而是更前面的地图、定位或目标选择出了问题。

## 21.6 自动探索到底在 Nav2 上面做了什么

自动探索不是自己发速度，它更像一个“上层任务调度器”。

它做的事是：

1. 读地图。
2. 找 frontier。
3. 从里面挑一个目标。
4. 把这个目标通过 Nav2 送出去。
5. 等 Nav2 执行。
6. 失败了就换目标，成功了就继续。
7. 最后判断是否结束并保存地图。

所以它是：

`/map -> frontier selection -> Nav2 goal -> wait result -> repeat -> save map`

它不是底层控制器。

这点你以后面试时一定要说清楚，不然很容易把“自动探索”和“底盘控制”混为一谈。

## 21.7 覆盖规划到底在 Nav2 上面做了什么

覆盖规划也不是自己控制轮子。

它做的事是：

1. 读取地图。
2. 提取可清扫区域。
3. 生成弓字形路径段或关键点。
4. 逐个交给导航系统执行。

也就是说它和自动探索有点像：

- 自动探索面向“未知空间”
- 覆盖规划面向“已知自由空间”

二者都站在 Nav2 之上，只是目标不同。

---

# 22. 你应该会用到的 Topic、Service、Action 清单

这一节不是让你背，而是让你以后排障时知道“先看哪里”。

## 22.1 常见 Topic

### `/scan`

含义：

- 激光雷达数据

谁会用：

- `slam_toolbox`
- `AMCL`
- RViz

没有它会怎样：

- 建图不行
- 定位不行

### `/odom`

含义：

- 机器人相对里程计坐标系的运动估计

谁会用：

- `AMCL`
- `Nav2`
- RViz

如果它乱，会怎样：

- 导航不稳定
- 定位预测变差

### `/map`

含义：

- 当前地图

谁会用：

- RViz
- 自动探索
- Nav2 代价地图

### `/amcl_pose`

含义：

- AMCL 给出的当前定位结果

谁会用：

- 你自己观察
- 有些上层逻辑

### `/cmd_vel`

含义：

- 速度控制命令

谁会用：

- Gazebo 机器人运动插件

如果它一直没有：

- 机器人就不会动

### `/plan`

含义：

- 全局路径

谁会用：

- RViz
- 调试

如果它出不来：

- 说明全局规划可能失败

## 22.2 常见 Service

### 保存地图服务

作用：

- 把当前 `/map` 保存成 `.yaml + .pgm`

你之前看到的：

- `Map saved successfully`

就是这个服务调用成功了。

### 清除 costmap 服务

作用：

- 当路径规划被旧障碍、坏数据、局部卡死影响时，清理代价地图

## 22.3 常见 Action

### `/navigate_to_pose`

作用：

- Nav2 最常见的导航动作接口

调用者：

- RViz 手动打点
- 自动探索脚本
- 覆盖任务层

你可以把它理解成：

“请帮我把机器人导航到这个目标点。”

### 为什么 Action 重要

因为它不是一次性消息，而是带状态的任务：

- 发送目标
- 执行中
- 成功
- 失败
- 取消

这对自动探索和覆盖任务非常关键，因为上层逻辑必须知道“这一单做完了没有”。

---

# 23. 从命令行到系统结束：完整走一遍 3 个典型例子

这一节很重要，因为很多小白会“会敲命令，但脑中没有完整流程”。

## 23.1 例子一：SLAM 建图并自动保存地图

你输入：

```bash
ros2 launch cleaning_robot_bringup slam.launch.py \
  world_name:=multi_obstacle \
  map_output:=src/cleaning_robot_bringup/maps/multi_obstacle
```

脑中应该出现下面这条流程：

1. 读取 `slam.launch.py`
2. 它解析启动参数，比如 world 名、地图输出路径、是否开 RViz
3. 它 include `sim.launch.py`
4. `sim.launch.py` 根据 `world_name` 找到具体 world 文件
5. Gazebo 打开这个世界
6. 机器人模型被加载并 spawn 进去
7. Gazebo 插件开始发 `/scan`、`/odom`
8. `slam_toolbox` 启动并开始发布 `/map`
9. Nav2 启动，自动探索脚本开始工作
10. 自动探索不断发导航目标
11. 机器人边走边建图
12. 到达结束条件后，自动探索脚本调用地图保存服务
13. `multi_obstacle.yaml` 和 `multi_obstacle.pgm` 被写出来

如果中间失败，你要知道大概可能断在哪：

- Gazebo 没起来：仿真层
- 机器人没 spawn：模型/launch 层
- `/scan` 没有：传感器/插件层
- 地图不长：SLAM 层
- 机器人不动：Nav2 或控制链路
- 地图没保存：auto_explore 结束逻辑或保存服务

## 23.2 例子二：加载已知地图做导航

你输入：

```bash
ros2 launch cleaning_robot_bringup nav.launch.py \
  map:=/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/maps/multi_room.yaml \
  world_name:=multi_room
```

脑中应该出现：

1. 打开 Gazebo 中的 `multi_room.world`
2. 机器人被放进世界
3. `map_server` 加载 `multi_room.yaml`
4. `AMCL` 启动
5. `Nav2` 启动
6. RViz 打开
7. 你手动发布初始位姿
8. `AMCL` 开始定位
9. 你在 RViz 发一个 2D 导航目标
10. Nav2 规划路径并让机器人走过去

你必须注意：

- 世界和地图要对应
- 如果地图来自 `multi_room`，但 world 用了 `narrow_passage`，定位几乎一定出问题

## 23.3 例子三：覆盖清扫

你输入：

```bash
ros2 launch cleaning_robot_bringup cleaning.launch.py \
  map:=/home/aderfd/cleaning_robot/cleaning_robot_ws/src/cleaning_robot_bringup/maps/narrow_passage.yaml \
  world_name:=narrow_passage
```

脑中应该出现：

1. 先走完整个 `nav.launch.py` 链路
2. 也就是仿真、地图、AMCL、Nav2 都先起来
3. 然后 coverage 相关节点启动
4. `coverage_planner` 读取地图
5. 生成弓字形清扫路径
6. 将路径拆成一个个导航目标或轨迹片段
7. 交给 Nav2 去执行
8. 机器人开始按覆盖路线运行

所以：

- 如果定位没稳，覆盖执行一定差
- 如果 Nav2 本身不稳定，coverage 也会跟着不稳定
- coverage 层是任务层，不是神奇的“独立第二套导航系统”

---

# 24. 你最容易误解的 20 件事

这一节我专门写给“刚入门很容易把概念混掉”的你。

## 24.1 “机器人会动”不等于“导航是对的”

机器人能动只说明：

- 速度链路可能通了

不代表：

- 定位一定对
- 地图一定对
- 路径一定合理

## 24.2 “地图保存成功”不等于“地图完整”

保存成功只说明：

- 写文件成功

不代表：

- 探索完整
- 地图高质量

## 24.3 “TF 有”不等于“TF 对”

有时候你能看到某条 TF，但它：

- 方向错
- 数值跳
- 时间不同步

这都会让系统出问题。

## 24.4 “AMCL 在发结果”不等于“定位稳定”

AMCL 可以一直发 `/amcl_pose`，但：

- 协方差很大
- 粒子不收敛
- `map->odom` 在跳

这都说明定位不稳。

## 24.5 “有全局路径”不等于“能成功到达”

因为全局路径只是大方向。

真正让机器人走的是局部控制器。

## 24.6 “world_name” 不是随便填的字符串

它决定：

- 启动哪个 world
- 可能加载哪个 world profile
- 可能影响自动探索参数

## 24.7 “地图名字” 和 “地图内容” 不是一回事

`multi_obstacle.yaml` 只是文件名。

真正重要的是：

- 这张图是不是在对应世界里建出来的
- 建图时有没有漂移
- 分辨率和 origin 是否正确

## 24.8 “coverage 路线很花”不一定说明规划器高级

有时候反而说明：

- 切换太频繁
- 规划不够区域化
- 导航层在兜圈

## 24.9 “恢复行为很多”通常不是好事

频繁 `spin`、`backup`、清 costmap，往往表示前面已有问题：

- 目标点不合理
- 局部规划不好
- 定位不稳
- costmap 脏

## 24.10 “纯仿真跑通”不等于“真机能用”

仿真没有：

- 地面打滑
- 电机误差
- 真实噪声
- 网络延迟
- 传感器遮挡的复杂性

## 24.11 `slam_toolbox` 和 `AMCL` 不是一个东西

- `slam_toolbox`：建图
- `AMCL`：已知地图定位

## 24.12 `Nav2` 也不是“一个节点”

它是一整套导航系统。

## 24.13 `launch` 文件不是“简单脚本”

它其实是：

- 进程组织器
- 参数注入器
- 模式切换器

## 24.14 世界文件和地图文件不是同一种东西

- `world`：Gazebo 仿真世界
- `yaml/pgm`：ROS 里的二维占据栅格地图

## 24.15 RViz 不是“真相本身”

RViz 只是可视化工具。

它显示的内容依赖：

- 话题
- TF
- 固定坐标系
- 时间同步

## 24.16 机器人“看起来在墙里”不一定是碰撞坏了

也可能只是：

- 定位跳了
- 地图漂了
- RViz 中 `map` 和 Gazebo 世界对应关系错了

## 24.17 初始位姿非常重要

哪怕地图是对的，如果初始位姿给错很多：

- AMCL 也会很难收敛
- Nav2 也会先从错误位置开始规划

## 24.18 自动探索不是保证 100% 覆盖

自动探索目标通常是：

- 把未知区域尽量变成已知

它不是严格意义上的覆盖优化器。

## 24.19 覆盖规划不是建图

它假设地图已经基本可用。

## 24.20 你现在的项目已经不只是“一个 demo”

因为你已经在真实做：

- 参数调整
- 多场景适配
- 故障分析
- 任务链路打通

只是还没达到成熟产品级别。

---

# 25. 14 天把项目吃到 80% 的学习计划

你说你现在很多地方不懂，所以不能给你一个“去把所有源码看完”的建议，那样太容易崩。

我给你的是一个更现实的 14 天版本。

## 第 1 天：只看全局

任务：

- 通读本文前 8 节
- 不抠代码
- 目标是知道四个包分别干什么

产出：

- 你能手画出项目结构脑图

## 第 2 天：只看启动链路

任务：

- 看 `slam.launch.py`
- 看 `nav.launch.py`
- 看 `cleaning.launch.py`

产出：

- 你能讲出三种模式的区别

## 第 3 天：只看仿真层

任务：

- 看 `sim.launch.py`
- 看 world 文件
- 看 `robot.urdf.xacro`

产出：

- 你知道机器人是怎么进 Gazebo 的

## 第 4 天：只补 ROS2 基础概念

任务：

- 重新读 Node / Topic / TF / Action
- 自己解释一遍什么是 `map -> odom -> base_footprint`

产出：

- 不再把 TF 当黑盒

## 第 5 天：只看 SLAM

任务：

- 读 `slam_toolbox.yaml`
- 结合你自己的建图经历理解参数

产出：

- 你知道为什么 `narrow_passage` 会漂

## 第 6 天：只看 AMCL + Nav2

任务：

- 读 `nav2_params.yaml`
- 重点看 `amcl`、planner、controller

产出：

- 你知道导航为什么依赖定位稳定

## 第 7 天：只看自动探索

任务：

- 读 `auto_explore_and_save.py`
- 对照本文第 9 节

产出：

- 你知道“frontier 是怎么选出来的”

## 第 8 天：只看 world profile

任务：

- 比较 `auto_explore/narrow_passage.yaml`
- 比较 `auto_explore/multi_obstacle.yaml`
- 比较 `auto_explore/multi_room.yaml`

产出：

- 你知道为什么不同世界不能完全共用一套参数

## 第 9 天：只看 coverage

任务：

- 读 `coverage_planner.py`
- 理解为什么“按整张图扫描线串联”效率低

产出：

- 你能说出 coverage 的原型缺陷

## 第 10 天：回顾你遇到的 5 个真实问题

任务：

- SLAM 漂移
- 提前结束
- 重复 frontier
- coverage 低效
- AMCL 跳变

产出：

- 你能把这 5 个问题讲成“项目经历”

## 第 11 天：开始刷面试题 1-30

要求：

- 每题不追求完美
- 先能说出思路

## 第 12 天：刷面试题 31-60

要求：

- 多把问题和你项目里的现象联系起来

## 第 13 天：刷面试题 61-100

要求：

- 开始说“如果继续改，我会怎么做”

## 第 14 天：做一次项目口头复盘

任务：

- 不看文档
- 自己录音 10 到 15 分钟
- 讲完整个项目

如果你能做到这一步，说明这项目已经开始变成你自己的了。

---

# 26. 项目术语表：把最常见的词一次讲清楚

这一节很适合你反复查。

## `URDF`

机器人结构描述格式。

用来描述：

- 连杆
- 关节
- 传感器安装位置

## `Xacro`

一种可以生成 URDF 的模板语言。

优点：

- 可复用
- 参数化

## `Gazebo`

机器人仿真器。

负责：

- 世界
- 物理
- 传感器仿真

## `RViz`

机器人可视化工具。

负责显示：

- 地图
- 激光
- TF
- 路径

## `TF`

坐标变换系统。

核心问题：

- 谁相对谁在哪里

## `odom`

里程计坐标系。

特点：

- 连续
- 会漂

## `map`

全局地图坐标系。

特点：

- 理论上更稳定
- 常由 SLAM 或 AMCL 维护到 `odom` 的关系

## `base_footprint`

机器人底盘在地面上的参考坐标系。

## `base_link`

机器人主体坐标系。

## `lidar_link`

激光雷达坐标系。

## `IMU`

惯性测量单元。

通常测：

- 角速度
- 加速度

## `SLAM`

Simultaneous Localization And Mapping。

边定位边建图。

## `AMCL`

Adaptive Monte Carlo Localization。

已知地图定位算法。

## `Nav2`

ROS 2 导航栈。

包含：

- 规划
- 控制
- 恢复行为

## `costmap`

代价地图。

用来表示：

- 哪些地方能走
- 哪些地方危险
- 哪些地方需要绕开

## `frontier`

已知区域和未知区域的边界。

自动探索常把它当作“值得去的地方”。

## `occupancy grid`

占据栅格地图。

每个格子通常表示：

- 空闲
- 障碍
- 未知

## `pgm`

地图图像文件。

保存栅格图像本身。

## `yaml`

地图元信息文件。

通常包含：

- 图片路径
- 分辨率
- 原点
- 阈值

## `Action`

带执行状态反馈的长任务接口。

比如导航到目标点。

## `Service`

请求-响应式接口。

适合：

- 保存地图
- 清除 costmap

## `Topic`

连续消息流接口。

适合：

- 激光
- 里程计
- 速度命令

## `launch`

系统启动编排文件。

## `world profile`

某个特定 world 的专用参数配置。

比如：

- `narrow_passage.yaml`
- `multi_obstacle.yaml`

## `coverage`

覆盖清扫任务。

目标不是去“未知区域探索”，而是对“已知可达区域”做尽量完整的清扫。

---

# 27. 如果你只剩 30 分钟准备面试，你至少要记住这 15 件事

1. 这个项目是 `ROS 2 Humble + Gazebo + slam_toolbox + AMCL + Nav2 + coverage` 的集成项目。
2. `cleaning_robot_description` 负责机器人模型。
3. `cleaning_robot_simulation` 负责 world 和仿真启动。
4. `cleaning_robot_bringup` 负责把建图、导航、自动探索等主链路启动起来。
5. `cleaning_robot_coverage` 负责弓字形覆盖路径规划和执行。
6. `slam.launch.py` 用于建图，`nav.launch.py` 用于已知地图导航，`cleaning.launch.py` 用于覆盖任务。
7. `slam_toolbox` 负责边建图边定位。
8. `AMCL` 负责已知地图下的定位。
9. `Nav2` 负责规划路径和控制机器人移动。
10. 自动探索的核心思想是 frontier-based exploration。
11. 覆盖规划的核心思想是从地图中提取可清扫区域并生成弓字形路径。
12. 你项目里真实遇到过的问题包括 SLAM 漂移、提前结束、重复目标、覆盖低效、AMCL 跳变。
13. 这些问题并不是“项目失败”，恰恰说明你真的在做机器人工程调试。
14. 这个项目当前更强的是系统集成和工程调试，不是最强算法创新。
15. 你可以诚实地说项目开发过程中大量借助了 AI，但你自己完成了运行、观察、排障、参数调整和问题分析。
