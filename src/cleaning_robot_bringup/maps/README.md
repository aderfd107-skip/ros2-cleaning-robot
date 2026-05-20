导航前生成或放置的地图文件统一存放在这里。

这个目录是阶段 0 和阶段 1 的基线地图产物位置。

## 默认文件

- `room_map.yaml`
- `room_map.pgm`

## 当前作用

- 保存单房间验证环境生成的地图
- 为 `cleaning_robot_bringup/launch/nav.launch.py` 提供默认地图输入
- 作为后续阶段回归测试时的基线地图目录

## 当前基线流程

1. 在单房间世界中启动 SLAM。
2. 让机器人自动探索直到超时或触发存图。
3. 将生成的地图保存到本目录，保存时传入的是基路径而不是 `.yaml` 文件名。
4. 使用本目录中的地图启动已知地图导航验证。

## 验收检查

- 不修改源码时可以生成 `room_map.yaml` 和 `room_map.pgm`
- `nav.launch.py` 可以直接加载 `room_map.yaml`
- 这里保存的地图可以作为后续优化阶段的回归对照

## 手动存图示例

```bash
cd ~/cleaning_robot/cleaning_robot_ws
source install/setup.bash
ros2 run nav2_map_server map_saver_cli -f src/cleaning_robot_bringup/maps/room_map
```
