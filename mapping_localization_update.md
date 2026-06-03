# Mapping / Localization / Global Planner Update

本文档记录本次会话中围绕建图、定位和全局规划链路新增或调整的功能，并给出常用使用指令。

## 1. 新增 global_planner ROS 2 包

新增路径：

```text
src/global_planner
```

该包从 `src/OctoPlanner3D/planner` 中提取核心逻辑，构建为 ROS 2 全局路径规划包。

规划流程：

```text
OctoMap
  -> 可通行空间提取
  -> 机器人半径碰撞检测
  -> preblocked/risk cost layer
  -> 26 邻域 3D A*
  -> nav_msgs/Path
```

主要文件：

```text
src/global_planner/include/global_planner/global_planner_core.hpp
src/global_planner/src/global_planner_core.cpp
src/global_planner/src/global_planner_node.cpp
src/global_planner/config/global_planner.yaml
src/global_planner/launch/global_planner.launch.py
src/global_planner/rviz/global_planner.rviz
```

## 2. 地图接入方式

`global_planner` 现在支持两种地图来源，由 yaml 参数选择。

配置文件：

```text
src/global_planner/config/global_planner.yaml
```

### 2.1 从文件读取 OctoMap

推荐用于静态地图全局规划。

```yaml
map_source: file
map_file_path: /absolute/path/to/map.bt
```

支持格式：

```text
.bt
.ot
.pcd
```

启动后如果读取成功，会看到类似日志：

```text
Map loaded from file: ... resolution=... traversable=... preblocked=...
```

如果使用 PCD 地图，规划器会在启动时把 PCD 占据点转换为内部 OctoMap：

```yaml
map_source: file
map_file_path: /absolute/path/to/map.pcd
pcd_octomap_resolution: 0.20
pcd_voxel_leaf_size: 0.0
pcd_min_z: -1000.0
pcd_max_z: 1000.0
```

参数说明：

```text
pcd_octomap_resolution  PCD 转 OctoMap 的体素分辨率
pcd_voxel_leaf_size     PCD 预降采样尺寸，0 表示不降采样
pcd_min_z / pcd_max_z   只使用指定高度范围内的点
```

### 2.2 从话题接收 OctoMap

用于在线建图或外部地图发布节点。

```yaml
map_source: topic
octomap_topic: /octomap_full
```

话题类型：

```text
octomap_msgs/msg/Octomap
```

## 3. debug / 非 debug 模式

由 yaml 参数控制：

```yaml
debug_mode: true
```

### 3.1 debug_mode: true

用于调试全局规划器。

行为：

```text
手动选择起点
手动选择终点
完成全局路径规划
```

起点：

```text
/global_planner/start
geometry_msgs/msg/PoseWithCovarianceStamped
```

终点：

```text
/goal_pose
geometry_msgs/msg/PoseStamped
```

RViz 中使用：

```text
2D Pose Estimate -> 选择起点
2D Goal Pose     -> 选择终点
```

### 3.2 debug_mode: false

用于实际定位导航流程。

行为：

```text
起点来自 icp_localization_node 发布的定位结果
手动选择终点
完成全局路径规划
```

定位起点话题：

```yaml
localization_pose_topic: /icp_pose
```

消息类型：

```text
geometry_msgs/msg/PoseWithCovarianceStamped
```

终点仍然使用：

```text
/goal_pose
geometry_msgs/msg/PoseStamped
```

是否随定位更新持续重规划：

```yaml
replan_on_pose_update: false
```

如果需要机器人移动时持续重规划，可改为：

```yaml
replan_on_pose_update: true
```

## 4. RViz 可视化和选点

新增 RViz 配置：

```text
src/global_planner/rviz/global_planner.rviz
```

`global_planner.launch.py` 默认启动 RViz。

启动：

```bash
source install/setup.bash
ros2 launch global_planner global_planner.launch.py
```

只启动规划器，不启动 RViz：

```bash
ros2 launch global_planner global_planner.launch.py use_rviz:=false
```

RViz 显示内容：

```text
/global_planner/occupied_map  地图占据点云
/global_path                  全局路径
/icp_pose                     ICP 定位位姿
TF                            坐标系
Grid                          栅格参考
```

地图点云可视化由 `global_planner_node` 发布：

```yaml
occupied_map_cloud_topic: /global_planner/occupied_map
publish_occupied_map_cloud: true
```

## 5. 全局路径规划输出

规划成功后发布：

```text
/global_path
nav_msgs/msg/Path
```

常用检查命令：

```bash
ros2 topic echo /global_path
ros2 topic info /global_path
```

## 6. 定位与 FAST-LIO 重置功能

本次会话新增了服务式 reset 设计。

目标行为：

```text
call reset service
  -> 重置 icp_localization_node 定位状态
  -> 请求重置 FAST-LIO 状态
  -> 清空已有初始位姿
  -> 必须重新手动选择 /initialpose
```

### 6.1 localization_adapter 新增 reset service

修改文件：

```text
src/localization_adapter/src/icp_localization_node.cpp
```

新增参数：

```yaml
reset_service_name: reset_localization
fastlio_reset_service: /fastlio_reset
fastlio_reset_timeout_s: 1.0
```

服务类型：

```text
std_srvs/srv/Trigger
```

调用命令：

```bash
ros2 service call /reset_localization std_srvs/srv/Trigger "{}"
```

调用后需要重新在 RViz 里选择初始位姿：

```text
2D Pose Estimate -> /initialpose
```

定位节点在重新收到 `/initialpose` 前会继续等待，不会继续进行 ICP 定位。

### 6.2 FAST-LIO 新增 reset service

修改文件：

```text
src/FAST_LIO/src/laserMapping.cpp
```

新增服务：

```text
/fastlio_reset
std_srvs/srv/Trigger
```

该服务会重置 FAST-LIO 的关键内部状态：

```text
lidar / imu 输入缓存
IMU 初始化状态
EKF 状态
局部地图 KD-tree
路径缓存
当前点云缓存
时间戳和计数器
```

也可以单独调用 FAST-LIO reset：

```bash
ros2 service call /fastlio_reset std_srvs/srv/Trigger "{}"
```

推荐使用统一入口：

```bash
ros2 service call /reset_localization std_srvs/srv/Trigger "{}"
```

## 7. 构建指令

单独构建全局规划器：

```bash
colcon build --packages-select global_planner
source install/setup.bash
```

构建定位模块：

```bash
colcon build --packages-select localization_adapter
source install/setup.bash
```

构建 FAST-LIO：

```bash
colcon build --packages-select fast_lio
source install/setup.bash
```

如果修改了多个包，建议构建：

```bash
colcon build --packages-select fast_lio localization_adapter global_planner
source install/setup.bash
```

## 8. 推荐运行流程

### 8.1 调试全局规划器

1. 配置 `src/global_planner/config/global_planner.yaml`：

```yaml
debug_mode: true
map_source: file
map_file_path: /absolute/path/to/map.bt
```

2. 构建并启动：

```bash
colcon build --packages-select global_planner
source install/setup.bash
ros2 launch global_planner global_planner.launch.py
```

3. 在 RViz 中：

```text
2D Pose Estimate -> 选起点
2D Goal Pose     -> 选终点
```

4. 查看路径：

```bash
ros2 topic echo /global_path
```

### 8.2 使用定位结果做全局规划起点

1. 配置 `global_planner.yaml`：

```yaml
debug_mode: false
localization_pose_topic: /icp_pose
map_source: file
map_file_path: /absolute/path/to/map.bt
```

2. 启动 FAST-LIO 和 ICP 定位。

3. 在 RViz 中先给定位模块初始位姿：

```text
2D Pose Estimate -> /initialpose
```

4. 启动全局规划器：

```bash
source install/setup.bash
ros2 launch global_planner global_planner.launch.py
```

5. 在 RViz 中选择终点：

```text
2D Goal Pose -> /goal_pose
```

6. 规划器使用 `/icp_pose` 作为起点，发布 `/global_path`。

### 8.3 重置定位和 FAST-LIO

1. 调用统一 reset 服务：

```bash
ros2 service call /reset_localization std_srvs/srv/Trigger "{}"
```

2. 等待服务返回后，在 RViz 中重新选择初始位姿：

```text
2D Pose Estimate -> /initialpose
```

3. 重新选择导航终点：

```text
2D Goal Pose -> /goal_pose
```

## 9. 常用排查命令

查看服务：

```bash
ros2 service list | grep reset
```

查看话题：

```bash
ros2 topic list | grep -E "global_path|occupied_map|icp_pose|initialpose|goal_pose"
```

查看地图可视化点云：

```bash
ros2 topic info /global_planner/occupied_map
```

查看全局规划器日志：

```bash
ros2 launch global_planner global_planner.launch.py
```

如果地图文件加载失败，重点检查：

```text
map_file_path 是否为绝对路径
文件是否为 .bt 或 .ot
如果是 PCD，文件是否为 .pcd 且 pcd_octomap_resolution > 0
当前 shell 是否 source install/setup.bash
```

## 10. 注意事项

- `install/` 目录下的 yaml 和 launch 是构建安装后的副本，源码修改应优先编辑 `src/...` 下的文件。
- 修改 `src/global_planner/config/global_planner.yaml` 后，需要重新 `colcon build --packages-select global_planner` 或手动同步到 `install/`。
- reset 功能涉及 `FAST_LIO` 和 `localization_adapter` 两个包，修改后需要重新构建这两个包。
- 如果只启动 `global_planner`，不会自动启动 FAST-LIO 和 ICP 定位节点。
- `debug_mode: false` 时，全局规划起点依赖 `/icp_pose`；如果没有定位输出，规划器会等待。
