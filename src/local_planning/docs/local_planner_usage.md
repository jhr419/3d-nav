# 3D DWA Local Planner Usage

本包提供 `local_planner_node`，用于把三维全局路径、机器人当前位姿和障碍物模型转换为底盘速度 `/cmd_vel`。

## 运行

单独启动局部规划器：

```bash
colcon build --packages-select local_planning
source install/setup.bash
ros2 launch local_planning local_planner.launch.py
```

同时启动现有 `octo_planner/jie_path_node` 和局部规划器：

```bash
ros2 launch local_planning navigation_3d.launch.py
```

`navigation_3d.launch.py` 不会启动 FAST-LIO 或 ICP，只依赖它们已经发布的 TF、定位和地图话题。

## 订阅话题

| Topic | Type | 说明 |
| --- | --- | --- |
| `/planned_path` | `nav_msgs/msg/Path` | 默认全局路径输入，要求路径点在 `map` 坐标系下包含真实 `x/y/z`。 |
| `/global_path_3d` | `nav_msgs/msg/Path` | 额外路径别名，可通过 `additional_global_path_topic` 关闭或修改。 |
| `/sportmodestate` | `unitree_go/msg/SportModeState` | Unitree 当前运动速度来源，使用 `velocity[0..2]` 和 `yaw_speed`。 |
| `/odom` | `nav_msgs/msg/Odometry` | 可选当前速度来源；没有 `/sportmodestate` 时可用它或 `last_cmd`。 |
| `/octomap` | `octomap_msgs/msg/Octomap` | OctoMap 障碍物来源。 |
| `/cloud_registered_body` | `sensor_msgs/msg/PointCloud2` | 默认实时避障点云来源。FAST-LIO 里该话题的 `frame_id` 常为 `body`，本包默认按 `body:livox_frame` 重映射后再查 TF。 |
| `/livox/lidar` / `/mid360` | `sensor_msgs/msg/PointCloud2` | 默认额外监听的点云来源，由 `additional_pointcloud_topics` 配置。 |
| `/local_obstacle_cloud` | `sensor_msgs/msg/PointCloud2` | 兼容旧参数 `local_obstacle_cloud_topic` 的额外点云来源；如果和 `pointcloud_topic` 不同会同时订阅。 |

## 发布话题

| Topic | Type | 说明 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 地面全向模型使用 `linear.x / linear.y / angular.z`。 |
| `/local_trajectory_marker` | `visualization_msgs/msg/Marker` | 当前最优局部轨迹，`LINE_STRIP`。 |
| `/dwa_candidate_trajectories` | `visualization_msgs/msg/MarkerArray` | 候选轨迹，可用 `publish_candidate_trajectories` 关闭。 |
| `/local_goal_marker` | `visualization_msgs/msg/Marker` | 当前 DWA 局部目标点。 |
| `/recovery_trajectory_marker` | `visualization_msgs/msg/Marker` | DWA 无有效轨迹或卡住时尝试的安全恢复轨迹。 |
| `/path_corridor_marker` | `visualization_msgs/msg/Marker` | 全局路径附近 unknown 放行走廊的半透明提示线。 |
| `/dwa_debug_text` | `std_msgs/msg/String` | 输出候选轨迹统计、碰撞原因、卡住状态和 recovery 状态。 |
| `/dynamic_obstacle_cloud` | `sensor_msgs/msg/PointCloud2` | DWA 实际使用的过滤后动态避障点云，可在 RViz 中直接检查。 |

## 关键参数

参数文件位于：

```text
src/local_planning/config/local_planner.yaml
```

常用参数：

| 参数 | 说明 |
| --- | --- |
| `map_frame` / `base_frame` | TF 查询使用的地图坐标系和机器人坐标系。 |
| `base_frame_candidates` | 逗号分隔候选 base frame，例如 `base_link,odin1_base_link,base_footprint`。 |
| `robot_model` | `ground_omni`、`ground_diff`、`aerial_3d`；当前主要实现 `vx/vy/wz`。 |
| `sport_mode_state_topic` | Unitree 运动状态话题，默认 `/sportmodestate`；如果实际是 `/lf/sportmodestate`，在 YAML 中改这里。 |
| `velocity_source` | DWA 当前速度来源：`sport_mode` 使用 `SportModeState`；`auto` 优先用新鲜 `SportModeState`，其次 `/odom`，最后上一条命令；`odom` 只用 `/odom`；`last_cmd` 用上一条 `/cmd_vel`；`zero` 每周期假设从零速开始。 |
| `obstacle_source` | `octomap`、`pointcloud`、`both`；默认 `both`，同时使用静态 OctoMap 和实时点云。 |
| `pointcloud_topic` / `additional_pointcloud_topics` / `pointcloud_frame_mode` | 实时点云话题和额外候选话题；`auto` 默认转到 `map` 并按当前 `base_frame` 做局部裁剪，也可强制 `map` 或 `base_link`。 |
| `pointcloud_frame_remaps` | 点云消息 frame 到 TF frame 的重映射，默认 `body:livox_frame`，用于 FAST-LIO `/cloud_registered_body`。 |
| `pointcloud_timeout` / `allow_planning_without_fresh_cloud` | 点云超过超时时视为不新鲜；`both` 下允许点云超时后只用 OctoMap 继续规划。 |
| `local_cloud_range_*` | 机器人局部范围裁剪窗口，按 base 坐标过滤点云。 |
| `remove_ground_points` / `ground_z_threshold` / `ground_relative_to_base` | 去除地面和低矮坡面点，避免把脚下地面当障碍。 |
| `ignore_robot_self_points` / `self_filter_*` | 去除机器人自身半径内点云。 |
| `enable_voxel_filter` / `voxel_leaf_size` | 对动态障碍点云做 voxel 降采样。 |
| `dynamic_obstacle_decay_time` | 动态点云缓存过期时间，过期后自动清空点云障碍层。 |
| `dynamic_obstacle_use_2d_footprint` | 默认开启。动态点云先做高度/地面过滤，再按平面 footprint 判断碰撞，避免障碍点因高度没落在机器人 body 薄切片内被放过。 |
| `dynamic_obstacle_radius` / `dynamic_obstacle_safety_margin` | 只用于实时点云动态避障的 footprint；OctoMap 静态碰撞仍使用 `robot_radius` / `safety_margin`。 |
| `weight_dynamic_obstacle_distance` | DWA 评分中的动态障碍距离权重。 |
| `enable_dynamic_speed_scaling` / `dynamic_obstacle_stop_distance` / `dynamic_obstacle_slow_distance` / `min_speed_scale` | 前方动态障碍距离触发的采样速度缩放；进入 stop 距离时正向速度窗口压到 0，保留低速停车/避让空间。 |
| `octomap_file` | 启动时预加载的 `.bt/.ot` 地图文件。 |
| `robot_radius` / `robot_height` / `safety_margin` | 地面机器人碰撞检测圆柱体尺寸。 |
| `collision_z_offset` | 碰撞圆柱体相对 `base_frame` 的底部 z 偏移，用于避开地面占据体素。 |
| `use_path_z_for_collision` / `terrain_following_enabled` | 地面 DWA 仍只输出 `vx/vy/wz`，但仿真点 z 会跟随三维全局路径。 |
| `collision_model` | 默认 `terrain_adaptive_cylinder`，按每个轨迹点的参考 z 检查机器人 body 高度。 |
| `body_z_offset` / `ground_ignore_depth` | body 碰撞检测从参考 z 上方开始，忽略坡面/地面低体素。 |
| `unknown_policy` / `path_corridor_radius` | 默认 `path_corridor_free`，路径走廊内 unknown 不直接当障碍。 |
| `adaptive_lookahead_enabled` | 前方 z 变化大或机器人偏离路径时缩短局部目标 lookahead。 |
| `recovery_enabled` / `stuck_detection_enabled` | 无有效 DWA 轨迹或持续没进展时尝试安全后退、横移或旋转。 |
| `max_vx/min_vx/max_vy/min_vy/max_wz/min_wz` | 速度边界。 |
| `max_acc_vx/max_acc_vy/max_acc_wz` | 动态窗口加速度约束。 |
| `vx_samples/vy_samples/wz_samples` | 候选速度采样数量。 |
| `local_goal_lookahead` | 从最近路径点向前选择局部目标点的距离。 |
| `goal_reached_tolerance` | 到达最终目标后的停车距离阈值。 |
| `weight_*` | 路径距离、目标距离、障碍距离、朝向、速度、平滑度评分权重。 |

## RViz 可视化

在 RViz 中添加：

```text
/planned_path
/planned_path_marker
/local_trajectory_marker
/dwa_candidate_trajectories
/local_goal_marker
/recovery_trajectory_marker
/path_corridor_marker
/dynamic_obstacle_cloud
/octomap_occupied_markers
```

候选轨迹中，绿色半透明线表示无碰撞候选，红色半透明线表示碰撞候选；最优轨迹用青色线显示，recovery 轨迹用橙色线显示。

坡边缘卡住时可以同时观察：

```bash
ros2 topic echo /dwa_debug_text
ros2 topic echo /cmd_vel
```

`/dwa_debug_text` 中的 `unknown_blocked_count`、`ground_blocked_count`、`valid_trajectories`、`dynamic_cloud_fresh`、`dynamic_cloud_points`、`dynamic_obstacle_speed_scale` 和 `recovery_state` 可以快速判断是 unknown 走廊、贴地体素、实时点云还是恢复动作在起作用。

## 与全局规划器的接口约定

局部规划器与全局规划器解耦，只要求全局规划器发布 `nav_msgs/msg/Path`：

1. `header.frame_id` 必须等于 `map_frame`。
2. 每个 `PoseStamped` 的 `position.x/y/z` 必须是真实三维路径点。
3. 路径为空时局部规划器会清空状态并发布零速度。
4. 路径更新后局部规划器会重新缓存路径，并按机器人当前位置修剪已走过的路径点。

当前工程的 `octo_planner/jie_path_node` 默认发布 `/planned_path`，因此 `local_planner.yaml` 默认直接接入它；如果后续全局规划器改为 `/global_path_3d`，节点也已经默认额外监听该话题。

## 扩展到真正 3D 移动体

核心接口已经保留：

```cpp
Velocity3D { vx, vy, vz, wx, wy, wz }
```

当前 `ground_omni` 和 `ground_diff` 只采样 `vx/vy/wz`。扩展 `aerial_3d` 时建议继续在 `DWA3DLocalPlanner` 中增加 `vz/wx/wy` 动态窗口、三维姿态积分和体素/机体包络碰撞检测；ROS 节点层不需要重写，只需要把参数和 `Twist` 发布映射补齐。
