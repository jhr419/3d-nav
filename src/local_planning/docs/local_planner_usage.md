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
| `/local_obstacle_cloud` | `sensor_msgs/msg/PointCloud2` | 局部点云障碍物来源，节点会转换到 `map` 坐标系。 |

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
| `obstacle_source` | `octomap`、`pointcloud`、`both`。 |
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
/octomap_occupied_markers
```

候选轨迹中，绿色半透明线表示无碰撞候选，红色半透明线表示碰撞候选；最优轨迹用青色线显示，recovery 轨迹用橙色线显示。

坡边缘卡住时可以同时观察：

```bash
ros2 topic echo /dwa_debug_text
ros2 topic echo /cmd_vel
```

`/dwa_debug_text` 中的 `unknown_blocked_count`、`ground_blocked_count`、`valid_trajectories` 和 `recovery_state` 可以快速判断是 unknown 走廊、贴地体素还是恢复动作在起作用。

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
