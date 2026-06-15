# sc_lio_sam_backend

独立 ROS 2 Humble 后端包，用 FAST-LIO2 的 odometry 和点云输出生成关键帧，使用 Scan Context 检索回环，使用 PCL ICP 做几何验证，并用 GTSAM ISAM2 做位姿图优化。

## 与 FAST_LIO 的接口

FAST_LIO 当前实际发布：

- `/Odometry`：`nav_msgs/msg/Odometry`，`header.frame_id=camera_init`，`child_frame_id=body`
- `/cloud_registered`：`sensor_msgs/msg/PointCloud2`，`frame_id=camera_init`
- `/cloud_registered_body`：`sensor_msgs/msg/PointCloud2`，`frame_id=body`
- `/cloud_effected`、`/Laser_map`、`/path`
- TF：`camera_init -> body`

后端默认订阅 `/Odometry` 和 `/cloud_registered_body`。如果改成 `/cloud_registered`，`cloud_frame_mode=auto` 会根据点云 frame 与 odom frame 一致这一事实，将世界系点云用当前 odom 逆变换转回关键帧局部系。

## TF 设计

FAST_LIO 继续发布 `camera_init -> body`，后端默认只发布 `map -> camera_init` 修正：

- `map_frame: map`
- `odom_frame: camera_init`
- `base_frame: body`
- `publish_tf: true`
- `publish_map_to_odom: true`

这样不会和 FAST_LIO 抢 `camera_init -> body`。如果你的前端使用标准 `odom -> base_link`，把 `odom_frame/base_frame/lidar_frame` 改成对应名称即可。

## 编译

```bash
cd /home/jhr/3dnav_ws
colcon build --symlink-install --packages-select sc_lio_sam_backend
source install/setup.bash
```

如需同时编译前端：

```bash
colcon build --symlink-install --packages-select fast_lio sc_lio_sam_backend
source install/setup.bash
```

## 运行

只启动后端：

```bash
ros2 launch sc_lio_sam_backend sc_lio_sam_backend.launch.py
```

组合启动 FAST_LIO 和后端：

```bash
ros2 launch sc_lio_sam_backend fastlio_with_sc_backend.launch.py
```

保存优化地图：

```bash
ros2 service call /sc_lio_sam_backend/save_map std_srvs/srv/Trigger {}
```

## 主要参数

- `odom_topic`、`cloud_topic`：前端输出接口
- `keyframe_distance_threshold`、`keyframe_angle_threshold_deg`：关键帧阈值
- `scan_context_ring_num`、`scan_context_sector_num`、`scan_context_max_radius`：Scan Context 分辨率和范围
- `loop_search_radius`、`loop_search_time_diff_threshold`、`scan_context_distance_threshold`：回环候选过滤
- `loop_fitness_score_threshold`、`icp_max_correspondence_distance`：ICP 验证阈值
- `prior_noise_sigmas`、`odom_noise_sigmas`、`loop_noise_sigmas`：GTSAM 约束噪声
- `publish_tf`、`publish_map_to_odom`：后端 TF 发布策略

## 输出

- `/sc_lio_sam_backend/optimized_path`
- `/sc_lio_sam_backend/optimized_odometry`
- `/sc_lio_sam_backend/loop_markers`
- `/sc_lio_sam_backend/optimized_map`
- `/sc_lio_sam_backend/save_map`

## 说明和限制

这个包没有修改 FAST_LIO，也不依赖 FAST_LIO 内部类。当前后端使用前端 odom 相邻关键帧作为 odometry factor，使用 Scan Context + ICP 生成 loop factor。大场景中如果漂移很大，`loop_search_radius` 可能会过滤掉真实回环，可以适当增大，或者设为 `0.0` 关闭空间半径过滤。

当前机器的 `/usr/local` 下同时存在两个 `libgtsam.so.4` ABI。为了不改系统库，CMake 会在本包可执行旁安装 `gtsam_abi/libgtsam.so.4` symlink，并通过 ament environment hook 将该目录放到 `LD_LIBRARY_PATH` 前面。
