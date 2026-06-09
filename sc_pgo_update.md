# SC-PGO ROS2 版本更新说明

## 1. 新增包位置

本次新增 ROS2 包：

```bash
/home/jhr/3dnav_ws/src/mapping_localization/sc_pgo
```

该包用于将原 ROS1 Melodic + GTSAM 4.x 的 SC-PGO 后端闭环建图模块迁移到 ROS2，并适配当前工作区已有的 FAST-LIO2 输出。

## 2. 主要功能

`sc_pgo` 节点实现了以下功能：

- 订阅 FAST-LIO2 输出的里程计和点云。
- 按平移/旋转阈值选取关键帧。
- 为关键帧生成 Scan Context 描述子。
- 基于 Scan Context 检测回环候选。
- 使用 ICP 对回环候选进行几何验证。
- 使用 GTSAM iSAM2 优化位姿图。
- 发布优化后的位姿、轨迹和全局点云地图。
- 保存关键帧点云、原始里程计轨迹和优化后轨迹。

## 3. 文件结构

主要文件如下：

```text
src/mapping_localization/sc_pgo/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── fastlio_sc_pgo.yaml
├── launch/
│   ├── sc_pgo.launch.py
│   └── fastlio_sc_pgo.launch.py
├── scripts/
│   └── sc_pgo_node
├── include/sc_pgo/
│   ├── common.hpp
│   ├── tic_toc.hpp
│   └── scan_context/
│       ├── scan_context.hpp
│       ├── kdtree_vector_of_vectors_adaptor.hpp
│       └── nanoflann.hpp
└── src/
    ├── sc_pgo_node.cpp
    └── scan_context.cpp
```

## 4. FAST-LIO2 适配关系

默认输入话题：

```yaml
odom_topic: "/Odometry"
cloud_topic: "/cloud_registered_body"
scan_context_cloud_topic: ""
```

含义：

- `/Odometry`：FAST-LIO2 发布的里程计，坐标关系通常为 `camera_init -> body`。
- `/cloud_registered_body`：FAST-LIO2 发布的当前帧 body 坐标系点云。
- `scan_context_cloud_topic` 为空时，Scan Context 也复用 `/cloud_registered_body`。

如果后续 FAST-LIO2 增加了雷达自身坐标系下的点云话题，可以把 `scan_context_cloud_topic` 改成该话题。节点内部会将建图/ICP 使用的点云和 Scan Context 使用的点云分开保存，避免互相影响。

## 5. 默认输出话题

`sc_pgo_node` 发布以下话题：

```text
/aft_pgo_odom
/aft_pgo_path
/aft_pgo_map
/loop_scan_local
/loop_submap_local
```

说明：

- `/aft_pgo_odom`：优化后的最新关键帧位姿。
- `/aft_pgo_path`：优化后的关键帧轨迹。
- `/aft_pgo_map`：优化后关键帧点云拼接得到的地图。
- `/loop_scan_local`：回环检测中当前关键帧局部点云，用于调试。
- `/loop_submap_local`：回环检测中历史局部子图，用于调试。

## 6. 编译方法

在工作区根目录执行：

```bash
cd /home/jhr/3dnav_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sc_pgo
source install/setup.bash
```

## 7. 启动方法

如果 FAST-LIO2 已经单独启动，可以只启动 SC-PGO：

```bash
ros2 launch sc_pgo sc_pgo.launch.py
```

如果希望同时启动 FAST-LIO2 和 SC-PGO：

```bash
ros2 launch sc_pgo fastlio_sc_pgo.launch.py
```

可以指定保存目录：

```bash
ros2 launch sc_pgo sc_pgo.launch.py save_directory:=maps/sc_pgo
```

## 8. 参数文件

参数文件位置：

```bash
src/mapping_localization/sc_pgo/config/fastlio_sc_pgo.yaml
```

常用参数：

```yaml
keyframe_meter_gap: 0.5
keyframe_deg_gap: 10.0
sc_dist_thres: 0.3
sc_max_radius: 80.0
save_directory: "maps/sc_pgo"
```

参数说明：

- `keyframe_meter_gap`：关键帧平移间隔，单位 m。
- `keyframe_deg_gap`：关键帧旋转间隔，单位 deg。
- `keyframe_min_points`：少于该点数的点云帧不会作为关键帧。
- `scan_context_leaf_size`：生成 Scan Context 前的体素降采样尺寸。
- `map_visualization_leaf_size`：发布 `/aft_pgo_map` 前的体素降采样尺寸。
- `sc_dist_thres`：Scan Context 回环距离阈值。
- `sc_max_radius`：Scan Context 最大检测半径，室外可用 80 m，室内可适当降低。
- `sc_num_exclude_recent`：回环检测时排除最近若干关键帧，避免把邻近帧误认为回环。
- `enable_loop_closure`：是否启用回环检测。
- `loop_icp_fitness_score_threshold`：ICP 验证回环的 fitness 阈值。
- `save_directory`：轨迹和关键帧点云保存目录。

## 9. 保存结果

默认保存到：

```bash
maps/sc_pgo
```

保存内容：

```text
maps/sc_pgo/
├── optimized_poses.txt
├── odom_poses.txt
├── times.txt
└── Scans/
    ├── 000000.pcd
    ├── 000001.pcd
    └── ...
```

说明：

- `optimized_poses.txt`：GTSAM 优化后的关键帧轨迹，KITTI 格式。
- `odom_poses.txt`：FAST-LIO2 原始关键帧里程计轨迹，KITTI 格式。
- `times.txt`：关键帧时间戳。
- `Scans/`：保存的关键帧点云。

## 10. GTSAM 运行时问题处理

本机环境中存在一个 GTSAM 版本不一致问题：

- GTSAM 头文件对应的是 4.2 ABI。
- `/usr/local/lib/libgtsam.so.4` 当前软链接指向 4.3a1。
- 直接运行时会出现如下错误：

```text
undefined symbol: _ZNK5gtsam5Pose37inverseEv
```

为避免修改系统全局 `/usr/local/lib`，当前包采用局部启动脚本处理：

```text
src/mapping_localization/sc_pgo/scripts/sc_pgo_node
```

脚本会在启动前将包内库目录放到 `LD_LIBRARY_PATH` 最前面：

```bash
export LD_LIBRARY_PATH="${script_dir}:${LD_LIBRARY_PATH:-}"
exec "${script_dir}/sc_pgo_node_bin" "$@"
```

因此 `ros2 launch sc_pgo sc_pgo.launch.py` 会优先加载：

```text
install/sc_pgo/lib/sc_pgo/libgtsam.so.4 -> libgtsam.so.4.2.0
```

这样可以避免误加载 `/usr/local/lib/libgtsam.so.4.3a1`。

## 11. 已验证内容

已经验证：

```bash
colcon build --packages-select sc_pgo
```

编译通过。

也已经验证：

```bash
ros2 launch sc_pgo sc_pgo.launch.py
```

节点可以正常启动，并输出：

```text
SC-PGO ROS2 node ready: odom=/Odometry cloud=/cloud_registered_body sc_cloud=/cloud_registered_body save_dir=maps/sc_pgo/
```

## 12. 后续建议

建议后续使用真实 rosbag 或在线 FAST-LIO2 数据流测试：

- `/Odometry` 是否持续发布。
- `/cloud_registered_body` 是否持续发布且点数足够。
- `/aft_pgo_path` 是否随关键帧增长。
- `/aft_pgo_map` 是否周期性发布。
- 回到已走过区域时 `/loop_scan_local` 和 `/loop_submap_local` 是否出现。
- `optimized_poses.txt` 是否在回环后产生明显位姿修正。

