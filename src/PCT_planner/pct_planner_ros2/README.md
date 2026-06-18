# pct_planner_ros2

这是 PCT Planner 的 ROS2 适配包。它不覆盖原来的 ROS1 脚本，而是提供一个可以直接拷进其他 ROS2 workspace 的 `ament_python` 包。

## 包含内容

- `pct_tomography`：读取 `.pcd`，构建 tomogram，发布 `sensor_msgs/PointCloud2`，并导出 `.pickle`
- `pct_map_publisher`：从已有 `.pcd` 和 `.pickle` 读取地图，并发布 RViz2 可视化用的 `/global_points`、`/tomogram`
- `pct_plan`：读取 tomogram，调用原项目 PyBind 规划核心，发布 `nav_msgs/Path`
- `pct_start_goal_marker`：在 RViz2 中提供三维可拖动/可旋转的 start/goal interactive markers，并发布 `geometry_msgs/PoseStamped`
- `launch/tomography.launch.py`
- `launch/planner.launch.py`
- `launch/start_goal_marker.launch.py`
- `rviz/pct_ros2.rviz`

## 依赖

ROS2 侧：

```bash
sudo apt install ros-$ROS_DISTRO-rclpy \
  ros-$ROS_DISTRO-interactive-markers \
  ros-$ROS_DISTRO-sensor-msgs-py \
  ros-$ROS_DISTRO-nav-msgs \
  ros-$ROS_DISTRO-geometry-msgs \
  ros-$ROS_DISTRO-visualization-msgs \
  ros-$ROS_DISTRO-rviz2
```

Python/算法侧：

```bash
python3 -m pip install numpy open3d
# 按你的 CUDA 版本安装 cupy，例如 CUDA 11.x:
python3 -m pip install cupy-cuda11x
```

`open3d` 只是优先使用的 PCD 读取器；如果它因为本机 NumPy/Sklearn 版本冲突导入失败，节点会自动回退到包内 PCD 读取器。

## 构建 ROS2 包

```bash
mkdir -p ~/ros2_ws/src
cp -r src/PCT_planner/pct_planner_ros2 <your_ros2_ws>/src/
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --packages-select pct_planner_ros2
source install/setup.bash
```

## 数据目录

默认目录：

```bash
mkdir -p ~/.ros/pct_planner/pcd
mkdir -p ~/.ros/pct_planner/tomogram
```

把点云放到：

```bash
~/.ros/pct_planner/pcd/
```

当前仓库自带 zip 里有 `building2_9.pcd` 和 `plaza3_10.pcd`：

```bash
unzip src/PCT_planner/rsc/pcd/pcd_files.zip -d maps/
```

`Spiral` 需要额外下载 `spiral0.3_2.pcd` 并放到同一目录。

## 运行 Tomography

Tomography 默认从 `config/tomography.yaml` 读取参数。这个 YAML 里包含 PCD 预处理、输出目录、地图几何和 traversability 代价参数。当前默认流程会先把原始点云拉平/对齐到 `full_map_leveled.pcd`，然后立刻用它生成 tomogram：

```bash
ros2 launch pct_planner_ros2 tomography.launch.py rviz:=true
```

也可以指定自己的 YAML：

```bash
ros2 launch pct_planner_ros2 tomography.launch.py \
  tomography_config:=/path/to/tomography.yaml \
  rviz:=true
```

当前默认 YAML 已配置为：

```text
enable_preprocess: true
preprocess_input_pcd: maps/map_origin.pcd
preprocess_output_pcd: maps/map_preprocessed.pcd
pcd_dir: maps
pcd_file: full_map_leveled.pcd
tomogram_dir: maps/tomogram
```

YAML 参数也可以被命令行非空参数覆盖：

```bash
ros2 launch pct_planner_ros2 tomography.launch.py \
  preprocess_input_pcd:=/path/to/raw_map.pcd \
  preprocess_output_pcd:=/path/to/leveled_map.pcd \
  pcd_file:=leveled_map.pcd \
  resolution:=0.15 \
  safe_margin:=0.5 \
  rviz:=true
```

完整可调参数包括：

```text
scene
pcd_dir
pcd_file
tomogram_dir
benchmark_repeats
map_frame
pointcloud_topic
layer_g_topic_prefix
layer_c_topic_prefix
tomogram_topic
resolution
ground_h
slice_dh
kernel_size
interval_min
interval_free
slope_max
step_max
standable_ratio
cost_barrier
safe_margin
inflation
auto_run
enable_preprocess
preprocess_input_pcd
preprocess_output_pcd
preprocess_overwrite
enable_manual_transform
roll
pitch
yaw
tx
ty
tz
enable_auto_level
ground_percentile
max_ground_points
ransac_distance_threshold
ransac_n
ransac_num_iterations
normal_target_axis
enable_ground_z_shift
target_ground_z
ground_z_shift_mode
ground_z_shift_percentile
random_seed
```

生成文件会写到：

```bash
~/.ros/pct_planner/tomogram/<pcd_file_stem>.pickle
```

## 运行 Planner

先构建原项目的 C++/PyBind 规划核心：

```bash
cd src/PCT_planner/planner
./build_thirdparty.sh
./build.sh
```

然后运行 ROS2 planner。`planner.launch.py` 默认从 `config/planner.yaml` 读取参数，这个 YAML 同时控制 planner、地图发布和三维选点工具：

```bash
ros2 launch pct_planner_ros2 planner.launch.py
```

也可以指定自己的 YAML：

```bash
ros2 launch pct_planner_ros2 planner.launch.py \
  planner_config:=/path/to/planner.yaml
```

`planner.yaml` 的主要段落：

```text
planner_launch        是否启动地图、三维选点工具、RViz2
pct_map_publisher    PCD/tomogram 文件、frame、点云发布 topic
pct_start_goal_marker 三维 start/goal marker 的 topic、默认位置、交互发布方式
pct_planner          tomogram、planner_lib_dir、路径 topic、自动规划行为
```

当前默认配置会同时启动 `pct_map_publisher`、`pct_start_goal_marker`、`pct_plan` 和 RViz2，并发布：

```text
/global_points
/tomogram
/pct_planner/start_pose
/pct_planner/goal_pose
/pct_path
```

如果 RViz2 里没有看到点云，先确认地图文件存在：

```bash
ls maps/map_preprocessed.pcd
ls maps/tomogram/map_preprocessed.pickle
```

YAML 参数也可以被命令行非空参数覆盖，适合临时试验：

```bash
ros2 launch pct_planner_ros2 planner.launch.py \
  pcd_file:=full_map_ground.pcd \
  tomogram_file:=full_map_ground \
  publish_on_feedback:=false \
  show_rviz:=true
```

如果只想跑 planner，不打开地图或 RViz2：

```bash
ros2 launch pct_planner_ros2 planner.launch.py \
  publish_map:=false \
  use_interactive_start_goal:=false \
  show_rviz:=false
```

`rviz:=true/false` 仍可作为旧命令的兼容别名；新配置里优先使用 `show_rviz`。

如果需要用固定二维参数起终点，而不是 RViz2 三维 marker：

```bash
ros2 launch pct_planner_ros2 planner.launch.py \
  use_interactive_start_goal:=false \
  auto_run:=true \
  start_x:=5.0 start_y:=5.0 \
  goal_x:=-6.0 goal_y:=-1.0
```

## 三维交互式起终点

单独启动 start/goal interactive marker：

```bash
ros2 launch pct_planner_ros2 start_goal_marker.launch.py
```

启动后 RViz2 的 fixed frame 应为 `map`，并显示 `InteractiveMarkers`。绿色小球是 start，红色小球是 goal。可以沿 x/y/z 三个方向拖动，也可以绕 roll/pitch/yaw 三轴旋转。右键菜单提供：

- `Set as Start`
- `Set as Goal`
- `Publish Start`
- `Publish Goal`
- `Publish Both`

默认每次 marker 位姿变化都会发布：

```text
/pct_planner/start_pose  geometry_msgs/msg/PoseStamped
/pct_planner/goal_pose   geometry_msgs/msg/PoseStamped
```

如果 RViz2 里看不到可拖拽小球，确认显示项：

```text
PCT Start/Goal Markers
Class = rviz_default_plugins/InteractiveMarkers
Interactive Markers Namespace = pct_start_goal_marker
```

并确认工具栏选中 `Interact`。同时可以用普通 MarkerArray 兜底显示检查小球位置：

```text
Start/Goal Preview
Topic = /pct_planner/start_goal_markers
```

检查 interactive marker server：

```bash
ros2 topic list -t | grep pct_start_goal_marker
ros2 service list | grep pct_start_goal_marker
```

验证 topic：

```bash
ros2 topic echo /pct_planner/start_pose
ros2 topic echo /pct_planner/goal_pose
```

单独启动时默认参数在 `config/start_goal_marker.yaml`。和 planner 一起启动时，三维选点工具参数来自 `config/planner.yaml` 的 `pct_start_goal_marker` 段。

单独启动也可以覆盖：

```bash
ros2 launch pct_planner_ros2 start_goal_marker.launch.py \
  frame_id:=map \
  marker_config:=/path/to/start_goal_marker.yaml
```

和 planner 一起启动时直接使用 `planner.launch.py`，参数来自 `config/planner.yaml`。

`pct_plan` 会订阅 `/pct_planner/start_pose` 和 `/pct_planner/goal_pose`，收到二者后按 PoseStamped 的 x/y 转成 tomogram 网格坐标，并用 z 选择最接近的可通行 tomogram 层，然后重新规划并发布 `/pct_path`。姿态会保留在 PoseStamped topic 中，当前 PCT Planner 核心仍不使用起终点朝向参与 A*。

## 迁移到其他项目

最小迁移内容：

1. 复制 `pct_planner_ros2/` 到目标 ROS2 workspace 的 `src/`
2. 构建并准备原 PCT Planner 的 `planner/lib` PyBind 核心
3. 运行时传入 `planner_lib_dir:=/path/to/planner/lib`
4. 把 `.pcd` 放到 `pcd_dir`，把 `.pickle` 输出到 `tomogram_dir`

如果目标项目已经有自己的 launch，可以直接启动这两个可执行入口：

```bash
ros2 run pct_planner_ros2 pct_tomography --ros-args \
  -p scene:=Building \
  -p pcd_dir:=/path/to/pcd \
  -p tomogram_dir:=/path/to/tomogram

ros2 run pct_planner_ros2 pct_plan --ros-args \
  -p scene:=Building \
  -p tomogram_dir:=/path/to/tomogram \
  -p planner_lib_dir:=/path/to/planner/lib
```
