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
cp -r /home/jhr/workspace/PCT_planner/pct_planner_ros2 ~/ros2_ws/src/
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --packages-select pct_planner_ros2
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
unzip /home/jhr/workspace/PCT_planner/rsc/pcd/pcd_files.zip -d ~/.ros/pct_planner/pcd/
```

`Spiral` 需要额外下载 `spiral0.3_2.pcd` 并放到同一目录。

## 运行 Tomography

```bash
ros2 launch pct_planner_ros2 tomography.launch.py scene:=Building rviz:=true
```

也可以显式指定目录：

```bash
ros2 launch pct_planner_ros2 tomography.launch.py \
  scene:=Building \
  pcd_dir:=/path/to/pcd \
  tomogram_dir:=/path/to/tomogram \
  rviz:=true
```

生成文件会写到：

```bash
~/.ros/pct_planner/tomogram/building2_9.pickle
```

## 运行 Planner

先构建原项目的 C++/PyBind 规划核心：

```bash
cd /home/jhr/workspace/PCT_planner/planner
./build_thirdparty.sh
./build.sh
```

然后运行 ROS2 planner。推荐显式传入 `planner_lib_dir`：

```bash
ros2 launch pct_planner_ros2 planner.launch.py \
  scene:=Building \
  planner_lib_dir:=/home/jhr/workspace/PCT_planner/planner/lib \
  rviz:=true
```

`planner.launch.py` 默认会同时启动 `pct_map_publisher`，从 `~/.ros/pct_planner/pcd` 和 `~/.ros/pct_planner/tomogram` 发布地图：

```text
/global_points
/tomogram
```

如果 RViz2 里没有看到点云，先确认地图文件存在：

```bash
ls ~/.ros/pct_planner/pcd
ls ~/.ros/pct_planner/tomogram
```

也可以显式传入目录或关闭地图发布：

```bash
ros2 launch pct_planner_ros2 planner.launch.py \
  scene:=Building \
  pcd_dir:=/path/to/pcd \
  tomogram_dir:=/path/to/tomogram \
  publish_map:=true
```

也可以用环境变量：

```bash
export PCT_PLANNER_LIB_DIR=/home/jhr/workspace/PCT_planner/planner/lib
ros2 launch pct_planner_ros2 planner.launch.py scene:=Building rviz:=true
```

自定义起终点：

```bash
ros2 launch pct_planner_ros2 planner.launch.py \
  scene:=Building \
  start_x:=5.0 start_y:=5.0 \
  goal_x:=-6.0 goal_y:=-1.0 \
  planner_lib_dir:=/home/jhr/workspace/PCT_planner/planner/lib
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

默认参数在 `config/start_goal_marker.yaml`，也可以启动时覆盖：

```bash
ros2 launch pct_planner_ros2 start_goal_marker.launch.py \
  frame_id:=map \
  marker_config:=/path/to/start_goal_marker.yaml
```

和 planner 一起启动：

```bash
ros2 launch pct_planner_ros2 planner.launch.py \
  scene:=Building \
  planner_lib_dir:=/home/jhr/workspace/PCT_planner/planner/lib \
  use_interactive_start_goal:=true \
  show_rviz:=true
```

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
