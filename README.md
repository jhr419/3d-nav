# ROS2 Humble 三维导航系统

本仓库是基于 ROS2 Humble 的三维导航工作区，面向 Livox MID360 + FAST-LIO 建图定位、PCT Tomography 地图处理、A* 全局规划、EGO 局部规划、Go2 底盘速度桥接和导航执行控制。

系统目标是把建图、地图预处理、定位、全局规划、局部规划、机器人接口和执行控制解耦，并统一通过 `3dnav_bringup` 启动。

> 说明：ROS2/ament 的实际包名不能以数字开头，因此可构建包名使用 `nav3d_*`。仓库同时安装了 `3dnav_*` 启动别名，日常仍可按本文命令使用 `ros2 launch 3dnav_bringup ...`。

## 目录结构

```text
3dnav_ws
├── maps
│   ├── map_origin.pcd
│   ├── map_preprocessed.pcd
│   ├── map_visualization.pcd
│   └── tomogram
│       └── map_preprocessed.pickle
├── src
│   ├── mapping
│   │   ├── FAST_LIO
│   │   ├── livox_ros_driver2
│   │   ├── mapping_adapter
│   │   └── 3dnav_mapping
│   ├── map_process
│   │   ├── PCT_planner
│   │   ├── map_mannger
│   │   └── 3dnav_map_process
│   ├── localization
│   │   ├── localization_adapter
│   │   └── 3dnav_localization
│   ├── global_planning
│   │   ├── pct_global_planner
│   │   ├── jie_3d_nav
│   │   └── 3dnav_global_planning
│   ├── local_planning
│   │   ├── ego_local_planner
│   │   └── 3dnav_local_planning
│   ├── robot_api
│   │   ├── unitree_ros2
│   │   ├── go2_driver
│   │   ├── go2_twist_bridge
│   │   ├── go2_teleop_ctrl_keyboard_py
│   │   └── 3dnav_robot_api
│   ├── 3dnav_common
│   ├── 3dnav_control
│   ├── 3dnav_rviz_plugins
│   └── 3dnav_bringup
└── README.md
```

历史地图资料已归档到 `maps/src_maps/`，运行时标准地图统一放在根目录 `maps/`。

## 核心模块

| 模块 | 实际包/目录 | 作用 |
| --- | --- | --- |
| 统一启动 | `nav3d_bringup` / `src/3dnav_bringup` | 统一 launch、系统路径和模块开关 |
| 建图 | `fast_lio`、`mapping_adapter` | FAST-LIO 建图、地图适配输出 |
| 地图处理 | `pct_planner_ros2`、`pcd_preprocessor_ros2` | PCD 预处理、tomogram 生成 |
| 定位 | `localization_adapter` | FAST-LIO 里程计 + ICP 地图定位 |
| 全局规划 | `nav3d_global_planning`、`pct_global_planner`、`octo_planner` | 默认 A* 全局规划，可切换 PCT / OctoMap 后端 |
| 局部规划 | `ego_local_planner` | 跟踪 `/planned_path` 并输出 `/cmd_vel_nav` |
| 执行控制 | `nav3d_control` | Start/Stop/Pause/Resume/Clear Path 状态机和速度门控 |
| 机器人接口 | `go2_twist_bridge`、`unitree_ros2` | `/cmd_vel` 到 Unitree Go2 Sport API |

## 依赖项

### 系统环境

- Ubuntu 22.04
- ROS2 Humble
- Python 3.10
- CMake、colcon、rosdep
- PCL、Eigen、Boost、FLANN
- Livox SDK2
- Unitree ROS2 消息和接口包

### 推荐 apt 依赖

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git python3-pip python3-rosdep \
  python3-colcon-common-extensions python3-vcstool \
  libeigen3-dev libboost-all-dev libpcl-dev libflann-dev libomp-dev \
  ros-humble-desktop \
  ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs \
  ros-humble-rosbag2-cpp ros-humble-rosbag2-storage \
  ros-humble-interactive-markers ros-humble-visualization-msgs
```

首次使用 rosdep：

```bash
sudo rosdep init 2>/dev/null || true
rosdep update
```

Python 依赖：

```bash
python3 -m pip install --user numpy scipy pyyaml open3d
```

Livox SDK2 需要提供：

```text
/usr/local/lib/liblivox_lidar_sdk_shared.so
livox_lidar_api.h
livox_lidar_def.h
```

如果系统里没有 Livox SDK2，请先按 Livox 官方 SDK2 文档安装，再构建本工作区。

### PCT Planner 核心库

PCT Planner 的 Python ROS2 wrapper 会调用 `src/map_process/PCT_planner/planner/lib` 下的核心库。如果该库缺失或不可用，先构建 PCT 原始核心：

```bash
cd ~/3dnav_ws/src/map_process/PCT_planner/planner
bash build_thirdparty.sh
bash build.sh
```

然后回到工作区根目录继续 colcon 构建。

## 构建方式

进入工作区：

```bash
cd ~/3dnav_ws
source /opt/ros/humble/setup.bash
```

安装 ROS 依赖：

```bash
rosdep install --from-paths src --ignore-src -r -y
```

完整构建：

```bash
colcon build --symlink-install
source install/setup.bash
```

建议加入 shell 启动脚本：

```bash
echo "source ~/3dnav_ws/install/setup.bash" >> ~/.bashrc
```

如果移动过源码目录或包路径，建议清理缓存后重建：

```bash
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

也可以分组构建：

```bash
colcon build --symlink-install --packages-select \
  livox_ros_driver2 fast_lio mapping_adapter localization_adapter \
  pct_planner_ros2 pcd_preprocessor_ros2 pct_global_planner \
  ego_local_planner nav3d_control nav3d_bringup
```

## 地图文件规范

所有运行地图统一放在根目录 `maps/`。

| 文件 | 用途 |
| --- | --- |
| `maps/map_origin.pcd` | 建图结果导出，地图预处理输入 |
| `maps/map_preprocessed.pcd` | 定位、ICP、全局规划使用的主地图 |
| `maps/map_visualization.pcd` | RViz 可视化地图，可选 |
| `maps/tomogram/map_preprocessed.pickle` | PCT Planner 使用的 tomogram |

如果 `maps/map_visualization.pcd` 不存在，定位 launch 会输出 warning，并回退使用 `maps/map_preprocessed.pcd` 作为可视化地图。

## 启动流程

### 0. 快速启动当前默认系统

进入工作区并加载环境：

```bash
cd ~/3dnav_ws
source install/setup.bash
```

在线导航一条命令启动当前默认链路：

```bash
ros2 launch 3dnav_bringup navigation.launch
```

默认启动内容：

```text
FAST-LIO / Livox
Localization
A* Global Planner
EGO Local Planner
nav_execution_controller_node
cmd_vel_gate_node
go2_twist_bridge
Localization RViz2
Planning RViz2
```

离线测试一条命令启动，只检查规划和 RViz，不接雷达、不接机器人、不等待实时点云：

```bash
ros2 launch 3dnav_bringup navigation.launch offline_test:=true
```

也可以使用总入口启动当前默认导航系统：

```bash
ros2 launch 3dnav_bringup full_system.launch mapping:=false navigation:=true rviz:=true
```

该总入口默认会打开两个 RViz 窗口：`localization_rviz2` 用于定位显示，`planning_rviz2` 用于路径规划显示。路径规划窗口复用 A* 离线测试同一套 PCT map/tomogram/goal marker RViz 配置。

总入口也支持离线测试开关：

```bash
ros2 launch 3dnav_bringup full_system.launch offline_test:=true
```

### 1. FAST-LIO 建图

在线建图：

```bash
ros2 launch 3dnav_bringup mapping.launch
```

常用参数：

```bash
ros2 launch 3dnav_bringup mapping.launch \
  start_livox_driver:=true \
  start_fastlio:=true \
  start_rviz:=true
```

FAST-LIO 支持直接读取 ROS2 bag 离线建图：

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml \
  rosbag_enable:=true \
  rosbag_path:=/path/to/bag \
  rviz:=false
```

建图完成后，将导出的原始地图保存为：

```text
maps/map_origin.pcd
```

### 2. 地图预处理与 tomogram 生成

输入：

```text
maps/map_origin.pcd
```

启动：

```bash
ros2 launch 3dnav_bringup map_preprocess.launch
```

输出：

```text
maps/map_preprocessed.pcd
maps/tomogram/map_preprocessed.pickle
```

检查：

```bash
ls maps
ls maps/tomogram
```

### 3. 地图规划可行性测试

```bash
ros2 launch 3dnav_bringup map_planner_test.launch
```

该流程用于验证 `map_preprocessed.pcd` 和 `map_preprocessed.pickle` 是否可被地图处理/规划链路正常加载。日常导航默认使用后续 A* 全局规划入口。

### 4. 定位

```bash
ros2 launch 3dnav_bringup localization.launch
```

默认算法：

```text
FAST-LIO + ICP
```

默认地图：

```text
map_pcd_path: maps/map_preprocessed.pcd
icp_map_pcd_path: maps/map_preprocessed.pcd
visualization_map_pcd_path: maps/map_visualization.pcd
```

检查定位输出：

```bash
ros2 topic list | grep -E "pose|icp"
ros2 topic echo /icp_pose
```

### 5. 全局规划

```bash
ros2 launch 3dnav_bringup global_planning.launch
```

默认算法：

```text
A* Global Planner
```

默认地图输入：

```text
maps/map_preprocessed.pcd
maps/tomogram/map_preprocessed.pickle
```

主要输出：

```text
/planned_path
/path
/planned_path_marker
/astar_global_planner/status
```

临时切换其它全局规划算法：

```bash
ros2 launch 3dnav_bringup global_planning.launch algorithm:=pct
ros2 launch 3dnav_bringup global_planning.launch algorithm:=jie_octomap
ros2 launch 3dnav_bringup global_planning.launch algorithm:=astar
```

单独测试全局规划且没有定位 TF 时：

```bash
ros2 launch 3dnav_bringup global_planning.launch offline_test:=true
```

`offline_test:=true` 会发布默认 `map -> base_link` 静态起点 TF，可用 `offline_start_x/y/z/yaw` 覆盖。

### 6. 局部规划

```bash
ros2 launch 3dnav_bringup local_planning.launch
```

默认算法：

```text
EGO Planner
```

输入：

```text
/planned_path
```

输出：

```text
/cmd_vel_nav
```

离线只检查局部规划链路、不等待实时点云：

```bash
ros2 launch 3dnav_bringup local_planning.launch require_fresh_cloud:=false
```

### 7. 机器人接口

```bash
ros2 launch 3dnav_bringup robot_api.launch
```

启动内容：

```text
nav_execution_controller_node
cmd_vel_gate_node
go2_twist_bridge
```

速度链路：

```text
EGO Planner
  -> /cmd_vel_nav
  -> cmd_vel_gate
  -> /cmd_vel
  -> go2_twist_bridge
  -> Go2
```

### 8. 在线导航

```bash
ros2 launch 3dnav_bringup navigation.launch
```

默认启动：

```text
Localization
A* Global Planner
EGO Local Planner
nav_execution_controller_node
cmd_vel_gate_node
go2_twist_bridge
Localization RViz2
Planning RViz2
```

`rviz:=true` 时会打开两个窗口：

```text
localization_rviz2: 定位、ICP、地图/点云显示
planning_rviz2: A* 路径规划、/global_points、/tomogram、起终点 marker、/planned_path
```

设置初始位姿后，在 `planning_rviz2` 中选择顶部工具栏的 `Interact`，拖动终点球旁边的坐标轴即可更新目标位置；需要上楼梯或跨楼层时，用蓝色 Z 轴控制高度。

如果只想看算法链路，不接管机器人速度桥：

```bash
ros2 launch 3dnav_bringup navigation.launch robot_api:=false
```

如果只做离线规划/RViz 检查，不启动定位、雷达和机器人接口：

```bash
ros2 launch 3dnav_bringup navigation.launch offline_test:=true
```

`offline_test:=true` 会自动覆盖为：

```text
localization:=false
robot_api:=false
start_livox_driver:=false
start_fastlio:=false
require_fresh_cloud:=false
```

离线模式还会发布一个静态 `map -> base_link`，供 A* 和 EGO local 获取当前机器人位姿。默认起点为：

```text
offline_start_x: -1.5
offline_start_y: 3.2
offline_start_z: 0.0
offline_start_yaw: 0.0
```

需要换起点时可覆盖：

```bash
ros2 launch 3dnav_bringup navigation.launch offline_test:=true \
  offline_start_x:=3.5 \
  offline_start_y:=5.5 \
  offline_start_z:=0.0
```

如果外部已经启动 FAST-LIO/Livox：

```bash
ros2 launch 3dnav_bringup navigation.launch \
  start_livox_driver:=false \
  start_fastlio:=false
```

临时切换全局规划算法：

```bash
ros2 launch 3dnav_bringup navigation.launch global_planner_algorithm:=pct
ros2 launch 3dnav_bringup navigation.launch global_planner_algorithm:=jie_octomap
ros2 launch 3dnav_bringup navigation.launch global_planner_algorithm:=astar
```

### 9. 完整系统

```bash
ros2 launch 3dnav_bringup full_system.launch
```

示例：

```bash
ros2 launch 3dnav_bringup full_system.launch mapping:=false rviz:=true
```

`full_system.launch` 同样默认使用 A* 全局规划，也可通过 `global_planner_algorithm:=pct` 或 `global_planner_algorithm:=jie_octomap` 临时切换。

## 导航执行控制

执行控制节点：

```text
nav_execution_controller_node
```

状态话题：

```bash
ros2 topic echo /nav3d/execution_state
```

状态机：

```text
IDLE
PATH_READY
RUNNING
PAUSED
STOPPED
GOAL_REACHED
ERROR
```

服务：

```bash
ros2 service call /nav3d/start std_srvs/srv/Trigger "{}"
ros2 service call /nav3d/stop std_srvs/srv/Trigger "{}"
ros2 service call /nav3d/pause std_srvs/srv/Trigger "{}"
ros2 service call /nav3d/resume std_srvs/srv/Trigger "{}"
ros2 service call /nav3d/clear_path std_srvs/srv/Trigger "{}"
```

执行逻辑：

1. RViz 选择目标点。
2. A* Global Planner 生成 `/planned_path`。
3. 状态变为 `PATH_READY`，机器人不会运动。
4. 调用 `/nav3d/start`。
5. 状态变为 `RUNNING`，`cmd_vel_gate` 开始转发 `/cmd_vel_nav` 到 `/cmd_vel`。
6. `/nav3d/stop` 或 `/nav3d/pause` 会立即发布零速度并阻断后续速度。

## 推荐工作流

### 新地图部署

```text
1. ros2 launch 3dnav_bringup mapping.launch
2. 保存/导出 FAST-LIO 地图到 maps/map_origin.pcd
3. ros2 launch 3dnav_bringup map_preprocess.launch
4. 确认 maps/map_preprocessed.pcd 和 maps/tomogram/map_preprocessed.pickle 存在
5. 可选：ros2 launch 3dnav_bringup map_planner_test.launch
6. ros2 launch 3dnav_bringup navigation.launch
```

### 日常导航

```text
1. source install/setup.bash
2. ros2 launch 3dnav_bringup navigation.launch
3. 在 RViz 中设置初始位姿和目标点
4. 等待 /nav3d/execution_state 变为 PATH_READY
5. ros2 service call /nav3d/start std_srvs/srv/Trigger "{}"
```

### 调参建议

- 定位 ICP 参数：`src/localization/localization_adapter/config/fastlio_mid360_icp_localization.yaml`
- PCT tomogram 参数：`src/map_process/PCT_planner/pct_planner_ros2/config/tomography.yaml`
- A* 全局规划参数：`src/global_planning/3dnav_global_planning/config/astar_global_planner.yaml`
- PCT 全局规划参数：`src/global_planning/pct_global_planner/config/pct_global_planner.yaml`
- EGO 局部规划参数：`src/local_planning/ego_local_planner/config/ego_local_planner.yaml`
- Go2 速度桥参数：`src/robot_api/go2_twist_bridge/config/twist_bridge.yaml`
- 执行控制参数：`src/3dnav_control/config/nav_execution_controller.yaml`

## 常用检查命令

查看包是否可见：

```bash
ros2 pkg prefix 3dnav_bringup
ros2 pkg prefix nav3d_bringup
ros2 pkg executables nav3d_control
```

查看 launch 参数：

```bash
ros2 launch 3dnav_bringup navigation.launch --show-args
```

查看关键话题：

```bash
ros2 topic list | grep -E "planned_path|cmd_vel|nav3d|icp|tomogram"
```

单独测试执行控制：

```bash
ros2 launch nav3d_control control.launch
```

## 常见问题

### 找不到 `3dnav_*` 包

确认已经 source 工作区：

```bash
source ~/3dnav_ws/install/setup.bash
```

实际构建包名是 `nav3d_*`，`3dnav_*` 是安装出来的兼容启动别名。

### `map_visualization.pcd` 不存在

这是允许的。定位会回退使用：

```text
maps/map_preprocessed.pcd
```

### PCT Planner 找不到 tomogram

确认文件存在：

```bash
ls maps/tomogram/map_preprocessed.pickle
```

如果不存在，先执行：

```bash
ros2 launch 3dnav_bringup map_preprocess.launch
```

### Livox 构建失败

确认 Livox SDK2 已安装，并且 `/usr/local/lib` 下存在：

```text
liblivox_lidar_sdk_shared.so
```

### Go2 不动

检查：

```bash
ros2 topic echo /nav3d/execution_state
ros2 topic echo /cmd_vel_nav
ros2 topic echo /cmd_vel
```

只有状态为 `RUNNING` 时，`cmd_vel_gate` 才会转发速度到 `/cmd_vel`。

## 维护约定

- 自研统一入口保持在 `3dnav_*` 目录下，实际 ROS 包名使用 `nav3d_*`。
- 第三方算法包名保持原名，例如 `fast_lio`、`livox_ros_driver2`、`pct_planner_ros2`。
- 不在配置文件中写死 `/home/...` 绝对路径。
- 地图运行文件统一进入根目录 `maps/`。
- 启动入口统一放在 `src/3dnav_bringup/launch/`。
