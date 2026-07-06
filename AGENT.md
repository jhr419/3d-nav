# AGENT.md

This file is the fast context entry point for Codex/AI agents working in this
ROS 2 Humble 3D navigation workspace. Read it before scanning the full repo.
After every completed code change task, update this file when behavior,
interfaces, launch flow, configuration, maps, or debugging workflow changes.

## 1. Project Overview

This workspace implements a ROS 2 Humble 3D navigation system for Livox MID360,
FAST-LIO mapping/localization, PCT Tomography map preprocessing, switchable
global planning, EGO local planning, Unitree Go2 velocity bridging, execution
control, and RViz/debug tooling.

Main pipeline:

```text
Livox/FAST-LIO or offline map
  -> map preprocessing / tomogram
  -> FAST-LIO + ICP localization
  -> global planner (/planned_path)
  -> EGO local planner (/cmd_vel_nav)
  -> nav3d cmd_vel gate (/cmd_vel)
  -> Go2 / Unitree bridge
```

Supported global planners:

- `astar`: self-contained A* planner in `src/global_planning/3dnav_global_planning`, current default.
- `pct`: PCT Planner backend through `src/global_planning/pct_global_planner`.
- `jie_octomap`: `jie_3d_nav` / OctoMap planner through `octo_planner`.

Navigation execution is controlled by services for Start, Stop, Pause, Resume,
and Clear Path. RViz2 is used for localization, planning, goal selection, and
debug markers. Local planner file logging is available under
`debug/logs/local_planner`.

## 2. Repository Layout

Actual top-level layout:

```text
3dnav_ws/
|-- AGENT.md
|-- README.md
|-- docs/
|   |-- astar_global_planner.md
|   `-- global_planner_switching.md
|-- maps/
|   |-- map_origin.pcd
|   |-- map_preprocessed.pcd
|   |-- ply_map.ply
|   |-- jiemaps/
|   |-- src_maps/
|   `-- tomogram/
|-- scripts/
|   |-- analyze_local_planner_log.py
|   `-- tail_latest_local_planner_log.sh
`-- src/
    |-- 3dnav_bringup/
    |-- 3dnav_common/
    |-- 3dnav_control/
    |-- 3dnav_rviz_plugins/
    |-- global_planning/
    |-- local_planning/
    |-- localization/
    |-- map_process/
    |-- mapping/
    `-- robot_api/
```

Generated or bulky directories such as `build/`, `install/`, `log/`,
`debug/logs/`, `debug/screenshots/`, large PCD/BT/pickle files, and third-party
vendor subtrees should not be scanned deeply unless the task requires it.

ROS package names use `nav3d_*` where ROS package names cannot start with a
digit. Compatibility launch aliases for `3dnav_*` are installed for daily use.

## 3. Module Responsibilities

### mapping

Path: `src/mapping`

- `FAST_LIO` (`fast_lio`): third-party FAST-LIO mapping/localization core.
- `livox_ros_driver2`: third-party Livox driver.
- `mapping_adapter`: project adapter around FAST-LIO/Livox mapping.
- `3dnav_mapping` (`nav3d_mapping`): project compatibility/bringup package.

Avoid casual edits to FAST-LIO and Livox driver internals. Prefer adapter,
launch, and config changes unless the task explicitly targets the core vendor
code.

### map_process

Path: `src/map_process`

- `PCT_planner`: upstream PCT tomography/planner code and ROS 2 wrappers.
- `pct_planner_ros2`: tomography generation, map publisher, planner wrapper,
  start/goal marker tools.
- `pcd_preprocessor_ros2`: PCD preprocessing for tomogram/map generation.
- `map_mannger`: utility package, including bin-to-PCD conversion.
- `3dnav_map_process` (`nav3d_map_process`): project compatibility package.

Primary preprocessing launch:

```bash
ros2 launch 3dnav_bringup map_preprocess.launch
```

It reads `maps/map_origin.pcd` and writes `maps/map_preprocessed.pcd` plus
`maps/tomogram/map_preprocessed.pickle`.

### localization

Path: `src/localization`

- `localization_adapter`: project FAST-LIO + ICP localization node.
- `3dnav_localization` (`nav3d_localization`): project compatibility package.

Main config:

```text
src/localization/localization_adapter/config/fastlio_mid360_icp_localization.yaml
```

Default map inputs:

- `map_pcd_path`: `maps/map_preprocessed.pcd`
- `icp_map_pcd_path`: `maps/map_preprocessed.pcd`
- `visualization_map_pcd_path`: `maps/map_visualization.pcd`, with launch
  fallback to `maps/map_preprocessed.pcd` if missing.

Important topics/frames:

- Inputs: `/Odometry`, `/cloud_registered_body`, `/initialpose`
- Outputs include ICP pose/map cloud and TF according to config.
- Frames: `map`, `odom`, `base_link`, `base_footprint`, `livox_frame`

### global_planning

Path: `src/global_planning`

- `3dnav_global_planning` (`nav3d_global_planning`): project A* global planner.
- `pct_global_planner`: project wrapper/backend for PCT global planning.
- `jie_3d_nav`: third-party / external OctoMap planner stack
  (`jie_map_msgs`, `jie_octomap`, `octo_planner`).

All global planners publish the same downstream interface:

```text
/planned_path
/path
/planned_path_marker
```

Default backend is configured in:

```text
src/3dnav_bringup/config/nav3d_system.yaml
```

Current default:

```yaml
global_planning:
  algorithm: "astar"
```

Switchable YAML/launch values:

```text
astar / pct / jie_octomap
```

One-off overrides:

```bash
ros2 launch 3dnav_bringup global_planning.launch algorithm:=astar
ros2 launch 3dnav_bringup global_planning.launch algorithm:=pct
ros2 launch 3dnav_bringup global_planning.launch algorithm:=jie_octomap
```

The A* planner config is:

```text
src/global_planning/3dnav_global_planning/config/astar_global_planner.yaml
```

See `docs/global_planner_switching.md` and `docs/astar_global_planner.md` for
planner details.

### local_planning

Path: `src/local_planning`

- `ego_local_planner`: active EGO local planner adapter and local A*/tracking
  implementation.
- `3dnav_local_planning` (`nav3d_local_planning`): compatibility launch package.

Main config:

```text
src/local_planning/ego_local_planner/config/ego_local_planner.yaml
```

Core interfaces:

- Subscribes: `/planned_path`
- Point clouds: `/livox/lidar`, `/cloud_registered`, `/cloud_registered_body`,
  `/mid360`, plus static `/global_points`
- Odometry: `/Odometry`
- Publishes: `/cmd_vel_nav`, `/ego_local_trajectory`,
  `/ego_local_trajectory_marker`, `/ego_local_map_vis`, target/candidate/collision
  markers, `/ego_local_planner/status`, `/ego_debug_text`

Local planner file logging is implemented in
`src/local_planning/ego_local_planner/src/ego_local_planner_node.cpp` through
`LocalPlannerLogger`. Configuration key:

```yaml
local_planner_logging:
  enabled: true
  log_dir: "debug/logs/local_planner"
  log_file_prefix: "local_planner"
  log_format: "text"
```

The logger records startup, state changes, path summaries, point cloud/map
status, local target updates, replans, plan success/failure, throttled warnings,
cmd_vel changes, goal reached, and periodic summaries. It uses project-relative
paths, timestamped files, warning throttling, flush intervals, and size-based
log rotation.

Debug helpers:

```bash
bash scripts/tail_latest_local_planner_log.sh
python3 scripts/analyze_local_planner_log.py
```

### robot_api

Path: `src/robot_api`

- `go2_twist_bridge`: project bridge from ROS Twist to Unitree Go2 control.
- `go2_driver`: Go2 driver integration.
- `go2_teleop_ctrl_keyboard_py`: teleop helper.
- `unitree_ros2`: third-party Unitree ROS 2 SDK/examples.
- `3dnav_robot_api` (`nav3d_robot_api`): compatibility launch package.

Velocity flow:

```text
/cmd_vel_nav -> cmd_vel_gate -> /cmd_vel -> go2_twist_bridge -> Go2
```

### bringup/control/common/rviz

- `src/3dnav_bringup`: unified launch, path resolution, system configuration,
  and algorithm switching.
- `src/3dnav_control`: execution state machine and command velocity gate.
- `src/3dnav_common`: common compatibility package.
- `src/3dnav_rviz_plugins`: RViz plugin/package placeholder.

Execution state machine states:

```text
IDLE, PATH_READY, RUNNING, PAUSED, STOPPED, GOAL_REACHED, ERROR
```

Control services:

```bash
ros2 service call /nav3d/start std_srvs/srv/Trigger "{}"
ros2 service call /nav3d/stop std_srvs/srv/Trigger "{}"
ros2 service call /nav3d/pause std_srvs/srv/Trigger "{}"
ros2 service call /nav3d/resume std_srvs/srv/Trigger "{}"
ros2 service call /nav3d/clear_path std_srvs/srv/Trigger "{}"
```

## 4. Build Instructions

Standard build:

```bash
cd /home/jhr/3dnav_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Install dependencies when needed:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

PCT core library, if missing:

```bash
cd /home/jhr/3dnav_ws/src/map_process/PCT_planner/planner
bash build_thirdparty.sh
bash build.sh
cd /home/jhr/3dnav_ws
colcon build --symlink-install
```

`src/map_process/PCT_planner/planner/lib` is a standalone CMake core and has
`COLCON_IGNORE` so workspace builds do not accidentally pick up an incompatible
system GTSAM/Eigen pair. Use the scripts above when the PCT `.so` files need to
be rebuilt.

## 5. Main Launch Commands

Map preprocessing:

```bash
ros2 launch 3dnav_bringup map_preprocess.launch
```

Map/planner feasibility test:

```bash
ros2 launch 3dnav_bringup map_planner_test.launch
```

Localization:

```bash
ros2 launch 3dnav_bringup localization.launch
```

Global planning:

```bash
ros2 launch 3dnav_bringup global_planning.launch
ros2 launch 3dnav_bringup global_planning.launch algorithm:=astar
ros2 launch 3dnav_bringup global_planning.launch algorithm:=pct
ros2 launch 3dnav_bringup global_planning.launch algorithm:=jie_octomap
```

Local planning:

```bash
ros2 launch 3dnav_bringup local_planning.launch
ros2 launch 3dnav_bringup local_planning.launch require_fresh_cloud:=false
```

Robot API / execution control:

```bash
ros2 launch 3dnav_bringup robot_api.launch
```

Default online navigation:

```bash
ros2 launch 3dnav_bringup navigation.launch
```

Offline planning/RViz test:

```bash
ros2 launch 3dnav_bringup navigation.launch offline_test:=true
```

Full system wrapper:

```bash
ros2 launch 3dnav_bringup full_system.launch
```

## 6. Maps And Project-Relative Paths

Runtime maps are kept in `maps/`:

- `maps/map_origin.pcd`: mapping output and preprocessing input.
- `maps/map_preprocessed.pcd`: main preprocessed map for localization and
  planning.
- `maps/map_preprocessed.bt`: optional OctoMap map path used by some planners.
- `maps/map_visualization.pcd`: optional lighter visualization map.
- `maps/tomogram/map_preprocessed.pickle`: PCT tomogram.
- `maps/jiemaps/map_preprocessed/`: jie/OctoMap derived artifacts
  (`meta.yaml`, `layers.npz`, `octomap_msg.npz`).
- `maps/src_maps/`: archived/source PCD maps; do not parse large point clouds
  unless the task requires it.

Do not hard-code `/home/jhr/3dnav_ws` in configs or source. Use launch helpers
from `src/3dnav_bringup/launch/_utils.py`, project-relative config paths, or
the `NAV3D_WS` environment variable when running from another directory.

## 7. Algorithm And Config Switches

YAML-driven switches currently include:

- Global planner backend:
  `src/3dnav_bringup/config/nav3d_system.yaml`,
  `global_planning.algorithm`.
- Global planner backend registration:
  `global_planning.algorithms` in the same file.
- A* behavior:
  `src/global_planning/3dnav_global_planning/config/astar_global_planner.yaml`.
- PCT global planner:
  `src/global_planning/pct_global_planner/config/pct_global_planner.yaml`.
- PCT tomography:
  `src/map_process/PCT_planner/pct_planner_ros2/config/tomography.yaml`.
- FAST-LIO + ICP localization:
  `src/localization/localization_adapter/config/fastlio_mid360_icp_localization.yaml`.
- EGO local planner and local planner logging:
  `src/local_planning/ego_local_planner/config/ego_local_planner.yaml`.
- Execution control:
  `src/3dnav_control/config/nav_execution_controller.yaml`.
- Go2 twist bridge:
  `src/robot_api/go2_twist_bridge/config/twist_bridge.yaml`.

## 8. Debugging Checklist

Build/package visibility:

```bash
ros2 pkg prefix 3dnav_bringup
ros2 pkg prefix nav3d_bringup
ros2 pkg executables nav3d_control
```

Key topics:

```bash
ros2 topic list | grep -E "planned_path|cmd_vel|nav3d|icp|tomogram"
ros2 topic echo /planned_path --once
ros2 topic echo /ego_local_planner/status --once
ros2 topic echo /nav3d/execution_state --once
```

Local planner logs:

```bash
bash scripts/tail_latest_local_planner_log.sh
python3 scripts/analyze_local_planner_log.py
```

Common motion issue: Go2 will not move unless `/nav3d/execution_state` is
`RUNNING`; `cmd_vel_gate` blocks `/cmd_vel_nav` in other states.

## 9. Maintenance Rules For Future Agents

- Always read this file first, then inspect only the relevant module files.
- After every completed code change task, update `AGENT.md` if any documented
  behavior, path, topic, parameter, launch command, map artifact, or debug
  workflow changed.
- Prefer project adapters, launch files, and YAML configs over modifying
  third-party cores (`FAST_LIO`, `livox_ros_driver2`, `PCT_planner`,
  `jie_3d_nav`, `unitree_ros2`) unless the task explicitly requires it.
- Keep paths project-relative. Do not introduce hard-coded `/home/...` paths.
- Keep `/planned_path` as the global-to-local planner contract unless all
  downstream consumers are updated together.
- Keep `/cmd_vel_nav -> cmd_vel_gate -> /cmd_vel` as the execution-safety path.
- Avoid scanning or modifying generated directories and large map/log assets
  unless explicitly requested.
- When changing local planning, preserve throttled file logging behavior and
  update `scripts/analyze_local_planner_log.py` if new event names are added.
