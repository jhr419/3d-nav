# Global Planner Switching

The navigation stack supports three global planning backends:

- `astar`: standalone A* planner in `nav3d_global_planning`, the default backend.
- `pct`: PCT Planner backend.
- `jie_octomap`: `jie_3d_nav` OctoMap planner through the `octo_planner` adapter.

All backends publish the same downstream interface:

- `/planned_path` (`nav_msgs/msg/Path`)
- `/path` (`nav_msgs/msg/Path`)
- `/planned_path_marker` (`visualization_msgs/msg/Marker`)

Paths use `frame_id: map`, contain real `x/y/z` waypoints, and are ordered
from start to goal. Global planners only publish paths; robot execution still
flows through EGO-Planner and the command gate.

## Switch By Configuration

Edit `src/3dnav_bringup/config/nav3d_system.yaml`:

```yaml
global_planning:
  enabled: true
  algorithm: "astar"        # astar / pct / jie_octomap
```

The same file registers each planner in `global_planning.algorithms`:

```yaml
algorithms:
  pct:
    package: "pct_global_planner"
    launch: "pct_global_planner.launch.py"
  jie_octomap:
    package: "octo_planner"
    launch: "jie_3d_global_planner.launch.py"
  astar:
    package: "nav3d_global_planning"
    launch: "astar_global_planner.launch.py"
```

Only the selected launch file is included, so the three planners do not publish
competing `/planned_path` messages.

## One-Off Launch Override

```bash
ros2 launch nav3d_bringup global_planning.launch algorithm:=pct
ros2 launch nav3d_bringup global_planning.launch algorithm:=jie_octomap
ros2 launch nav3d_bringup global_planning.launch algorithm:=astar
```

The compatibility package alias also allows:

```bash
ros2 launch 3dnav_bringup global_planning.launch algorithm:=astar
```

## Full Navigation

Use the configured algorithm:

```bash
ros2 launch nav3d_bringup navigation.launch
```

Temporarily override it:

```bash
ros2 launch nav3d_bringup navigation.launch global_planner_algorithm:=pct
ros2 launch nav3d_bringup navigation.launch global_planner_algorithm:=jie_octomap
ros2 launch nav3d_bringup navigation.launch global_planner_algorithm:=astar
```

## Map Files

The common project-relative map paths are:

```text
maps/map_preprocessed.pcd
maps/map_preprocessed.bt
maps/tomogram/map_preprocessed.pickle
```

`pct` uses the tomogram pickle. `jie_octomap` prefers the `.bt` file and can
build `/octomap` from the PCD. `astar` also prefers `maps/map_preprocessed.bt`;
if it is missing, it loads `maps/map_preprocessed.pcd` directly into its
internal voxel grid. In `2.5d` mode, `astar` also subscribes to the PCT
`/tomogram` visualization cloud and only searches low-cost tomogram cells, so
algorithm switching keeps the same map domain.

Avoid hard-coded `/home/...` paths. Use paths relative to the workspace root or
set `NAV3D_WS` when launching from another directory.

## Goal And Start Inputs

The global planners use TF start by default:

```yaml
start_source: "tf"
map_frame: "map"
base_frame: "base_link"
```

Supported goal topics:

```text
/goal_pose_3d
/goal_point_3d
/goal_pose
```

For `/goal_pose`, the planner uses `default_goal_z`.

## EGO-Planner Interface

EGO-Planner subscribes to:

```text
/planned_path
```

Verify the global path:

```bash
ros2 topic echo /planned_path --once
ros2 topic info /planned_path
```

Verify EGO-Planner is connected:

```bash
ros2 topic info /planned_path
ros2 topic echo /ego_local_planner/status
```

Robot motion remains gated:

```text
/planned_path -> EGO-Planner -> /cmd_vel_nav -> cmd_vel_gate -> /cmd_vel
```

Use `/nav3d/start` to execute and `/nav3d/stop` to stop.
