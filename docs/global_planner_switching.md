# Global Planner Switching

The navigation stack currently supports two global planning backends:

- `pct`: PCT Planner, the existing default.
- `jie_octomap`: `jie_3d_nav` OctoMap planner through the `octo_planner` adapter.

Both backends publish the same downstream interface:

- `/planned_path` (`nav_msgs/msg/Path`)
- `/path` (`nav_msgs/msg/Path`)
- `/planned_path_marker` (`visualization_msgs/msg/Marker`, `LINE_STRIP`)

All global paths use the `map` frame and contain real `x/y/z` waypoints.

## Switch By Configuration

Edit `src/3dnav_bringup/config/nav3d_system.yaml`:

```yaml
global_planning:
  algorithm: "pct"
```

or:

```yaml
global_planning:
  algorithm: "jie_octomap"
```

The unified launch reads the same `global_planning.algorithms` table, so future
planners can be added by registering a package, launch file, and config file in
that YAML.

## Launch PCT Only

```bash
ros2 launch nav3d_bringup global_planning.launch algorithm:=pct
```

The compatibility package alias installed by `nav3d_bringup` also allows:

```bash
ros2 launch 3dnav_bringup global_planning.launch algorithm:=pct
```

## Launch jie_3d_nav Only

```bash
ros2 launch nav3d_bringup global_planning.launch algorithm:=jie_octomap
```

The lower-level launch is:

```bash
ros2 launch octo_planner jie_3d_global_planner.launch.py
```

This starts only the OctoMap global planner path pipeline. It does not start
local planning, localization, the old `d1_controller`, or any node that publishes
directly to `/cmd_vel`.

## Full Navigation With jie_3d_nav

Set:

```yaml
global_planning:
  algorithm: "jie_octomap"
```

Then run:

```bash
ros2 launch nav3d_bringup navigation.launch
```

For a one-off override:

```bash
ros2 launch nav3d_bringup navigation.launch global_planner_algorithm:=jie_octomap
```

## jie_3d_nav Map Files

The preferred OctoMap input is:

```text
maps/map_preprocessed.bt
```

If that file is missing but this PCD exists:

```text
maps/map_preprocessed.pcd
```

`octo_planner/jie_3d_global_planner.launch.py` starts
`jie_octomap/pcd_to_octomap_node` automatically and publishes `/octomap` from
the PCD. If neither file exists, the adapter publishes `MAP_ERROR` on:

```text
/jie_3d_global_planner/status
```

All map paths are project-relative and are resolved against the workspace root.
Do not configure `/home/...` paths in the planner config.

## Goal And Start Inputs

`jie_octomap` uses the current robot pose from TF by default:

```yaml
start_source: "tf"
map_frame: "map"
base_frame: "base_link"
```

It accepts 3D goals on:

```text
/goal_pose_3d
/goal_point_3d
```

It also keeps a debug manual start mode through:

```text
/jie_3d_nav/manual_start_point
/jie_3d_nav/manual_start_pose
```

Set `start_source: "manual"` or `start_source: "topic"` in
`octo_planner/config/jie_3d_global_planner.yaml` to use those debug topics.

## EGO-Planner Interface

EGO-Planner subscribes to:

```text
/planned_path
```

The adapter republishes the raw `jie_path_node` output to `/planned_path` and
`/path`, then publishes a `LINE_STRIP` marker on `/planned_path_marker`. This
keeps EGO-Planner independent of which global planner is selected.

Check the path with:

```bash
ros2 topic echo /planned_path --once
ros2 topic info /planned_path
```

Check EGO-Planner is subscribed with:

```bash
ros2 topic info /planned_path
ros2 topic echo /ego_local_planner/status
```

## Execution Control

`jie_octomap` only publishes global paths. Robot motion still flows through:

```text
/planned_path -> EGO-Planner -> /cmd_vel_nav -> cmd_vel_gate -> /cmd_vel
```

The robot should not execute a newly published path until `/nav3d/start` opens
the navigation execution controller. `/nav3d/stop` should still stop motion
through `cmd_vel_gate`.

## Avoid Topic Conflicts

Do not run `pct` and `jie_octomap` global planners at the same time in normal
navigation. Both intentionally converge on `/planned_path`, `/path`, and
`/planned_path_marker`, so running both can make the local planner receive
competing global paths.
