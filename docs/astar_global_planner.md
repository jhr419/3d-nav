# A* Global Planner

The standalone A* planner lives in `nav3d_global_planning`:

```text
src/global_planning/3dnav_global_planning
```

It is launched through the unified bringup:

```bash
ros2 launch nav3d_bringup global_planning.launch algorithm:=astar
```

or as part of full navigation:

```bash
ros2 launch nav3d_bringup navigation.launch global_planner_algorithm:=astar
```

## Configuration

Main config file:

```text
src/global_planning/3dnav_global_planning/config/astar_global_planner.yaml
```

Important parameters:

```yaml
map_source: "octomap"        # octomap / pcd / voxel_grid
octomap_file: "maps/map_preprocessed.bt"
pcd_file: "maps/map_preprocessed.pcd"
tomogram_topic: "/tomogram"
planning_mode: "2.5d"        # 2.5d / 3d
resolution: 0.20
unknown_as_occupied: true
require_traversable_support: true
enforce_obstacle_clearance: true
hard_collision_clearance: 0.05
hard_collision_min_column_points: 2
hard_collision_min_vertical_span: 0.20
tomogram_traversable_cost_threshold: 50.0
wall_clearance_cost_enabled: true
wall_preferred_clearance: 0.60
wall_clearance_weight: 10.0
wall_clearance_power: 2.0
tomogram_cost_enabled: true
tomogram_cost_normalizer: 20.0
tomogram_cost_weight: 8.0
tomogram_cost_power: 1.5
tomogram_z_aware_support: true
tomogram_filter_by_z: true
tomogram_support_z_tolerance: 0.50
traversable_support_radius: 0.25
traversable_neighbor_radius: 0.30
traversable_min_neighbors: 3
robot_radius: 0.35
safety_margin: 0.0
inflation_radius: 0.25
obstacle_min_relative_z: 0.20
obstacle_max_relative_z: 1.40
terrain_following_enabled: false
hard_min_clearance: 0.25
preferred_clearance: 0.75
clearance_weight: 2.0
```

`2.5d` searches in XY, checks obstacles within the robot body height range, and
by default restricts A* to the low-cost `/tomogram` domain published by the PCT
map publisher. This keeps A* from routing through unmapped blank space in RViz.
The output path still includes z. `3d` searches XYZ with 6, 18, or 26 neighbors.
The tomogram domain decides where A* is allowed to search, while
`hard_collision_clearance` keeps PCD/OctoMap wall cells as hard obstacles. The
larger clearance parameters remain soft costs so the search prefers open space
without sealing narrow passages.
For 2.5D, a PCD XY column is treated as a hard wall only when it has enough
vertical evidence in the robot body height band; this filters isolated map
noise that would otherwise split a corridor.
`wall_preferred_clearance` and the tomogram cost weights bias the search toward
the corridor centerline without making narrow doorways invalid.
With `tomogram_z_aware_support`, each planning request uses the tomogram layers
between the start and goal heights, which allows stair plans without projecting
all floors into one 2D map.

## Path Quality

A* scoring combines path length, clearance risk, and turn smoothness:

```yaml
clearance_cost_enabled: true
path_length_weight: 1.0
clearance_weight: 2.0
smoothness_weight: 1.0
search_bounds_padding: 8.0
search_bounds_use_full_map: false
enable_z_smoothing: true
z_smoothing_iterations: 30
z_max_step: 0.12
z_max_slope: 0.80
```

Cells closer than `hard_min_clearance`, `inflation_radius`, or
`robot_radius + safety_margin` are rejected. Cells between hard and preferred
clearance remain usable but receive extra cost.

The node removes duplicate points, resamples the path, smooths it, and validates
the smoothed path. If smoothing reduces clearance or causes collision, it falls
back to the unsmoothed path.

## Topics

Inputs:

```text
/tomogram           sensor_msgs/msg/PointCloud2
/goal_pose_3d       geometry_msgs/msg/PoseStamped
/goal_point_3d      geometry_msgs/msg/PointStamped
/goal_pose          geometry_msgs/msg/PoseStamped
```

Outputs:

```text
/planned_path       nav_msgs/msg/Path
/path               nav_msgs/msg/Path
/planned_path_marker visualization_msgs/msg/Marker
```

Debug:

```text
/astar_global_planner/status
/astar_global_planner/debug
/astar_expanded_nodes_marker
/astar_closed_set_marker
/astar_clearance_marker
```

## Verification

Launch A*:

```bash
ros2 launch nav3d_bringup global_planning.launch algorithm:=astar
```

Publish a goal:

```bash
ros2 topic pub --once /goal_pose_3d geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 3.0, y: 2.0, z: 0.5},
    orientation: {w: 1.0}
  }
}"
```

Check outputs:

```bash
ros2 topic echo /astar_global_planner/status --once
ros2 topic echo /planned_path --once
ros2 topic echo /astar_global_planner/debug --once
```

In RViz, enable `AStar Expanded Nodes` and `AStar Closed Set` only when needed;
large searches can publish many points. `AStar Clearance` shows the final path
clearance risk.
