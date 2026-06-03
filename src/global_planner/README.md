# global_planner

ROS 2 3D global planner extracted from `OctoPlanner3D/planner`. The package plans on an OctoMap loaded from a file path or received from an `octomap_msgs/msg/Octomap` topic.

## Pipeline

```text
OctoMap
  -> traversable cell extraction
  -> collision checking with robot radius
  -> preblocked/risk cost layer near unsafe cells
  -> 26-neighbor 3D A* search
  -> nav_msgs/Path
```

## Interfaces

- Map file: set `map_source: file` and `map_file_path` to an OctoMap `.bt/.ot` file or a PCD `.pcd` file.
- Map topic: set `map_source: topic` to subscribe to `/octomap_full` (`octomap_msgs/msg/Octomap`)
- Debug mode start: `/global_planner/start` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- Localization mode start: `/icp_pose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- Subscribe: `/goal_pose` (`geometry_msgs/msg/PoseStamped`)
- Publish: `/global_path` (`nav_msgs/msg/Path`)
- Publish map visualization: `/global_planner/occupied_map` (`sensor_msgs/msg/PointCloud2`)

Topics are configurable in `config/global_planner.yaml`.

## Map Input

For static maps, configure:

```yaml
map_source: file
map_file_path: /absolute/path/to/map.bt
```

Supported file formats are OctoMap `.bt` / `.ot` and point cloud `.pcd`.

For PCD maps, the node converts occupied PCD points into an internal OctoMap:

```yaml
map_source: file
map_file_path: /absolute/path/to/map.pcd
pcd_octomap_resolution: 0.20
pcd_voxel_leaf_size: 0.0
pcd_min_z: -1000.0
pcd_max_z: 1000.0
```

If you want to use a live map publisher instead:

```yaml
map_source: topic
octomap_topic: /octomap_full
```

## Modes

Set `debug_mode` in `config/global_planner.yaml`:

- `debug_mode: true`: manually publish/select both start and goal.
- `debug_mode: false`: use the latest pose from `icp_localization_node` as the start, then manually select only the goal.

When `replan_on_pose_update` is `false`, localization mode plans when the first localization pose arrives or when a new goal/map arrives. Set it to `true` if you want continuous replanning as `/icp_pose` updates.

## Run

```bash
colcon build --packages-select global_planner
source install/setup.bash
ros2 launch global_planner global_planner.launch.py
```

RViz starts by default. To launch only the planner:

```bash
ros2 launch global_planner global_planner.launch.py use_rviz:=false
```

Example start/goal:

```bash
ros2 topic pub --once /global_planner/start geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}}}"
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 5.0, y: 2.0, z: 1.0}, orientation: {w: 1.0}}}"
```

## Notes

- The pure C++ planner lives in `GlobalPlannerCore` and can be reused without ROS node logic.
- `require_ground_support` keeps paths on cells supported by occupied voxels below.
- `robot_radius` inflates collision checks around each candidate cell.
- `enable_preblocked_costmap` adds a soft penalty near cells identified as risky by the preprocessing mask.
