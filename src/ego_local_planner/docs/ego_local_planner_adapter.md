# EGO Local Planner Adapter

This package is a ROS 2 ground-robot adapter for the ROS 1 EGO-Planner code in
`src/ego-planner-swarm`.

The upstream EGO packages are catkin/ROS 1 packages and publish UAV-oriented
B-spline and `quadrotor_msgs/PositionCommand` outputs. This adapter keeps the
current 3D navigation interfaces instead:

- input global path: `/planned_path` (`nav_msgs/msg/Path`)
- input live cloud: `/livox/lidar` (`sensor_msgs/msg/PointCloud2`)
- input pose: TF `map -> base_link`, with `/odom` fallback
- output command: `/cmd_vel` (`geometry_msgs/msg/Twist`, `vx`, `vy`, `wz`)
- visualization: `/ego_local_trajectory`, `/ego_local_trajectory_marker`,
  `/ego_local_map_vis`, `/ego_target_marker`

The first version uses an EGO-style local occupancy map and frequent local
replanning loop, but avoids catkin, UAV dynamics, `quadrotor_msgs`, and vertical
velocity output. It builds a local inflated occupancy map from live point clouds,
selects a local target from the global path, replans a local collision-free
trajectory, and tracks it as ground omnidirectional `cmd_vel`.

## Recommended launch split

Start the existing global planner and web selector first:

```bash
ros2 launch jie_octomap web_octomap.launch.py
```

Then start only the EGO local planner:

```bash
ros2 launch ego_local_planner ego_local_planner.launch.py
```

`ego_navigation.launch.py` is kept only as an optional combined/debug launch.
Its global-planner and goal-bridge nodes are disabled by default so it will not
start a duplicate `jie_path_node` when `web_octomap.launch.py` is already
running.
