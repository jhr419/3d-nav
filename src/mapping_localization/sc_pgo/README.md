# SC-PGO ROS2

This package ports the SC-PGO pose graph backend to ROS 2 and adapts the default
interfaces to the existing FAST-LIO2 package in this workspace.

Default inputs:

- `/Odometry` (`nav_msgs/msg/Odometry`)
- `/cloud_registered_body` (`sensor_msgs/msg/PointCloud2`)

Default outputs:

- `/aft_pgo_odom`
- `/aft_pgo_path`
- `/aft_pgo_map`
- `/loop_scan_local`
- `/loop_submap_local`

Build only this package:

```bash
colcon build --packages-select sc_pgo
```

Run SC-PGO after FAST-LIO2 is already running:

```bash
ros2 launch sc_pgo sc_pgo.launch.py
```

Run FAST-LIO2 and SC-PGO together:

```bash
ros2 launch sc_pgo fastlio_sc_pgo.launch.py
```

The node writes `optimized_poses.txt`, `odom_poses.txt`, `times.txt`, and
optional keyframe scans under `save_directory` (`/tmp/sc_pgo` by default).
