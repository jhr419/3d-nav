# localization_adapter

This package provides an ICP localization node for initializing and
relocalizing a robot against an existing FAST-LIO / mapping_adapter map.

## Launch

```bash
ros2 launch localization_adapter fastlio_icp_localization.launch \
  map_pcd_path:=/home/jhr/jhr/fast_ws/src/mapping_adapter/maps/map.pcd
```

The launch runs `icp_localization_node` and opens RViz. By default it does not
start FAST-LIO, so RViz first shows the prior map from `/icp_map`. Use the
`2D Pose Estimate` tool to publish `/initialpose`; ICP will start after odometry
and scan topics are available. The odometry topic is FAST-LIO's `/Odometry` by
default, and the scan topic is FAST-LIO's `/cloud_registered_body`.

If you want this launch to also start FAST-LIO as the odometry/scan source, add:

```bash
start_fastlio:=true
```

Topic overrides are available when your FAST-LIO output names differ:

```bash
fastlio_odom_topic:=/Odometry fastlio_scan_topic:=/cloud_registered_body
```

## Inputs

- `/Odometry`
- `/cloud_registered_body`
- `/initialpose`

## Outputs

- `/icp_pose`
- `/icp_aligned_cloud`
- `/icp_map`
- `/tf`: `map -> camera_init` by default

Send an initial pose from RViz/Nav2 on `/initialpose` to initialize or
relocalize. For startup without RViz, set `use_initial_pose_param: true` and
tune `initial_pose_xyz` / `initial_pose_rpy` in
`config/fastlio_mid360_icp_localization.yaml`.
