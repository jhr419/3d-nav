# localization_adapter

This package provides an ICP localization node for initializing and
relocalizing a robot against an existing FAST-LIO / mapping_adapter map.

## Launch

```bash
ros2 launch localization_adapter fastlio_icp_localization.launch \
  visualization_map_pcd_path:=maps/map_visualization.pcd \
  icp_map_pcd_path:=maps/map_preprocessed.pcd
```

The launch runs `icp_localization_node` and opens RViz. By default it does not
start FAST-LIO, so RViz first shows the ground map from
`/icp_visualization_map`. Use the `2D Pose Estimate` tool to publish
`/initialpose`; ICP registers against the non-ground target map from
`icp_map_pcd_path` after odometry and scan topics are available. The odometry
topic is FAST-LIO's `/Odometry` by default, and the scan topic is FAST-LIO's
`/cloud_registered_body`.

If you want this launch to also start FAST-LIO as the odometry/scan source, add
the option below. The launch uses FAST-LIO's
`config/mid360_localization.yaml`, where `localization.odom_only_mode: true`
publishes `/Odometry` and `/cloud_registered_body` without building the
FAST-LIO map.

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
- `/icp_visualization_map`
- `/icp_map` non-ground ICP target map
- `/tf`: `map -> camera_init` by default

Send an initial pose from RViz/Nav2 on `/initialpose` to initialize or
relocalize. For startup without RViz, set `use_initial_pose_param: true` and
tune `initial_pose_xyz` / `initial_pose_rpy` in
`config/fastlio_mid360_icp_localization.yaml`.
