# dddmr_fastlio_adapter

This package adapts LiDAR SLAM mapping output to the DDDMR map format used by
`mcl_3dl` and the beginner guide navigation launches. It supports the original
Fast-LIO2 odometry flow and a GLIM ROS2 flow that reads the pose from TF.

## Runtime Inputs

Fast-LIO2 mode:

- `/Odometry` (`nav_msgs/msg/Odometry`) from Fast-LIO2
- `/cloud_registered_body` (`sensor_msgs/msg/PointCloud2`) from Fast-LIO2

Start Fast-LIO2 with the adapter:

```bash
ros2 launch mapping_adapter fast_adapter_mapping.launch \
  mapping_dir:=/tmp/dddmr_fastlio_mid360_map
```

This launch keeps FAST-LIO on the known-good `fast_lio/config/mid360.yaml` and
uses `mapping_adapter/config/fastlio_mid360_dddmr.yaml` only for converting
FAST-LIO's body-frame outputs into the DDDMR map layout.

GLIM mode:

- GLIM TF: `map -> odom -> base_link`
- `/glim_ros/points` (`sensor_msgs/msg/PointCloud2`) from `librviz_viewer.so`

For GLIM, make sure your GLIM `config_ros.json` loads `librviz_viewer.so`;
otherwise `glim_rosnode` will not publish the TF and point topics used here.

## Runtime Outputs

- `/lego_loam_map`
- `/lego_loam_ground`
- `/lego_loam_ground_edge`
- `/cloud_keypose_6d`
- `/key_poses`
- `/tf`: `map -> base_link` when `publish_tf` is true

## Saved DDDMR Map Layout

Call the save service after mapping:

```bash
ros2 service call /save_fastlio_dddmr_map std_srvs/srv/Trigger {}
```

In GLIM mode, the same save callback is also exposed as:

```bash
ros2 service call /save_glim_dddmr_map std_srvs/srv/Trigger {}
```

The adapter writes:

```text
<mapping_dir>/
  poses.pcd
  edges.pcd
  pcd/
    0_feature.pcd
    0_ground.pcd
    0_surface.pcd
    1_feature.pcd
    1_ground.pcd
    1_surface.pcd
    ...
```

For navigation, set `pose_graph_dir` in the DDDMR navigation yaml to the same
`mapping_dir`.

## GLIM Mapping Launch

After building in your container, start GLIM + the DDDMR adapter with:

```bash
ros2 launch dddmr_beginner_guide livox_mid360_glim_dddmr_mapping.launch.py \
  glim_config_path:=/path/to/your/glim/config \
  mapping_dir:=/tmp/dddmr_glim_mid360_map
```

The default adapter config is
`dddmr_fastlio_adapter/config/glim_mid360_dddmr_adapter.yaml`. Tune
`base_to_body_*`, ground thresholds, and the GLIM `config_ros.json`
`points_topic`/`imu_topic` for your robot.
