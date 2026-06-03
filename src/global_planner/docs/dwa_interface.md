# 3D DWA Interface Contract

This package currently provides the 3D global planning side. The local planner can be added later without changing the global path contract.

## Subscribed Topics

- `/global_path_3d` (`nav_msgs/msg/Path`): resampled 3D global path from start to goal.
- `/local_obstacle_cloud` (`sensor_msgs/msg/PointCloud2`): local obstacle cloud in `map`, `odom`, or `base_link`.
- `/odom` (`nav_msgs/msg/Odometry`) or TF `map -> base_link`: robot pose and velocity source.

## Published Topics

- `/cmd_vel` (`geometry_msgs/msg/Twist`): velocity command.
- `/local_trajectory_marker` (`visualization_msgs/msg/Marker`): selected local trajectory for debugging.

## Global Path Format

- `header.frame_id` is `map`.
- `poses` are ordered from current start to final goal.
- `pose.position.x/y/z` are all valid and must not be flattened to z=0.
- `pose.orientation` may be the unit quaternion for intermediate poses.
- The final pose may preserve the requested goal orientation.
- Path spacing is controlled by `path_resample_resolution` and should stay close to the map resolution.

## Coordinate Frames

- Global planning uses `map`.
- TF start lookup uses `map -> base_link`.
- Local obstacle input may arrive in `map`, `odom`, or `base_link`; the local planner should transform it into its planning frame before collision checks.

## Motion Model

Ground robot default:

- `linear.x`: forward velocity `vx`
- `linear.y`: lateral velocity `vy` for omnidirectional bases, otherwise keep zero
- `angular.z`: yaw rate `wz`

True 3D vehicle optional extension:

- `linear.x/y/z`: `vx`, `vy`, `vz`
- `angular.x/y/z`: `wx`, `wy`, `wz`

## Local Obstacle Map Options

Recommended first input:

- `sensor_msgs/msg/PointCloud2` on `/local_obstacle_cloud`

Future options:

- local OctoMap submap
- ESDF or distance field
- rolling voxel grid with inflation

## Expected Local Planner Behavior

- Track `/global_path_3d` in order.
- Select a short lookahead segment around the robot.
- Sample velocity commands according to the selected motion model.
- Reject trajectories colliding with local obstacles plus safety inflation.
- Publish `/cmd_vel` at a fixed control rate and `/local_trajectory_marker` for visualization.
