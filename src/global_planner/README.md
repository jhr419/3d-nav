# global_planner

This package integrates FAST-LIO/ICP localization with 3D OctoMap global planning.

## Build

```bash
colcon build --packages-select global_planner
source install/setup.bash
```

## Full Localization + 3D Goal Picking + Global Planning

```bash
ros2 launch global_planner global_planner_localization.launch
```

Workflow:

1. Wait for FAST-LIO odometry and ICP localization to start.
2. In the Open3D picker terminal, enter a start pose:

   ```text
   start x y z
   ```

   This publishes `/initial_pose_3d` and the bridge forwards it to `/initialpose` for ICP.

3. Enter a goal:

   ```text
   goal x y z
   ```

4. The planner publishes:

   - `/global_path`
   - `/global_path_3d`
   - `/global_path_marker`

## Planner Only

```bash
ros2 launch global_planner global_planner.launch start_source:=topic launch_open3d_picker:=true
```

## Direct Goal Topic

```bash
ros2 topic pub --once /goal_pose_3d geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.5, z: 0.2}, orientation: {w: 1.0}}}"
```
