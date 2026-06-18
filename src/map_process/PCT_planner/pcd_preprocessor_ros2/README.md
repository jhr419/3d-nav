# pcd_preprocessor_ros2

ROS2 Humble Python package for leveling and rigidly transforming PCD point clouds before PCT-Planner tomography mapping.

PCT-Planner tomography assumes `z` is vertical and the `XY` plane is the horizontal navigation plane. If the input PCD is tilted, the same floor may be split across multiple height slices and produce unstable traversability cost. This node preprocesses the PCD, saves a corrected PCD, and publishes the result on `/processed_points`.

Open3D is required for `enable_auto_level: true`. Install it into the Python environment used by ROS2, for example:

```bash
/usr/bin/python3 -m pip install open3d tqdm
```

## Build

```bash
cd <your_3dnav_ws>
colcon build --symlink-install --packages-select pcd_preprocessor_ros2
source install/setup.bash
```

## Run

Edit `pcd_preprocessor_ros2/config/pcd_preprocessor.yaml`, then run:

```bash
ros2 launch pcd_preprocessor_ros2 pcd_preprocessor.launch.py
```

With RViz2:

```bash
ros2 launch pcd_preprocessor_ros2 pcd_preprocessor.launch.py rviz:=true
```

To use another YAML file:

```bash
ros2 launch pcd_preprocessor_ros2 pcd_preprocessor.launch.py config_file:=/path/to/pcd_preprocessor.yaml
```

## PCT-Planner Tomography Workflow

Save the processed PCD under:

```text
maps/
```

Then edit:

```text
src/map_process/PCT_planner/pct_planner_ros2/config/tomography.yaml
```

Set `pcd_file` to the output file name, for example:

```yaml
pcd_dir: maps
pcd_file: full_map_leveled.pcd
```

Then build/source the PCT-Planner ROS2 package if needed and run tomography normally.

## Main Parameters

- `input_pcd`: input PCD path.
- `output_pcd`: processed PCD output path.
- `enable_manual_transform`: apply manual rigid transform before auto leveling.
- `roll`, `pitch`, `yaw`: manual rotation in radians.
- `tx`, `ty`, `tz`: manual translation in meters.
- `enable_auto_level`: fit a ground plane with Open3D RANSAC and align its normal to `normal_target_axis`.
- `ground_percentile`: use points with `z` in the lowest percentile as ground candidates.
- `max_ground_points`: random sample cap for RANSAC candidates.
- `ransac_distance_threshold`, `ransac_n`, `ransac_num_iterations`: Open3D plane segmentation settings.
- `normal_target_axis`: target normal, normally `[0.0, 0.0, 1.0]`.
- `enable_ground_z_shift`: shift the leveled cloud so the selected ground reference equals `target_ground_z`.
- `ground_z_shift_mode`: `plane_inliers_median` is recommended when the global lowest point is not ground; `min` uses the global minimum `z`.
- `ground_z_shift_percentile`: percentile used by `plane_inliers_percentile`.
- `frame_id`: frame used for `/processed_points`, default `map`.

The manual transform is:

```text
p_out = Rz(yaw) * Ry(pitch) * Rx(roll) * p_in + t
```

Open3D is preferred for ASCII and binary PCD read/write and required for automatic leveling. If Open3D is missing, the node can still read ASCII and uncompressed binary PCD files and write ASCII XYZ PCD, but `enable_auto_level` must be `false`.
