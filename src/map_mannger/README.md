# map_mannger

地图管理工具包。当前包含 `bin2pcd_node`，用于把 `float32` 二进制点云地图转换为 PCD 并自动保存。

默认 bin 格式为每个点 4 个 `float32`：

```text
x y z intensity
```

如果地图只有 `x y z` 三个字段，可以把 `fields_per_point` 设为 `3`，节点会把 intensity 写为 `0`。

## 使用

```bash
ros2 run map_mannger bin2pcd_node --ros-args \
  -p input_bin_path:=/path/to/map.bin \
  -p output_pcd_path:=/path/to/map.pcd
```

也可以通过 launch 启动：

```bash
ros2 launch map_mannger bin2pcd.launch.py \
  input_bin_path:=/path/to/map.bin \
  output_pcd_path:=/path/to/map.pcd
```

如果不设置 `output_pcd_path`，节点会自动在输入文件旁边保存同名 `.pcd` 文件。
