# FAST_LIO 代码更新日志

## 2026-06-01 直接读取 ROS 2 Bag 建图

### 目标

为 FAST_LIO 增加直接读取 ROS 2 bag 的离线建图能力，使建图时不再必须单独运行 `ros2 bag play`。

### 使用方式

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml \
  rosbag_enable:=true \
  rosbag_path:=/path/to/your_ros2_bag \
  rviz:=false
```

### 新增参数

```yaml
rosbag:
    enable: false
    path: ""
    storage_id: "sqlite3"
    auto_shutdown: true
```

参数含义：

- `enable`：是否启用直接读取 bag 模式。
- `path`：ROS 2 bag 路径。
- `storage_id`：bag 存储后端，默认 `sqlite3`。
- `auto_shutdown`：bag 处理完成后是否自动退出节点。

### 涉及文件

- `src/FAST_LIO/src/laserMapping.cpp`
  - 增加 `rosbag2_cpp::Reader` 支持。
  - 直接从 bag 存储中反序列化 `/livox/lidar` 和 `/livox/imu`。
  - 复用已有 LiDAR/IMU 回调和 FAST_LIO 后端处理流程。
  - 直接读 bag 模式下不再订阅在线话题，并尽可能快地处理 bag 数据。
- `src/FAST_LIO/launch/mapping.launch.py`
  - 增加 `rosbag_enable`、`rosbag_path`、`rosbag_storage_id`、`rosbag_auto_shutdown` 启动参数。
- `src/FAST_LIO/config/mid360.yaml`
  - 增加带中文注释的 `rosbag` 参数。
- `src/FAST_LIO/CMakeLists.txt` 和 `src/FAST_LIO/package.xml`
  - 增加 `rosbag2_cpp` 和 `rosbag2_storage` 依赖。

### 构建验证

```bash
colcon build --packages-select fast_lio
```

结果：构建通过，仅出现已有的 CMake/Boost 依赖提示。

## 2026-05-30 输入 RPY 旋转参数

### 目标

修改 FAST_LIO 的输入预处理，使雷达点云和 IMU 数据在进入 FAST_LIO 后端前，可以按同一组可配置的 RPY 角进行旋转。

### YAML 参数

更新文件：

- `src/FAST_LIO/config/mid360.yaml`
- `install/fast_lio/share/fast_lio/config/mid360.yaml`

新增参数：

```yaml
preprocess:
    input_rotation_rpy_rad: [0.0, 0.485596, 0.0]
```

参数含义：

- 单位：弧度。
- 顺序：`[roll, pitch, yaw]`。
- 旋转约定：`R = Rz(yaw) * Ry(pitch) * Rx(roll)`。
- 当前值：roll 为 `0.0`，pitch 为 `0.485596`，yaw 为 `0.0`。

### 旋转公式

对输入向量 `(x, y, z)`，代码先根据 RPY 生成旋转矩阵，然后执行：

```text
[x', y', z']^T = Rz(yaw) * Ry(pitch) * Rx(roll) * [x, y, z]^T
```

其中：

```text
roll, pitch, yaw = preprocess.input_rotation_rpy_rad
```

### 坐标系转换逻辑说明

这次坐标转换属于“输入数据预旋转”，作用在雷达点云和 IMU 原始数据进入 FAST_LIO 后端之前。

它解决的问题是：如果雷达和 IMU 的驱动输出坐标系相对 FAST_LIO 期望的输入坐标系有统一安装角偏差，可以先用同一个 RPY 旋转把两类数据一起转到新的输入坐标系。这样 FAST_LIO 后端看到的点云、加速度、角速度仍然在同一个一致坐标系下。

该转换和 `mapping.extrinsic_R`、`mapping.extrinsic_T` 不同：

- `preprocess.input_rotation_rpy_rad` 是对原始传感器输入做统一预处理。
- `mapping.extrinsic_R` 和 `mapping.extrinsic_T` 是 FAST_LIO 内部使用的 LiDAR-IMU 外参。
- 如果只旋转点云、不旋转 IMU，点云运动补偿和 IMU 预积分使用的坐标方向会不一致，容易导致畸变补偿错误或位姿估计异常。
- 因此当前实现中，雷达点云、IMU `linear_acceleration`、IMU `angular_velocity` 都使用同一个旋转矩阵。

### 点云坐标转换位置

点云转换在 `src/FAST_LIO/src/preprocess.cpp` 中完成。

核心函数：

```cpp
void Preprocess::set_input_rotation_rpy(double roll_rad, double pitch_rad, double yaw_rad)
```

该函数根据 YAML 中的 `[roll, pitch, yaw]` 生成 3x3 旋转矩阵：

```text
R = Rz(yaw) * Ry(pitch) * Rx(roll)
```

点云逐点转换函数：

```cpp
void Preprocess::rotate_input(double x, double y, double z, PointType &pt) const
```

该函数执行：

```text
pt = R * [x, y, z]^T
```

已接入的点云路径：

- Livox `CustomMsg` 特征提取路径。
- Livox `CustomMsg` 普通面点路径。
- MID360 `PointCloud2` 路径。

当前 `mid360.yaml` 中 `lidar_type: 1`，所以实际运行主要走 Livox `CustomMsg` 路径；保留 MID360 `PointCloud2` 路径是为了后续切换输入类型时仍然生效。

### IMU 坐标转换位置

IMU 转换在 `src/FAST_LIO/src/laserMapping.cpp` 中完成。

核心函数：

```cpp
void set_input_rotation_rpy(double roll_rad, double pitch_rad, double yaw_rad)
```

该函数生成和点云预处理完全相同的 3x3 旋转矩阵。

IMU 向量转换函数：

```cpp
inline void rotate_input(double x, double y, double z, geometry_msgs::msg::Vector3 &vec)
```

在 `imu_cbk(...)` 回调中，代码会对下面两个 IMU 三维向量执行同一旋转：

```cpp
linear_acceleration
angular_velocity
```

时间戳、协方差、消息入队逻辑不变。

### 参数读取和同步

参数读取在 `src/FAST_LIO/src/laserMapping.cpp` 的节点初始化阶段完成：

```cpp
preprocess.input_rotation_rpy_rad
```

读取后会同时更新：

- `laserMapping.cpp` 中 IMU 回调用的旋转矩阵。
- `Preprocess` 对象中点云预处理用的旋转矩阵。

启动时会打印：

```text
input RPY rotation [roll, pitch, yaw] rad
```

### 涉及文件

#### `src/FAST_LIO/src/preprocess.h`

在 `Preprocess` 类中增加雷达输入 RPY 旋转支持：

- `set_input_rotation_rpy(double roll_rad, double pitch_rad, double yaw_rad)`
- `rotate_input(double x, double y, double z, PointType &pt) const`
- 缓存 3x3 输入旋转矩阵：
  - `input_rotation[9]`

#### `src/FAST_LIO/src/preprocess.cpp`

增加可配置点云 RPY 旋转实现。

构造函数默认初始化为 `0.0 rad`，因此其他配置文件即使没有设置该参数，也不会影响原行为。

旋转已应用到：

- Livox `CustomMsg` 开启特征提取路径。
- Livox `CustomMsg` 普通面点路径。
- MID360 `PointCloud2` 路径。

当前 `mid360.yaml` 使用 `lidar_type: 1`，所以实际会走 Livox `CustomMsg` 路径；同时保留 MID360 `PointCloud2` 路径支持，方便后续切换配置。

#### `src/FAST_LIO/src/laserMapping.cpp`

增加 IMU 输入 RPY 旋转状态：

- `input_rotation_rpy_rad`
- `input_rotation[9]`
- `set_input_rotation_rpy(double roll_rad, double pitch_rad, double yaw_rad)`
- `rotate_input(...)`

声明并读取 ROS 参数：

```cpp
preprocess.input_rotation_rpy_rad
```

读取参数后：

- 更新 IMU 侧旋转矩阵。
- 将同一组 RPY 角传给 `Preprocess` 对象。
- 启动日志打印实际读取值：

```text
input RPY rotation [roll, pitch, yaw] rad
```

IMU 入队前会同步旋转：

- `linear_acceleration`
- `angular_velocity`

IMU 时间戳处理逻辑保持不变。

### 构建验证

```bash
colcon build --packages-select fast_lio
```

结果：构建通过，仅出现 Boost 全局占位符弃用提示。

## 2026-05-30 YAML 中文注释

### 目标

为 `src/FAST_LIO/config/mid360.yaml` 中已有参数补充中文行内注释，方便后续调参。

### 说明

- `install/fast_lio/share/fast_lio/config/mid360.yaml` 在当前工作区中解析到同一个源文件，因此安装目录视图会自动显示同样注释。
- 本步骤只修改注释，不改变参数值。

## 2026-05-30 地图高度过滤

### 目标

为 FAST_LIO 增加可配置地图高度过滤，只输出或保存 `h_min` 到 `h_max` 范围内的地图点。

### YAML 参数

```yaml
mapping:
    height_filter_en: false
    h_min: -1000000000.0
    h_max: 1000000000.0
```

参数含义：

- `height_filter_en`：是否开启地图高度过滤。
- `h_min`：世界坐标系 `z` 方向高度下限，单位 m。
- `h_max`：世界坐标系 `z` 方向高度上限，单位 m。

### 初始实现

涉及文件：

- `src/FAST_LIO/src/laserMapping.cpp`
  - 增加高度过滤参数和启动日志。
  - 增加 `pass_map_height_filter(...)`。
  - 在初始化 ikd-tree 地图时应用过滤。
  - 在增量加入 ikd-tree 地图时应用过滤。
  - 在累计 `/Laser_map` 和保存地图缓存时应用过滤。
- `src/FAST_LIO/config/mid360.yaml`
  - 在 `mapping` 下增加带中文注释的高度过滤参数。

### 注意

该初始实现会过滤 FAST_LIO 用于 scan-to-map 匹配的内部局部地图。如果高度范围过窄，可能移除地面、墙面、天花板或其他关键几何约束，从而导致漂移。

### 构建验证

```bash
colcon build --packages-select fast_lio
```

结果：构建通过，仅出现 Boost 全局占位符弃用提示。

## 2026-06-01 高度过滤稳定性修正

### 问题现象

开启高度过滤后，如果同时过滤 ikd-tree 局部地图，FAST_LIO 可能发生漂移；高度范围包含全部点云时则不会漂移。

### 原因

FAST_LIO 的位姿估计依赖当前点云和 ikd-tree 局部地图之间的 scan-to-map 匹配。若按高度裁掉内部局部地图：

- 可用平面约束会减少。
- 最近邻平面拟合可能变稀疏或产生偏差。
- 地面、墙面、天花板或竖直结构被裁掉后，EKF 更新约束变弱。
- 最终可能出现里程计漂移。

### 修正方案

将“定位用局部地图”和“最终输出地图”分离：

- `mapping.height_filter_en`：控制输出/保存地图是否按高度过滤。
- `mapping.height_filter_local_map_en`：控制是否同时过滤 FAST_LIO 内部定位用局部地图。

推荐配置：

```yaml
mapping:
    height_filter_en: true
    height_filter_local_map_en: false
    h_min: -10.0
    h_max: 10.0
```

这样内部 ikd-tree 仍使用完整点云保证定位稳定，而 `/Laser_map` 和保存地图只输出 `h_min` 到 `h_max` 范围内的点。

### 涉及文件

- `src/FAST_LIO/src/laserMapping.cpp`
  - 增加 `map_height_filter_local_map_en`。
  - 增加 `pass_local_map_height_filter(...)`。
  - 初始化 ikd-tree 和增量加点时使用局部地图过滤开关。
  - `/Laser_map` 发布和保存地图缓存继续使用输出地图过滤。
- `src/FAST_LIO/config/mid360.yaml`
  - 增加 `height_filter_local_map_en` 中文注释参数。

### 构建验证

```bash
colcon build --packages-select fast_lio
```

结果：构建通过，仅出现 Boost 全局占位符弃用提示。
