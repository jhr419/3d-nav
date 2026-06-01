#include <algorithm>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "dddmr_fastlio_adapter/point_types.hpp"

namespace
{

// 将 ROS Pose 转成 Eigen 仿射变换，后续统一用矩阵做坐标系运算。
Eigen::Affine3d poseToAffine(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Translation3d translation(
    pose.position.x,
    pose.position.y,
    pose.position.z);
  Eigen::Quaterniond rotation(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  return translation * rotation.normalized();
}

// 从仿射变换中提取 roll/pitch/yaw，供 DDDMR 的 PointTypePose 字段使用。
Eigen::Vector3d affineToRpy(const Eigen::Affine3d & affine)
{
  const Eigen::Quaterniond q(affine.rotation());
  tf2::Quaternion tf_q(q.x(), q.y(), q.z(), q.w());
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(tf_q.normalized()).getRPY(roll, pitch, yaw);
  return Eigen::Vector3d(roll, pitch, yaw);
}

// 将 Eigen 仿射变换转回 ROS Pose，用于发布 PoseArray 和 TF。
geometry_msgs::msg::Pose affineToPose(const Eigen::Affine3d & affine)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3d t = affine.translation();
  const Eigen::Quaterniond q(affine.rotation());
  pose.position.x = t.x();
  pose.position.y = t.y();
  pose.position.z = t.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

// 把关键帧位姿编码成带时间和序号的 PCL 点，兼容 DDDMR/LeGO-LOAM 的位姿点云格式。
PointTypePose affineToPointTypePose(
  const Eigen::Affine3d & affine,
  const float intensity,
  const double time)
{
  PointTypePose point;
  const Eigen::Vector3d t = affine.translation();
  const Eigen::Vector3d euler = affineToRpy(affine);
  point.x = static_cast<float>(t.x());
  point.y = static_cast<float>(t.y());
  point.z = static_cast<float>(t.z());
  point.roll = static_cast<float>(euler.x());
  point.pitch = static_cast<float>(euler.y());
  point.yaw = static_cast<float>(euler.z());
  point.intensity = intensity;
  point.time = time;
  return point;
}

// 计算两个位姿之间的平移距离，用于判断是否需要新增关键帧。
double poseDistance(const Eigen::Affine3d & a, const Eigen::Affine3d & b)
{
  return (a.translation() - b.translation()).norm();
}

// 计算两个位姿之间的旋转角度差，用于关键帧角度阈值判断。
double poseAngle(const Eigen::Affine3d & a, const Eigen::Affine3d & b)
{
  Eigen::Quaterniond qa(a.rotation());
  Eigen::Quaterniond qb(b.rotation());
  return qa.angularDistance(qb);
}

// 对点云做体素降采样，控制单帧和全局地图的点数规模。
void downsample(
  pcl::PointCloud<PointType>::Ptr cloud,
  const double leaf_size)
{
  if (!cloud || cloud->empty() || leaf_size <= 0.0) {
    return;
  }
  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.setInputCloud(cloud);
  pcl::PointCloud<PointType> filtered;
  voxel.filter(filtered);
  *cloud = filtered;
}

}  // namespace

class FastLioDddmrAdapter : public rclcpp::Node
{
public:
  FastLioDddmrAdapter()
  : Node("fastlio_dddmr_adapter")
  {
    // 输入/输出话题、坐标系、保存目录和发布频率等运行参数。
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    body_cloud_topic_ = declare_parameter<std::string>("body_cloud_topic", "/cloud_registered_body");
    map_cloud_topic_ = declare_parameter<std::string>("map_cloud_topic", "/Laser_map");
    pose_source_ = declare_parameter<std::string>("pose_source", "odom");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    child_frame_ = declare_parameter<std::string>("child_frame", "base_link");
    pose_frame_ = declare_parameter<std::string>("pose_frame", base_frame_);
    mapping_dir_ = declare_parameter<std::string>("mapping_dir", "/tmp/dddmr_fastlio_map");
    save_on_shutdown_ = declare_parameter<bool>("save_on_shutdown", false);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_period_s_ = declare_parameter<double>("publish_period_s", 1.0);
    pose_lookup_timeout_s_ = declare_parameter<double>("pose_lookup_timeout_s", 0.05);

    keyframe_distance_ = declare_parameter<double>("keyframe_distance", 0.5);
    keyframe_angle_ = declare_parameter<double>("keyframe_angle", 0.5);
    keyframe_min_points_ = declare_parameter<int>("keyframe_min_points", 100);
    keyframe_cloud_leaf_size_ = declare_parameter<double>("keyframe_cloud_leaf_size", 0.15);
    global_map_leaf_size_ = declare_parameter<double>("global_map_leaf_size", 0.25);
    max_range_ = declare_parameter<double>("max_range", 80.0);
    min_range_ = declare_parameter<double>("min_range", 0.3);
    ground_z_min_ = declare_parameter<double>("ground_z_min", -0.35);
    ground_z_max_ = declare_parameter<double>("ground_z_max", 0.25);
    ground_abs_slope_max_ = declare_parameter<double>("ground_abs_slope_max", 0.35);
    feature_min_z_ = declare_parameter<double>("feature_min_z", -0.2);
    keep_ground_in_features_ = declare_parameter<bool>("keep_ground_in_features", false);

    // base_to_body 描述机器人底盘坐标系到 FAST-LIO 点云机体系的外参。
    const auto base_to_body_xyz =
      declare_parameter<std::vector<double>>("base_to_body_xyz", {0.0, 0.0, 0.0});
    const auto base_to_body_rpy =
      declare_parameter<std::vector<double>>("base_to_body_rpy", {0.0, 0.0, 0.0});
    validateVectorParameter("base_to_body_xyz", base_to_body_xyz);
    validateVectorParameter("base_to_body_rpy", base_to_body_rpy);
    base_to_body_ = makeTransform(base_to_body_xyz, base_to_body_rpy);
    body_to_base_ = base_to_body_.inverse();

    // base_to_pose 描述底盘坐标系到 TF 位姿源坐标系的外参。
    const auto base_to_pose_xyz =
      declare_parameter<std::vector<double>>("base_to_pose_xyz", {0.0, 0.0, 0.0});
    const auto base_to_pose_rpy =
      declare_parameter<std::vector<double>>("base_to_pose_rpy", {0.0, 0.0, 0.0});
    validateVectorParameter("base_to_pose_xyz", base_to_pose_xyz);
    validateVectorParameter("base_to_pose_rpy", base_to_pose_rpy);
    base_to_pose_ = makeTransform(base_to_pose_xyz, base_to_pose_rpy);
    pose_to_base_ = base_to_pose_.inverse();

    if (pose_source_ != "odom" && pose_source_ != "tf" && pose_source_ != "auto") {
      RCLCPP_WARN(
        get_logger(),
        "pose_source should be 'odom', 'tf', or 'auto'; falling back to 'odom'.");
      pose_source_ = "odom";
    }

    // 发布 DDDMR/LeGO-LOAM 风格的地图、地面、地面边缘和关键帧位姿。
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_map", 1);
    ground_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_ground", 1);
    ground_edge_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("lego_loam_ground_edge", 1);
    keypose_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("cloud_keypose_6d", 1);
    keypose_array_pub_ = create_publisher<geometry_msgs::msg::PoseArray>("key_poses", 1);

    // 订阅 FAST-LIO 输出和保存触发信号，保存接口同时兼容 fastlio/glim 的命名。
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20, std::bind(&FastLioDddmrAdapter::odomCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      body_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FastLioDddmrAdapter::cloudCallback, this, std::placeholders::_1));
    map_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      map_cloud_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FastLioDddmrAdapter::mapCloudCallback, this, std::placeholders::_1));
    save_sub_ = create_subscription<std_msgs::msg::Bool>(
      "fastlio_dddmr_save_current_map", 1,
      std::bind(&FastLioDddmrAdapter::saveTopicCallback, this, std::placeholders::_1));
    save_glim_sub_ = create_subscription<std_msgs::msg::Bool>(
      "glim_dddmr_save_current_map", 1,
      std::bind(&FastLioDddmrAdapter::saveTopicCallback, this, std::placeholders::_1));
    save_srv_ = create_service<std_srvs::srv::Trigger>(
      "save_fastlio_dddmr_map",
      std::bind(
        &FastLioDddmrAdapter::saveServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2));
    save_glim_srv_ = create_service<std_srvs::srv::Trigger>(
      "save_glim_dddmr_map",
      std::bind(
        &FastLioDddmrAdapter::saveServiceCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    // TF buffer 用于按点云时间戳查询位姿；定时器周期性发布聚合后的地图输出。
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(std::max(0.1, publish_period_s_))),
      std::bind(&FastLioDddmrAdapter::publishOutputs, this));

    RCLCPP_INFO(
      get_logger(),
      "DDDMR SLAM adapter: pose_source=%s odom=%s cloud=%s save_dir=%s",
      pose_source_.c_str(), odom_topic_.c_str(), body_cloud_topic_.c_str(), mapping_dir_.c_str());
  }

  ~FastLioDddmrAdapter() override
  {
    if (save_on_shutdown_ && !keyframes_.empty()) {
      std::string message;
      saveMap(message);
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    }
  }

private:
  struct KeyFrame
  {
    // 单个关键帧保存位姿以及在 base 坐标系下切分后的特征/地面点云。
    double stamp = 0.0;
    Eigen::Affine3d map_to_base = Eigen::Affine3d::Identity();
    pcl::PointCloud<PointType>::Ptr feature{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr ground{new pcl::PointCloud<PointType>()};
    pcl::PointCloud<PointType>::Ptr ground_edge{new pcl::PointCloud<PointType>()};
  };

  void validateVectorParameter(
    const std::string & name,
    const std::vector<double> & value) const
  {
    if (!value.empty() && value.size() != 3) {
      RCLCPP_WARN(
        get_logger(),
        "%s should contain exactly 3 values; missing values will be treated as 0.",
        name.c_str());
    }
  }

  Eigen::Affine3d makeTransform(
    const std::vector<double> & xyz,
    const std::vector<double> & rpy)
  {
    // 参数不足 3 个值时用 0 补齐，避免 launch 参数缺项导致节点退出。
    const double x = xyz.size() > 0 ? xyz[0] : 0.0;
    const double y = xyz.size() > 1 ? xyz[1] : 0.0;
    const double z = xyz.size() > 2 ? xyz[2] : 0.0;
    const double roll = rpy.size() > 0 ? rpy[0] : 0.0;
    const double pitch = rpy.size() > 1 ? rpy[1] : 0.0;
    const double yaw = rpy.size() > 2 ? rpy[2] : 0.0;
    return Eigen::Translation3d(x, y, z) *
      Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    last_odom_ = *msg;
    have_odom_ = true;
    // FAST-LIO 里程计通常表示 map->body，这里换算成 map->base 供建图使用。
    const Eigen::Affine3d map_to_body = poseToAffine(msg->pose.pose);
    last_map_to_base_ = map_to_body * body_to_base_;
    if (publish_tf_) {
      publishBaseTf(msg->header.stamp, last_map_to_base_);
    }
  }

  void mapCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_fastlio_map_ = *msg;
    have_fastlio_map_ = true;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 每帧点云先取得同一时刻的 map->base 位姿，失败则等待位姿源就绪。
    Eigen::Affine3d map_to_base = Eigen::Affine3d::Identity();
    if (!getMapToBase(msg->header.stamp, map_to_base)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for SLAM pose source '%s' (odom=%s tf=%s->%s)",
        pose_source_.c_str(), odom_topic_.c_str(), map_frame_.c_str(), pose_frame_.c_str());
      return;
    }

    pcl::PointCloud<PointType>::Ptr cloud_body(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*msg, *cloud_body);
    if (static_cast<int>(cloud_body->size()) < keyframe_min_points_) {
      return;
    }

    if (!shouldAddKeyFrame(map_to_base)) {
      return;
    }

    // 新关键帧中的点仍保存在 base 坐标系，重建全局地图时再统一变换到 map 坐标系。
    KeyFrame keyframe;
    keyframe.stamp = rclcpp::Time(msg->header.stamp).seconds();
    keyframe.map_to_base = map_to_base;
    splitCloudInBaseFrame(cloud_body, keyframe);
    if (static_cast<int>(keyframe.feature->size() + keyframe.ground->size()) < keyframe_min_points_) {
      return;
    }

    downsample(keyframe.feature, keyframe_cloud_leaf_size_);
    downsample(keyframe.ground, keyframe_cloud_leaf_size_);
    downsample(keyframe.ground_edge, keyframe_cloud_leaf_size_);
    keyframes_.push_back(keyframe);
    // 有新关键帧后，全局点云缓存失效，等发布或保存时再重建。
    dirty_global_clouds_ = true;

    RCLCPP_INFO(
      get_logger(),
      "DDDMR keyframe %zu: feature=%zu ground=%zu",
      keyframes_.size() - 1, keyframe.feature->size(), keyframe.ground->size());
  }

  bool getMapToBase(
    const builtin_interfaces::msg::Time & stamp,
    Eigen::Affine3d & map_to_base)
  {
    // auto 模式优先用 TF，查不到时回退到最近一帧里程计。
    if (pose_source_ == "tf" || pose_source_ == "auto") {
      if (lookupTfPose(stamp, map_to_base)) {
        return true;
      }
      if (pose_source_ == "tf") {
        return false;
      }
    }

    if (!have_odom_) {
      return false;
    }

    const Eigen::Affine3d map_to_body = poseToAffine(last_odom_.pose.pose);
    map_to_base = map_to_body * body_to_base_;
    return true;
  }

  bool lookupTfPose(
    const builtin_interfaces::msg::Time & stamp,
    Eigen::Affine3d & map_to_base)
  {
    if (!tf_buffer_) {
      return false;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_,
        pose_frame_,
        rclcpp::Time(stamp),
        rclcpp::Duration::from_seconds(std::max(0.0, pose_lookup_timeout_s_)));
      const Eigen::Affine3d map_to_pose =
        Eigen::Affine3d(tf2::transformToEigen(transform.transform));
      // TF 查到的是 map->pose_frame，需要通过外参换成 map->base。
      map_to_base = map_to_pose * pose_to_base_;
      last_map_to_base_ = map_to_base;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "TF pose lookup failed: %s", ex.what());
      return false;
    }
  }

  bool shouldAddKeyFrame(const Eigen::Affine3d & map_to_base) const
  {
    if (keyframes_.empty()) {
      return true;
    }
    // 平移或旋转任一变化超过阈值时，才保留为新关键帧。
    const auto & previous = keyframes_.back().map_to_base;
    return poseDistance(previous, map_to_base) >= keyframe_distance_ ||
      poseAngle(previous, map_to_base) >= keyframe_angle_;
  }

  void splitCloudInBaseFrame(
    const pcl::PointCloud<PointType>::Ptr & cloud_body,
    KeyFrame & keyframe) const
  {
    // 将 FAST-LIO body 点云转到 base 坐标系，并按高度/坡度粗略切分地面与障碍特征。
    for (const auto & p_body : cloud_body->points) {
      if (!std::isfinite(p_body.x) || !std::isfinite(p_body.y) || !std::isfinite(p_body.z)) {
        continue;
      }
      const double range = std::sqrt(
        p_body.x * p_body.x + p_body.y * p_body.y + p_body.z * p_body.z);
      if (range < min_range_ || range > max_range_) {
        continue;
      }
      const Eigen::Vector3d base_xyz =
        body_to_base_ * Eigen::Vector3d(p_body.x, p_body.y, p_body.z);
      PointType p_base;
      p_base.x = static_cast<float>(base_xyz.x());
      p_base.y = static_cast<float>(base_xyz.y());
      p_base.z = static_cast<float>(base_xyz.z());
      p_base.intensity = p_body.intensity;

      const double planar = std::hypot(p_base.x, p_base.y);
      const double slope = planar > 0.01 ? std::abs(p_base.z) / planar : std::abs(p_base.z);
      const bool is_ground =
        p_base.z >= ground_z_min_ &&
        p_base.z <= ground_z_max_ &&
        slope <= ground_abs_slope_max_;

      if (is_ground) {
        keyframe.ground->push_back(p_base);
        // 靠近地面高度上下边界的点作为 ground_edge，给后续算法保留地面边缘信息。
        if (std::abs(p_base.z - ground_z_max_) < 0.08 || std::abs(p_base.z - ground_z_min_) < 0.08) {
          keyframe.ground_edge->push_back(p_base);
        }
        if (keep_ground_in_features_) {
          keyframe.feature->push_back(p_base);
        }
      } else if (p_base.z >= feature_min_z_) {
        keyframe.feature->push_back(p_base);
      }
    }
  }

  void rebuildGlobalClouds()
  {
    // 从所有关键帧重新生成全局地图缓存，避免每次新增关键帧都立即做昂贵聚合。
    global_feature_.reset(new pcl::PointCloud<PointType>());
    global_ground_.reset(new pcl::PointCloud<PointType>());
    global_ground_edge_.reset(new pcl::PointCloud<PointType>());
    keypose_cloud_.reset(new pcl::PointCloud<PointTypePose>());

    for (size_t i = 0; i < keyframes_.size(); ++i) {
      const auto & keyframe = keyframes_[i];
      pcl::PointCloud<PointType> feature_map;
      pcl::PointCloud<PointType> ground_map;
      pcl::PointCloud<PointType> ground_edge_map;
      pcl::transformPointCloud(*keyframe.feature, feature_map, keyframe.map_to_base);
      pcl::transformPointCloud(*keyframe.ground, ground_map, keyframe.map_to_base);
      pcl::transformPointCloud(*keyframe.ground_edge, ground_edge_map, keyframe.map_to_base);
      // 关键帧内点云由 base 坐标系变换到 map 坐标系后累加。
      *global_feature_ += feature_map;
      *global_ground_ += ground_map;
      *global_ground_edge_ += ground_edge_map;
      keypose_cloud_->push_back(
        affineToPointTypePose(keyframe.map_to_base, static_cast<float>(i), keyframe.stamp));
    }

    downsample(global_feature_, global_map_leaf_size_);
    downsample(global_ground_, global_map_leaf_size_);
    downsample(global_ground_edge_, global_map_leaf_size_);
    dirty_global_clouds_ = false;
  }

  void publishOutputs()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // 只有关键帧变化后才重建全局点云，平时直接发布缓存。
    if (dirty_global_clouds_) {
      rebuildGlobalClouds();
    }

    const auto stamp = now();
    publishCloud(map_pub_, global_feature_, map_frame_, stamp);
    publishCloud(ground_pub_, global_ground_, map_frame_, stamp);
    publishCloud(ground_edge_pub_, global_ground_edge_, map_frame_, stamp);
    publishKeyPoses(stamp);
  }

  void publishCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub,
    const pcl::PointCloud<PointType>::Ptr & cloud,
    const std::string & frame,
    const rclcpp::Time & stamp) const
  {
    if (!cloud) {
      return;
    }
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = frame;
    msg.header.stamp = stamp;
    pub->publish(msg);
  }

  void publishKeyPoses(const rclcpp::Time & stamp)
  {
    // 同时发布 PCL 格式关键帧位姿和 ROS PoseArray，方便不同消费者订阅。
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*keypose_cloud_, cloud_msg);
    cloud_msg.header.frame_id = map_frame_;
    cloud_msg.header.stamp = stamp;
    keypose_pub_->publish(cloud_msg);

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = map_frame_;
    pose_array.header.stamp = stamp;
    pose_array.poses.reserve(keyframes_.size());
    for (const auto & keyframe : keyframes_) {
      pose_array.poses.push_back(affineToPose(keyframe.map_to_base));
    }
    keypose_array_pub_->publish(pose_array);
  }

  void publishBaseTf(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Affine3d & map_to_base)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = child_frame_;
    const auto pose = affineToPose(map_to_base);
    transform.transform.translation.x = pose.position.x;
    transform.transform.translation.y = pose.position.y;
    transform.transform.translation.z = pose.position.z;
    transform.transform.rotation = pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void saveTopicCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg->data) {
      return;
    }
    std::string message;
    const bool ok = saveMap(message);
    if (ok) {
      RCLCPP_INFO(get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "%s", message.c_str());
    }
  }

  void saveServiceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    response->success = saveMap(response->message);
  }

  bool saveMap(std::string & message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (keyframes_.empty()) {
      message = "No DDDMR keyframes collected yet.";
      return false;
    }
    if (dirty_global_clouds_) {
      rebuildGlobalClouds();
    }

    try {
      // 保存全局地图、地面、关键帧位姿，以及每个关键帧的局部点云。
      std::filesystem::create_directories(mapping_dir_);
      const std::string pcd_dir = mapping_dir_ + "/pcd";
      std::filesystem::create_directories(pcd_dir);

      pcl::io::savePCDFileASCII(mapping_dir_ + "/map.pcd", *global_feature_);
      pcl::io::savePCDFileASCII(mapping_dir_ + "/ground.pcd", *global_ground_);
      pcl::io::savePCDFileASCII(mapping_dir_ + "/poses.pcd", *keypose_cloud_);

      pcl::PointCloud<pcl::PointXYZ> edges;
      // edges.pcd 用连续关键帧索引构造简单轨迹边，满足 DDDMR 读取格式。
      for (size_t i = 1; i < keyframes_.size(); ++i) {
        pcl::PointXYZ edge;
        edge.x = static_cast<float>(i - 1);
        edge.y = static_cast<float>(i - 1);
        edge.z = static_cast<float>(i);
        edges.push_back(edge);
      }
      pcl::io::savePCDFileASCII(mapping_dir_ + "/edges.pcd", edges);

      for (size_t i = 0; i < keyframes_.size(); ++i) {
        pcl::io::savePCDFileASCII(
          pcd_dir + "/" + std::to_string(i) + "_feature.pcd",
          *keyframes_[i].feature);
        pcl::io::savePCDFileASCII(
          pcd_dir + "/" + std::to_string(i) + "_ground.pcd",
          *keyframes_[i].ground);
        pcl::io::savePCDFileASCII(
          pcd_dir + "/" + std::to_string(i) + "_surface.pcd",
          *keyframes_[i].ground);
      }
    } catch (const std::exception & ex) {
      message = std::string("Failed to save DDDMR SLAM map: ") + ex.what();
      return false;
    }

    message = "Saved DDDMR SLAM map to " + mapping_dir_ +
      " with " + std::to_string(keyframes_.size()) + " keyframes.";
    return true;
  }

  std::mutex mutex_;

  std::string odom_topic_;
  std::string body_cloud_topic_;
  std::string map_cloud_topic_;
  std::string pose_source_;
  std::string map_frame_;
  std::string base_frame_;
  std::string child_frame_;
  std::string pose_frame_;
  std::string mapping_dir_;
  bool save_on_shutdown_ = false;
  bool publish_tf_ = true;
  double publish_period_s_ = 1.0;
  double pose_lookup_timeout_s_ = 0.05;
  double keyframe_distance_ = 0.5;
  double keyframe_angle_ = 0.5;
  int keyframe_min_points_ = 100;
  double keyframe_cloud_leaf_size_ = 0.15;
  double global_map_leaf_size_ = 0.25;
  double max_range_ = 80.0;
  double min_range_ = 0.3;
  double ground_z_min_ = -0.35;
  double ground_z_max_ = 0.25;
  double ground_abs_slope_max_ = 0.35;
  double feature_min_z_ = -0.2;
  bool keep_ground_in_features_ = false;

  Eigen::Affine3d body_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d base_to_body_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d pose_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d base_to_pose_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d last_map_to_base_ = Eigen::Affine3d::Identity();

  nav_msgs::msg::Odometry last_odom_;
  sensor_msgs::msg::PointCloud2 latest_fastlio_map_;
  bool have_odom_ = false;
  bool have_fastlio_map_ = false;
  bool dirty_global_clouds_ = true;

  std::vector<KeyFrame> keyframes_;
  pcl::PointCloud<PointType>::Ptr global_feature_{new pcl::PointCloud<PointType>()};
  pcl::PointCloud<PointType>::Ptr global_ground_{new pcl::PointCloud<PointType>()};
  pcl::PointCloud<PointType>::Ptr global_ground_edge_{new pcl::PointCloud<PointType>()};
  pcl::PointCloud<PointTypePose>::Ptr keypose_cloud_{new pcl::PointCloud<PointTypePose>()};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr save_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr save_glim_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_glim_srv_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_edge_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr keypose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr keypose_array_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr save_timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FastLioDddmrAdapter>());
  rclcpp::shutdown();
  return 0;
}
