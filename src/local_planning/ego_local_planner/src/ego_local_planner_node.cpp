#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/time.h>
#include <tf2/utils.h>
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

namespace ego_local_planner
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high)
{
  return std::max(low, std::min(high, value));
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

std::string trim(const std::string & input)
{
  std::size_t first = 0;
  while (first < input.size() && std::isspace(static_cast<unsigned char>(input[first]))) {
    ++first;
  }
  std::size_t last = input.size();
  while (last > first && std::isspace(static_cast<unsigned char>(input[last - 1]))) {
    --last;
  }
  return input.substr(first, last - first);
}

std::vector<std::string> splitTopicList(const std::string & topics)
{
  std::vector<std::string> out;
  std::stringstream ss(topics);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trim(item);
    if (!item.empty() && std::find(out.begin(), out.end(), item) == out.end()) {
      out.push_back(item);
    }
  }
  return out;
}

std::string joinTopicList(const std::vector<std::string> & topics)
{
  std::ostringstream ss;
  for (std::size_t i = 0; i < topics.size(); ++i) {
    if (i > 0) {
      ss << ",";
    }
    ss << topics[i];
  }
  return ss.str();
}

std::string remapFrameName(const std::string & frame, const std::string & remaps)
{
  for (const auto & item : splitTopicList(remaps)) {
    const auto separator = item.find(':');
    if (separator == std::string::npos) {
      continue;
    }
    const std::string from = trim(item.substr(0, separator));
    const std::string to = trim(item.substr(separator + 1));
    if (!from.empty() && !to.empty() && frame == from) {
      return to;
    }
  }
  return frame;
}

bool containsName(const std::vector<std::string> & names, const std::string & value)
{
  return !value.empty() && std::find(names.begin(), names.end(), value) != names.end();
}

builtin_interfaces::msg::Duration durationFromSeconds(double seconds)
{
  seconds = std::max(0.0, seconds);
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<uint32_t>(
    std::round((seconds - static_cast<double>(duration.sec)) * 1.0e9));
  if (duration.nanosec >= 1000000000U) {
    duration.sec += 1;
    duration.nanosec -= 1000000000U;
  }
  return duration;
}

geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

Eigen::Vector3d transformPoint(
  const geometry_msgs::msg::TransformStamped & transform,
  const Eigen::Vector3d & point)
{
  const auto & t = transform.transform.translation;
  const auto & q_msg = transform.transform.rotation;
  const Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  return q * point + Eigen::Vector3d(t.x, t.y, t.z);
}

rclcpp::Duration periodFromFrequency(double frequency)
{
  frequency = std::max(0.1, frequency);
  return rclcpp::Duration::from_seconds(1.0 / frequency);
}

}  // namespace

struct Pose3D
{
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  double yaw{0.0};
};

struct Trajectory
{
  std::vector<Eigen::Vector3d> points;
  rclcpp::Time stamp;
  bool collision_free{false};
};

struct VoxelKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & key) const
  {
    std::size_t seed = 0;
    auto combine = [&seed](int value) {
      seed ^= std::hash<int>{}(value) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    };
    combine(key.x);
    combine(key.y);
    combine(key.z);
    return seed;
  }
};

struct GridKey
{
  int x{0};
  int y{0};

  bool operator==(const GridKey & other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator!=(const GridKey & other) const
  {
    return !(*this == other);
  }
};

struct GridKeyHash
{
  std::size_t operator()(const GridKey & key) const
  {
    std::size_t seed = 0;
    seed ^= std::hash<int>{}(key.x) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
    return seed;
  }
};

class EgoLocalPlannerNode : public rclcpp::Node
{
public:
  EgoLocalPlannerNode()
  : Node("ego_local_planner_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    readParameters();

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    global_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic_, rclcpp::QoS(1),
      std::bind(&EgoLocalPlannerNode::globalPathCallback, this, std::placeholders::_1));
    for (const auto & topic : pointcloud_topics_) {
      pointcloud_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&EgoLocalPlannerNode::pointCloudCallback, this, std::placeholders::_1)));
    }
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&EgoLocalPlannerNode::odomCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, rclcpp::QoS(10));
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
      trajectory_topic_, rclcpp::QoS(1));
    trajectory_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      trajectory_marker_topic_, rclcpp::QoS(1));
    map_vis_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      map_vis_topic_, rclcpp::QoS(1));
    target_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      target_marker_topic_, rclcpp::QoS(1));
    status_pub_ = create_publisher<std_msgs::msg::String>(
      status_topic_, rclcpp::QoS(1).transient_local().reliable());
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, rclcpp::QoS(10));

    replan_timer_ = create_wall_timer(
      periodFromFrequency(replan_frequency_).to_chrono<std::chrono::nanoseconds>(),
      std::bind(&EgoLocalPlannerNode::replanCallback, this));
    control_timer_ = create_wall_timer(
      periodFromFrequency(control_frequency_).to_chrono<std::chrono::nanoseconds>(),
      std::bind(&EgoLocalPlannerNode::controlCallback, this));
    collision_timer_ = create_wall_timer(
      periodFromFrequency(collision_check_frequency_).to_chrono<std::chrono::nanoseconds>(),
      std::bind(&EgoLocalPlannerNode::collisionCallback, this));
    debug_timer_ = create_wall_timer(500ms, std::bind(&EgoLocalPlannerNode::debugCallback, this));

    setStatus("WAITING_FOR_PATH");
    RCLCPP_INFO(
      get_logger(),
      "EGO local planner adapter ready: path=%s clouds=%s cmd=%s",
      global_path_topic_.c_str(), joinTopicList(pointcloud_topics_).c_str(),
      cmd_vel_topic_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("base_frame_candidates", "base_link,base_footprint,odin1_base_link");
    declare_parameter<std::string>("lidar_frame", "livox_frame");

    declare_parameter<std::string>("global_path_topic", "/planned_path");
    declare_parameter<std::string>("pointcloud_topic", "/livox/lidar");
    declare_parameter<std::string>(
      "additional_pointcloud_topics", "/cloud_registered,/cloud_registered_body,/mid360");
    declare_parameter<std::string>("pointcloud_frame_remaps", "body:livox_frame");
    declare_parameter<std::string>("pointcloud_map_frame_aliases", "camera_init");
    declare_parameter<bool>("require_fresh_cloud", true);
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    declare_parameter<std::string>("ego_local_trajectory_topic", "/ego_local_trajectory");
    declare_parameter<std::string>(
      "ego_local_trajectory_marker_topic", "/ego_local_trajectory_marker");
    declare_parameter<std::string>("ego_local_map_vis_topic", "/ego_local_map_vis");
    declare_parameter<std::string>("ego_target_marker_topic", "/ego_target_marker");
    declare_parameter<std::string>("status_topic", "/ego_local_planner/status");
    declare_parameter<std::string>("debug_topic", "/ego_local_planner/debug");

    declare_parameter<std::string>("robot_model", "ground_omni");
    declare_parameter<bool>("allow_z_motion", false);
    declare_parameter<bool>("cloud_in_sensor_frame", true);

    declare_parameter<double>("local_map_radius", 5.0);
    declare_parameter<double>("local_map_height", 2.0);
    declare_parameter<double>("voxel_resolution", 0.15);
    declare_parameter<double>("inflation_radius", 0.35);
    declare_parameter<double>("robot_clear_radius", 0.65);
    declare_parameter<double>("astar_search_margin", 2.0);

    declare_parameter<bool>("remove_ground_points", true);
    declare_parameter<bool>("ground_relative_to_base", true);
    declare_parameter<double>("ground_z_threshold", 0.08);
    declare_parameter<double>("self_filter_radius", 0.45);
    declare_parameter<double>("dynamic_obstacle_timeout", 0.5);

    declare_parameter<double>("local_target_lookahead", 2.0);
    declare_parameter<double>("min_local_target_lookahead", 0.8);
    declare_parameter<double>("goal_reached_tolerance", 0.35);
    declare_parameter<bool>("z_following_enabled", true);
    declare_parameter<double>("max_allowed_z_jump", 0.35);

    declare_parameter<double>("max_vel_x", 0.6);
    declare_parameter<double>("max_vel_y", 0.4);
    declare_parameter<double>("max_acc_x", 0.5);
    declare_parameter<double>("max_acc_y", 0.5);
    declare_parameter<double>("max_yaw_rate", 1.0);
    declare_parameter<double>("max_yaw_acc", 1.5);

    declare_parameter<double>("traj_tracking_lookahead_time", 0.5);
    declare_parameter<double>("traj_tracking_lookahead_distance", 0.6);
    declare_parameter<double>("kp_x", 0.8);
    declare_parameter<double>("kp_y", 0.8);
    declare_parameter<double>("kp_yaw", 1.2);
    declare_parameter<double>("min_cmd_vel", 0.03);
    declare_parameter<double>("max_cmd_vel_x", 0.6);
    declare_parameter<double>("max_cmd_vel_y", 0.4);
    declare_parameter<double>("max_cmd_wz", 1.0);

    declare_parameter<double>("replan_frequency", 5.0);
    declare_parameter<double>("control_frequency", 20.0);
    declare_parameter<double>("collision_check_frequency", 10.0);
    declare_parameter<bool>("replan_on_dynamic_obstacle", true);
    declare_parameter<bool>("replan_when_path_blocked", true);
    declare_parameter<double>("path_block_check_distance", 2.0);
  }

  void readParameters()
  {
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    base_frame_candidates_param_ = get_parameter("base_frame_candidates").as_string();
    lidar_frame_ = get_parameter("lidar_frame").as_string();

    global_path_topic_ = get_parameter("global_path_topic").as_string();
    pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();
    additional_pointcloud_topics_ = get_parameter("additional_pointcloud_topics").as_string();
    pointcloud_frame_remaps_ = get_parameter("pointcloud_frame_remaps").as_string();
    pointcloud_map_frame_aliases_param_ =
      get_parameter("pointcloud_map_frame_aliases").as_string();
    require_fresh_cloud_ = get_parameter("require_fresh_cloud").as_bool();
    odom_topic_ = get_parameter("odom_topic").as_string();
    cmd_vel_topic_ = get_parameter("cmd_vel_topic").as_string();
    pointcloud_topics_.clear();
    if (!pointcloud_topic_.empty()) {
      pointcloud_topics_.push_back(pointcloud_topic_);
    }
    for (const auto & topic : splitTopicList(additional_pointcloud_topics_)) {
      if (std::find(pointcloud_topics_.begin(), pointcloud_topics_.end(), topic) ==
        pointcloud_topics_.end())
      {
        pointcloud_topics_.push_back(topic);
      }
    }
    pointcloud_map_frame_aliases_ = splitTopicList(pointcloud_map_frame_aliases_param_);
    if (std::find(
        pointcloud_map_frame_aliases_.begin(), pointcloud_map_frame_aliases_.end(),
        map_frame_) == pointcloud_map_frame_aliases_.end())
    {
      pointcloud_map_frame_aliases_.push_back(map_frame_);
    }
    base_frame_candidates_.clear();
    if (!base_frame_.empty()) {
      base_frame_candidates_.push_back(base_frame_);
    }
    for (const auto & frame : splitTopicList(base_frame_candidates_param_)) {
      if (std::find(base_frame_candidates_.begin(), base_frame_candidates_.end(), frame) ==
        base_frame_candidates_.end())
      {
        base_frame_candidates_.push_back(frame);
      }
    }

    trajectory_topic_ = get_parameter("ego_local_trajectory_topic").as_string();
    trajectory_marker_topic_ = get_parameter("ego_local_trajectory_marker_topic").as_string();
    map_vis_topic_ = get_parameter("ego_local_map_vis_topic").as_string();
    target_marker_topic_ = get_parameter("ego_target_marker_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    debug_topic_ = get_parameter("debug_topic").as_string();

    robot_model_ = get_parameter("robot_model").as_string();
    allow_z_motion_ = get_parameter("allow_z_motion").as_bool();
    cloud_in_sensor_frame_ = get_parameter("cloud_in_sensor_frame").as_bool();

    local_map_radius_ = get_parameter("local_map_radius").as_double();
    local_map_height_ = get_parameter("local_map_height").as_double();
    voxel_resolution_ = std::max(0.03, get_parameter("voxel_resolution").as_double());
    inflation_radius_ = std::max(0.0, get_parameter("inflation_radius").as_double());
    robot_clear_radius_ = std::max(0.0, get_parameter("robot_clear_radius").as_double());
    astar_search_margin_ = std::max(0.5, get_parameter("astar_search_margin").as_double());

    remove_ground_points_ = get_parameter("remove_ground_points").as_bool();
    ground_relative_to_base_ = get_parameter("ground_relative_to_base").as_bool();
    ground_z_threshold_ = get_parameter("ground_z_threshold").as_double();
    self_filter_radius_ = get_parameter("self_filter_radius").as_double();
    dynamic_obstacle_timeout_ = get_parameter("dynamic_obstacle_timeout").as_double();

    local_target_lookahead_ = get_parameter("local_target_lookahead").as_double();
    min_local_target_lookahead_ = get_parameter("min_local_target_lookahead").as_double();
    goal_reached_tolerance_ = get_parameter("goal_reached_tolerance").as_double();
    z_following_enabled_ = get_parameter("z_following_enabled").as_bool();
    max_allowed_z_jump_ = get_parameter("max_allowed_z_jump").as_double();

    max_vel_x_ = get_parameter("max_vel_x").as_double();
    max_vel_y_ = get_parameter("max_vel_y").as_double();
    max_acc_x_ = get_parameter("max_acc_x").as_double();
    max_acc_y_ = get_parameter("max_acc_y").as_double();
    max_yaw_rate_ = get_parameter("max_yaw_rate").as_double();
    max_yaw_acc_ = get_parameter("max_yaw_acc").as_double();

    traj_tracking_lookahead_time_ = get_parameter("traj_tracking_lookahead_time").as_double();
    traj_tracking_lookahead_distance_ =
      get_parameter("traj_tracking_lookahead_distance").as_double();
    kp_x_ = get_parameter("kp_x").as_double();
    kp_y_ = get_parameter("kp_y").as_double();
    kp_yaw_ = get_parameter("kp_yaw").as_double();
    min_cmd_vel_ = get_parameter("min_cmd_vel").as_double();
    max_cmd_vel_x_ = get_parameter("max_cmd_vel_x").as_double();
    max_cmd_vel_y_ = get_parameter("max_cmd_vel_y").as_double();
    max_cmd_wz_ = get_parameter("max_cmd_wz").as_double();

    replan_frequency_ = get_parameter("replan_frequency").as_double();
    control_frequency_ = get_parameter("control_frequency").as_double();
    collision_check_frequency_ = get_parameter("collision_check_frequency").as_double();
    replan_on_dynamic_obstacle_ = get_parameter("replan_on_dynamic_obstacle").as_bool();
    replan_when_path_blocked_ = get_parameter("replan_when_path_blocked").as_bool();
    path_block_check_distance_ = get_parameter("path_block_check_distance").as_double();
  }

  void globalPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    const std::string path_frame = msg->header.frame_id.empty() ? map_frame_ : msg->header.frame_id;
    geometry_msgs::msg::TransformStamped path_to_map;
    const bool transform_path = path_frame != map_frame_;
    if (transform_path && !lookupTransform(map_frame_, path_frame, path_to_map)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cannot transform global path from '%s' to '%s'.",
        path_frame.c_str(), map_frame_.c_str());
      return;
    }

    std::vector<Eigen::Vector3d> path;
    path.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      Eigen::Vector3d point(
        pose.pose.position.x,
        pose.pose.position.y,
        pose.pose.position.z);
      if (transform_path) {
        point = transformPoint(path_to_map, point);
      }
      path.push_back(point);
    }

    global_path_ = std::move(path);
    global_path_frame_ = map_frame_;
    has_path_ = !global_path_.empty();
    needs_replan_ = true;

    if (has_path_) {
      setStatus((!require_fresh_cloud_ || has_cloud_) ? "REPLAN" : "WAITING_FOR_CLOUD");
    } else {
      active_traj_.points.clear();
      setStatus("WAITING_FOR_PATH");
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
    has_odom_ = true;
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    Pose3D base_pose;
    if (!getCurrentPose(base_pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cannot update EGO local map because current pose is unavailable.");
      return;
    }

    const std::string raw_cloud_frame =
      msg->header.frame_id.empty() ? lidar_frame_ : msg->header.frame_id;
    std::string cloud_frame = remapFrameName(raw_cloud_frame, pointcloud_frame_remaps_);
    if (containsName(pointcloud_map_frame_aliases_, cloud_frame)) {
      cloud_frame = map_frame_;
    }

    geometry_msgs::msg::TransformStamped cloud_to_map;
    const bool need_transform = cloud_frame != map_frame_;
    if (need_transform && !lookupTransform(map_frame_, cloud_frame, cloud_to_map)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cannot transform point cloud from '%s' to '%s'.",
        cloud_frame.c_str(), map_frame_.c_str());
      return;
    }

    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
    std::unordered_set<GridKey, GridKeyHash> occupied_xy;
    std::vector<Eigen::Vector3d> raw_points;
    raw_points.reserve(msg->width * msg->height);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
      if (!std::isfinite(point.x()) || !std::isfinite(point.y()) || !std::isfinite(point.z())) {
        continue;
      }

      if (need_transform) {
        point = transformPoint(cloud_to_map, point);
      }

      const double dx = point.x() - base_pose.position.x();
      const double dy = point.y() - base_pose.position.y();
      const double dz = point.z() - base_pose.position.z();
      if (std::hypot(dx, dy) > local_map_radius_) {
        continue;
      }
      if (std::abs(dz) > local_map_height_) {
        continue;
      }
      if (std::hypot(dx, dy) < self_filter_radius_) {
        continue;
      }
      if (remove_ground_points_) {
        const double ground_reference = ground_relative_to_base_ ? base_pose.position.z() : 0.0;
        if (point.z() - ground_reference < ground_z_threshold_) {
          continue;
        }
      }

      raw_points.push_back(point);
      insertInflatedPoint(point, occupied_voxels, occupied_xy);
    }

    clearRobotFootprint(base_pose.position, occupied_voxels, occupied_xy);

    raw_obstacle_points_ = std::move(raw_points);
    occupied_voxels_ = std::move(occupied_voxels);
    occupied_xy_ = std::move(occupied_xy);
    has_cloud_ = true;
    last_cloud_time_ = now();
    needs_replan_ = needs_replan_ || replan_on_dynamic_obstacle_;

    publishLocalMap();
  }

  void replanCallback()
  {
    clearExpiredCloud();

    Pose3D current_pose;
    if (!getCurrentPose(current_pose)) {
      publishZeroCommand();
      setStatus("FAILURE");
      return;
    }

    if (!has_path_ || global_path_.empty()) {
      publishZeroCommand();
      setStatus("WAITING_FOR_PATH");
      return;
    }

    if (require_fresh_cloud_ && !isCloudFresh()) {
      publishZeroCommand();
      setStatus("WAITING_FOR_CLOUD");
      return;
    }

    if (isGoalReached(current_pose.position)) {
      active_traj_.points.clear();
      publishZeroCommand();
      setStatus("GOAL_REACHED");
      return;
    }

    const bool path_blocked = replan_when_path_blocked_ &&
      isPathBlocked(current_pose.position, path_block_check_distance_);
    const bool trajectory_blocked = !active_traj_.points.empty() &&
      !isTrajectoryCollisionFree(active_traj_);

    if (!needs_replan_ && !path_blocked && !trajectory_blocked) {
      return;
    }

    setStatus(path_blocked || trajectory_blocked ? "REPLAN" : "PLANNING");

    const Eigen::Vector3d local_target =
      selectLocalTargetFromGlobalPath(current_pose.position, global_path_);
    last_local_target_ = local_target;
    publishTargetMarker(local_target);

    Trajectory new_traj;
    if (planLocalTrajectory(current_pose.position, local_target, new_traj)) {
      active_traj_ = std::move(new_traj);
      active_traj_.stamp = now();
      needs_replan_ = false;
      trajectory_collision_ = false;
      publishTrajectory(active_traj_);
      setStatus("TRACKING");
      return;
    }

    active_traj_.points.clear();
    publishZeroCommand();
    setStatus(path_blocked ? "BLOCKED" : "FAILURE");
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "EGO local replan failed: %s. Waiting for a clear local corridor or global replan.",
      last_plan_failure_reason_.c_str());
  }

  void controlCallback()
  {
    clearExpiredCloud();

    Pose3D current_pose;
    if (!getCurrentPose(current_pose)) {
      publishZeroCommand();
      return;
    }

    if (has_path_ && isGoalReached(current_pose.position)) {
      active_traj_.points.clear();
      publishZeroCommand();
      setStatus("GOAL_REACHED");
      return;
    }

    if (active_traj_.points.empty()) {
      publishZeroCommand();
      return;
    }

    if (!isTrajectoryCollisionFree(active_traj_)) {
      trajectory_collision_ = true;
      needs_replan_ = true;
      publishZeroCommand();
      setStatus("REPLAN");
      return;
    }

    geometry_msgs::msg::Twist cmd;
    if (!computeCmdVelFromLocalTrajectory(active_traj_, current_pose, cmd)) {
      publishZeroCommand();
      return;
    }

    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd = applyCommandLimits(cmd);
    cmd_pub_->publish(cmd);
    last_cmd_ = cmd;
    last_cmd_time_ = now();
    last_cmd_valid_ = true;
    setStatus("TRACKING");
  }

  void collisionCallback()
  {
    if (active_traj_.points.empty()) {
      return;
    }

    if (!isTrajectoryCollisionFree(active_traj_)) {
      trajectory_collision_ = true;
      needs_replan_ = true;
      publishZeroCommand();
      setStatus("REPLAN");
      return;
    }

    Pose3D current_pose;
    if (replan_when_path_blocked_ && getCurrentPose(current_pose) &&
      isPathBlocked(current_pose.position, path_block_check_distance_))
    {
      needs_replan_ = true;
      setStatus("REPLAN");
    }
  }

  void debugCallback()
  {
    Pose3D pose;
    const bool has_pose = getCurrentPose(pose);

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3);
    ss << "status=" << status_;
    if (has_pose) {
      ss << " pose=(" << pose.position.x() << ", " << pose.position.y() << ", "
        << pose.position.z() << ", yaw=" << pose.yaw << ")";
    } else {
      ss << " pose=unavailable";
    }
    ss << " local_target=(" << last_local_target_.x() << ", " << last_local_target_.y()
      << ", " << last_local_target_.z() << ")";
    ss << " cloud_fresh=" << (isCloudFresh() ? "true" : "false");
    ss << " local_map_points=" << raw_obstacle_points_.size();
    ss << " inflated_voxels=" << occupied_voxels_.size();
    ss << " needs_replan=" << (needs_replan_ ? "true" : "false");
    ss << " trajectory_collision=" << (trajectory_collision_ ? "true" : "false");
    ss << " last_plan_failure=\"" << last_plan_failure_reason_ << "\"";
    ss << " cmd_vel=(" << last_cmd_.linear.x << ", " << last_cmd_.linear.y
      << ", " << last_cmd_.angular.z << ")";

    std_msgs::msg::String msg;
    msg.data = ss.str();
    debug_pub_->publish(msg);
  }

  Eigen::Vector3d selectLocalTargetFromGlobalPath(
    const Eigen::Vector3d & current_pos,
    const std::vector<Eigen::Vector3d> & global_path) const
  {
    if (global_path.empty()) {
      return current_pos;
    }

    std::size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < global_path.size(); ++i) {
      const double distance = (global_path[i].head<2>() - current_pos.head<2>()).norm();
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = i;
      }
    }

    const Eigen::Vector3d final_goal = global_path.back();
    if ((final_goal.head<2>() - current_pos.head<2>()).norm() <=
      std::max(goal_reached_tolerance_, local_target_lookahead_))
    {
      return clampTargetZ(final_goal, current_pos.z());
    }

    const double lookahead =
      std::max(min_local_target_lookahead_, local_target_lookahead_);
    double accumulated = 0.0;
    Eigen::Vector3d selected = global_path.back();
    for (std::size_t i = nearest_index; i + 1 < global_path.size(); ++i) {
      accumulated += (global_path[i + 1].head<2>() - global_path[i].head<2>()).norm();
      selected = global_path[i + 1];
      if (accumulated >= lookahead) {
        break;
      }
    }

    return clampTargetZ(selected, current_pos.z());
  }

  bool computeCmdVelFromLocalTrajectory(
    const Trajectory & traj,
    const Pose3D & current_pose,
    geometry_msgs::msg::Twist & cmd)
  {
    if (traj.points.size() < 2) {
      return false;
    }

    std::size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < traj.points.size(); ++i) {
      const double distance = (traj.points[i].head<2>() - current_pose.position.head<2>()).norm();
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = i;
      }
    }

    const double target_distance = std::max(
      traj_tracking_lookahead_distance_,
      traj_tracking_lookahead_time_ * std::max(max_vel_x_, max_vel_y_) * 0.5);
    double accumulated = 0.0;
    std::size_t target_index = nearest_index;
    for (std::size_t i = nearest_index; i + 1 < traj.points.size(); ++i) {
      accumulated += (traj.points[i + 1].head<2>() - traj.points[i].head<2>()).norm();
      target_index = i + 1;
      if (accumulated >= target_distance) {
        break;
      }
    }

    const Eigen::Vector3d target = traj.points[target_index];
    const Eigen::Vector2d error_world =
      target.head<2>() - current_pose.position.head<2>();

    const double cos_yaw = std::cos(current_pose.yaw);
    const double sin_yaw = std::sin(current_pose.yaw);
    const double error_x_base = cos_yaw * error_world.x() + sin_yaw * error_world.y();
    const double error_y_base = -sin_yaw * error_world.x() + cos_yaw * error_world.y();

    cmd.linear.x = clamp(kp_x_ * error_x_base, -max_cmd_vel_x_, max_cmd_vel_x_);
    cmd.linear.y = clamp(kp_y_ * error_y_base, -max_cmd_vel_y_, max_cmd_vel_y_);

    const Eigen::Vector2d heading_world = target.head<2>() - current_pose.position.head<2>();
    const double desired_yaw = heading_world.norm() > 0.05 ?
      std::atan2(heading_world.y(), heading_world.x()) : current_pose.yaw;
    const double yaw_error = normalizeAngle(desired_yaw - current_pose.yaw);
    cmd.angular.z = clamp(kp_yaw_ * yaw_error, -max_cmd_wz_, max_cmd_wz_);

    const double planar_speed = std::hypot(cmd.linear.x, cmd.linear.y);
    if (planar_speed > 1.0e-4 && planar_speed < min_cmd_vel_) {
      const double scale = min_cmd_vel_ / planar_speed;
      cmd.linear.x *= scale;
      cmd.linear.y *= scale;
    }

    return true;
  }

  bool planLocalTrajectory(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & target,
    Trajectory & traj)
  {
    last_plan_failure_reason_.clear();
    std::vector<Eigen::Vector3d> path;
    if (isSegmentCollisionFree(start, target)) {
      path = {start, target};
    } else if (!runGridAStar(start, target, path)) {
      if (last_plan_failure_reason_.empty()) {
        std::ostringstream ss;
        ss << "astar_failed start_occupied="
          << (isGridOccupied(toGridKey(start.x(), start.y())) ? "true" : "false")
          << " target_occupied="
          << (isGridOccupied(toGridKey(target.x(), target.y())) ? "true" : "false")
          << " occupied_xy=" << occupied_xy_.size();
        last_plan_failure_reason_ = ss.str();
      }
      return false;
    }

    path = shortcutPath(path);
    path = resamplePath(path, 0.20);

    if (path.size() < 2) {
      last_plan_failure_reason_ = "local trajectory has fewer than 2 points after resampling";
      return false;
    }

    Trajectory candidate;
    candidate.points = std::move(path);
    candidate.stamp = now();
    candidate.collision_free = isTrajectoryCollisionFree(candidate);
    if (!candidate.collision_free) {
      last_plan_failure_reason_ = "candidate trajectory became occupied during final collision check";
      return false;
    }

    last_plan_failure_reason_.clear();
    traj = std::move(candidate);
    return true;
  }

  bool runGridAStar(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & target,
    std::vector<Eigen::Vector3d> & path)
  {
    const GridKey start_key = toGridKey(start.x(), start.y());
    GridKey goal_key = toGridKey(target.x(), target.y());
    if (isGridOccupied(goal_key)) {
      const auto free_goal = findNearestFreeGrid(goal_key);
      if (!free_goal.has_value()) {
        last_plan_failure_reason_ = "local target and nearby cells are occupied";
        return false;
      }
      goal_key = *free_goal;
    }

    const int radius_cells = static_cast<int>(std::ceil(local_map_radius_ / voxel_resolution_));
    const int margin_cells = static_cast<int>(std::ceil(astar_search_margin_ / voxel_resolution_));
    const int min_x = std::min(start_key.x, goal_key.x) - margin_cells;
    const int max_x = std::max(start_key.x, goal_key.x) + margin_cells;
    const int min_y = std::min(start_key.y, goal_key.y) - margin_cells;
    const int max_y = std::max(start_key.y, goal_key.y) + margin_cells;

    struct NodeRecord
    {
      double g{std::numeric_limits<double>::infinity()};
      GridKey parent;
      bool has_parent{false};
      bool closed{false};
    };
    struct QueueItem
    {
      GridKey key;
      double f{0.0};
      double g{0.0};
      bool operator>(const QueueItem & other) const
      {
        return f > other.f;
      }
    };

    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
    std::unordered_map<GridKey, NodeRecord, GridKeyHash> records;
    auto heuristic = [&](const GridKey & key) {
      return std::hypot(
        static_cast<double>(key.x - goal_key.x),
        static_cast<double>(key.y - goal_key.y)) * voxel_resolution_;
    };

    records[start_key].g = 0.0;
    open.push({start_key, heuristic(start_key), 0.0});

    const std::vector<GridKey> neighbors = {
      {1, 0}, {-1, 0}, {0, 1}, {0, -1},
      {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };

    int iterations = 0;
    const int max_iterations = 20000;
    while (!open.empty() && iterations++ < max_iterations) {
      const QueueItem current = open.top();
      open.pop();

      auto & current_record = records[current.key];
      if (current_record.closed) {
        continue;
      }
      current_record.closed = true;

      if (current.key == goal_key) {
        reconstructGridPath(records, start_key, goal_key, start.z(), path);
        return path.size() >= 2;
      }

      for (const auto & step : neighbors) {
        const GridKey next{current.key.x + step.x, current.key.y + step.y};
        if (next.x < min_x || next.x > max_x || next.y < min_y || next.y > max_y) {
          continue;
        }
        if (std::abs(next.x - start_key.x) > radius_cells ||
          std::abs(next.y - start_key.y) > radius_cells)
        {
          continue;
        }
        if (next != start_key && isGridOccupied(next)) {
          continue;
        }

        const double step_cost = std::hypot(static_cast<double>(step.x), static_cast<double>(step.y)) *
          voxel_resolution_;
        const double new_g = current_record.g + step_cost;
        auto & next_record = records[next];
        if (next_record.closed || new_g >= next_record.g) {
          continue;
        }

        next_record.g = new_g;
        next_record.parent = current.key;
        next_record.has_parent = true;
        open.push({next, new_g + heuristic(next), new_g});
      }
    }

    std::ostringstream ss;
    ss << "astar_search_exhausted iterations=" << iterations
      << " open_empty=" << (open.empty() ? "true" : "false")
      << " start_occupied=" << (isGridOccupied(start_key) ? "true" : "false")
      << " goal_occupied=" << (isGridOccupied(goal_key) ? "true" : "false")
      << " occupied_xy=" << occupied_xy_.size()
      << " search_margin=" << astar_search_margin_;
    last_plan_failure_reason_ = ss.str();
    return false;
  }

  template<typename Records>
  void reconstructGridPath(
    const Records & records,
    const GridKey & start_key,
    const GridKey & goal_key,
    double fallback_z,
    std::vector<Eigen::Vector3d> & path)
  {
    std::vector<GridKey> keys;
    GridKey current = goal_key;
    keys.push_back(current);

    while (!(current == start_key)) {
      const auto iter = records.find(current);
      if (iter == records.end() || !iter->second.has_parent) {
        path.clear();
        return;
      }
      current = iter->second.parent;
      keys.push_back(current);
    }

    std::reverse(keys.begin(), keys.end());
    path.clear();
    path.reserve(keys.size());
    for (const auto & key : keys) {
      const Eigen::Vector2d xy = gridKeyToWorld(key);
      const double z = referenceZ(xy.x(), xy.y(), fallback_z);
      path.emplace_back(xy.x(), xy.y(), z);
    }
  }

  std::vector<Eigen::Vector3d> shortcutPath(const std::vector<Eigen::Vector3d> & path)
  {
    if (path.size() <= 2) {
      return path;
    }

    std::vector<Eigen::Vector3d> out;
    out.push_back(path.front());

    std::size_t index = 0;
    while (index + 1 < path.size()) {
      std::size_t next_index = index + 1;
      for (std::size_t candidate = path.size() - 1; candidate > index; --candidate) {
        if (isSegmentCollisionFree(path[index], path[candidate])) {
          next_index = candidate;
          break;
        }
      }
      out.push_back(path[next_index]);
      index = next_index;
    }

    return out;
  }

  std::vector<Eigen::Vector3d> resamplePath(
    const std::vector<Eigen::Vector3d> & path,
    double spacing) const
  {
    if (path.empty()) {
      return {};
    }

    spacing = std::max(0.05, spacing);
    std::vector<Eigen::Vector3d> out;
    out.push_back(path.front());

    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const Eigen::Vector3d start = path[i];
      const Eigen::Vector3d end = path[i + 1];
      const double distance = (end.head<2>() - start.head<2>()).norm();
      const int steps = std::max(1, static_cast<int>(std::ceil(distance / spacing)));
      for (int step = 1; step <= steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(steps);
        Eigen::Vector3d point = (1.0 - t) * start + t * end;
        point.z() = referenceZ(point.x(), point.y(), point.z());
        out.push_back(point);
      }
    }

    return out;
  }

  bool isSegmentCollisionFree(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & end) const
  {
    const double distance = (end.head<2>() - start.head<2>()).norm();
    const int steps = std::max(1, static_cast<int>(std::ceil(distance / (voxel_resolution_ * 0.5))));
    for (int i = 0; i <= steps; ++i) {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      Eigen::Vector3d point = (1.0 - t) * start + t * end;
      point.z() = referenceZ(point.x(), point.y(), point.z());
      if (!isPoseCollisionFree(point)) {
        return false;
      }
    }
    return true;
  }

  bool isTrajectoryCollisionFree(const Trajectory & traj) const
  {
    if (traj.points.empty()) {
      return false;
    }
    for (const auto & point : traj.points) {
      if (!isPoseCollisionFree(point)) {
        return false;
      }
    }
    for (std::size_t i = 0; i + 1 < traj.points.size(); ++i) {
      if (!isSegmentCollisionFree(traj.points[i], traj.points[i + 1])) {
        return false;
      }
    }
    return true;
  }

  bool isPoseCollisionFree(const Eigen::Vector3d & pose) const
  {
    if (require_fresh_cloud_ && !isCloudFresh()) {
      return false;
    }
    return !isGridOccupied(toGridKey(pose.x(), pose.y()));
  }

  bool isPathBlocked(const Eigen::Vector3d & current_pos, double check_distance) const
  {
    if (global_path_.empty()) {
      return false;
    }

    std::size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < global_path_.size(); ++i) {
      const double distance = (global_path_[i].head<2>() - current_pos.head<2>()).norm();
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = i;
      }
    }

    double accumulated = 0.0;
    Eigen::Vector3d last = current_pos;
    for (std::size_t i = nearest_index; i < global_path_.size(); ++i) {
      const Eigen::Vector3d point = global_path_[i];
      accumulated += (point.head<2>() - last.head<2>()).norm();
      if (!isPoseCollisionFree(point)) {
        return true;
      }
      if (accumulated >= check_distance) {
        break;
      }
      last = point;
    }
    return false;
  }

  bool isGoalReached(const Eigen::Vector3d & current_pos) const
  {
    if (global_path_.empty()) {
      return false;
    }
    return (global_path_.back().head<2>() - current_pos.head<2>()).norm() <=
           goal_reached_tolerance_;
  }

  bool isGridOccupied(const GridKey & key) const
  {
    return occupied_xy_.find(key) != occupied_xy_.end();
  }

  std::optional<GridKey> findNearestFreeGrid(const GridKey & preferred) const
  {
    const int max_radius = static_cast<int>(std::ceil(1.5 / voxel_resolution_));
    for (int radius = 1; radius <= max_radius; ++radius) {
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          if (std::max(std::abs(dx), std::abs(dy)) != radius) {
            continue;
          }
          const GridKey key{preferred.x + dx, preferred.y + dy};
          if (!isGridOccupied(key)) {
            return key;
          }
        }
      }
    }
    return std::nullopt;
  }

  void insertInflatedPoint(
    const Eigen::Vector3d & point,
    std::unordered_set<VoxelKey, VoxelKeyHash> & occupied_voxels,
    std::unordered_set<GridKey, GridKeyHash> & occupied_xy) const
  {
    const VoxelKey base = toVoxelKey(point);
    const int radius_cells =
      static_cast<int>(std::ceil(inflation_radius_ / voxel_resolution_));

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        const double planar_distance = std::hypot(
          static_cast<double>(dx) * voxel_resolution_,
          static_cast<double>(dy) * voxel_resolution_);
        if (planar_distance <= inflation_radius_ + 1.0e-9) {
          occupied_xy.insert({base.x + dx, base.y + dy});
        }
        for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
          const double distance = std::sqrt(
            static_cast<double>(dx * dx + dy * dy + dz * dz)) * voxel_resolution_;
          if (distance <= inflation_radius_ + 1.0e-9) {
            occupied_voxels.insert({base.x + dx, base.y + dy, base.z + dz});
          }
        }
      }
    }
  }

  void clearRobotFootprint(
    const Eigen::Vector3d & base_position,
    std::unordered_set<VoxelKey, VoxelKeyHash> & occupied_voxels,
    std::unordered_set<GridKey, GridKeyHash> & occupied_xy) const
  {
    const GridKey center_xy = toGridKey(base_position.x(), base_position.y());
    const VoxelKey center_voxel = toVoxelKey(base_position);
    const int radius_cells =
      static_cast<int>(std::ceil(robot_clear_radius_ / voxel_resolution_));
    const int z_cells =
      static_cast<int>(std::ceil(local_map_height_ / voxel_resolution_));

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        const double distance = std::hypot(
          static_cast<double>(dx) * voxel_resolution_,
          static_cast<double>(dy) * voxel_resolution_);
        if (distance > robot_clear_radius_) {
          continue;
        }

        occupied_xy.erase({center_xy.x + dx, center_xy.y + dy});
        for (int dz = -z_cells; dz <= z_cells; ++dz) {
          occupied_voxels.erase({center_voxel.x + dx, center_voxel.y + dy, center_voxel.z + dz});
        }
      }
    }
  }

  VoxelKey toVoxelKey(const Eigen::Vector3d & point) const
  {
    return {
      static_cast<int>(std::floor(point.x() / voxel_resolution_)),
      static_cast<int>(std::floor(point.y() / voxel_resolution_)),
      static_cast<int>(std::floor(point.z() / voxel_resolution_)),
    };
  }

  GridKey toGridKey(double x, double y) const
  {
    return {
      static_cast<int>(std::floor(x / voxel_resolution_)),
      static_cast<int>(std::floor(y / voxel_resolution_)),
    };
  }

  Eigen::Vector2d gridKeyToWorld(const GridKey & key) const
  {
    return Eigen::Vector2d(
      (static_cast<double>(key.x) + 0.5) * voxel_resolution_,
      (static_cast<double>(key.y) + 0.5) * voxel_resolution_);
  }

  Eigen::Vector3d voxelKeyToWorld(const VoxelKey & key) const
  {
    return Eigen::Vector3d(
      (static_cast<double>(key.x) + 0.5) * voxel_resolution_,
      (static_cast<double>(key.y) + 0.5) * voxel_resolution_,
      (static_cast<double>(key.z) + 0.5) * voxel_resolution_);
  }

  double referenceZ(double x, double y, double fallback_z) const
  {
    if (!z_following_enabled_ || global_path_.empty()) {
      return fallback_z;
    }

    double best_distance = std::numeric_limits<double>::infinity();
    double best_z = fallback_z;
    const Eigen::Vector2d query(x, y);

    for (std::size_t i = 0; i + 1 < global_path_.size(); ++i) {
      const Eigen::Vector2d a = global_path_[i].head<2>();
      const Eigen::Vector2d b = global_path_[i + 1].head<2>();
      const Eigen::Vector2d ab = b - a;
      const double length_sq = ab.squaredNorm();
      const double t = length_sq > 1.0e-8 ?
        clamp((query - a).dot(ab) / length_sq, 0.0, 1.0) : 0.0;
      const Eigen::Vector2d projection = a + t * ab;
      const double distance = (query - projection).norm();
      if (distance < best_distance) {
        best_distance = distance;
        best_z = (1.0 - t) * global_path_[i].z() + t * global_path_[i + 1].z();
      }
    }

    if (global_path_.size() == 1) {
      best_z = global_path_.front().z();
    }

    return best_z;
  }

  Eigen::Vector3d clampTargetZ(const Eigen::Vector3d & target, double current_z) const
  {
    if (!z_following_enabled_) {
      return Eigen::Vector3d(target.x(), target.y(), current_z);
    }
    Eigen::Vector3d out = target;
    out.z() = clamp(out.z(), current_z - max_allowed_z_jump_, current_z + max_allowed_z_jump_);
    return out;
  }

  bool getCurrentPose(Pose3D & pose) const
  {
    geometry_msgs::msg::TransformStamped transform;
    for (const auto & base_frame : base_frame_candidates_) {
      if (!lookupTransform(map_frame_, base_frame, transform)) {
        continue;
      }
      pose.position = Eigen::Vector3d(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);
      pose.yaw = tf2::getYaw(transform.transform.rotation);
      return true;
    }

    if (!has_odom_) {
      return false;
    }

    geometry_msgs::msg::PoseStamped odom_pose;
    odom_pose.header = last_odom_.header;
    if (odom_pose.header.frame_id.empty()) {
      odom_pose.header.frame_id = odom_frame_;
    }
    odom_pose.pose = last_odom_.pose.pose;

    if (odom_pose.header.frame_id == map_frame_) {
      pose.position = Eigen::Vector3d(
        odom_pose.pose.position.x,
        odom_pose.pose.position.y,
        odom_pose.pose.position.z);
      pose.yaw = tf2::getYaw(odom_pose.pose.orientation);
      return true;
    }

    geometry_msgs::msg::TransformStamped odom_to_map;
    if (!lookupTransform(map_frame_, odom_pose.header.frame_id, odom_to_map)) {
      return false;
    }

    geometry_msgs::msg::PoseStamped map_pose;
    tf2::doTransform(odom_pose, map_pose, odom_to_map);
    pose.position = Eigen::Vector3d(
      map_pose.pose.position.x,
      map_pose.pose.position.y,
      map_pose.pose.position.z);
    pose.yaw = tf2::getYaw(map_pose.pose.orientation);
    return true;
  }

  bool lookupTransform(
    const std::string & target,
    const std::string & source,
    geometry_msgs::msg::TransformStamped & transform) const
  {
    try {
      transform = tf_buffer_.lookupTransform(target, source, tf2::TimePointZero);
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  bool isCloudFresh() const
  {
    if (!has_cloud_) {
      return !require_fresh_cloud_;
    }
    return (now() - last_cloud_time_).seconds() <= dynamic_obstacle_timeout_;
  }

  void clearExpiredCloud()
  {
    if (has_cloud_ && (now() - last_cloud_time_).seconds() > dynamic_obstacle_timeout_) {
      has_cloud_ = false;
      raw_obstacle_points_.clear();
      occupied_voxels_.clear();
      occupied_xy_.clear();
      needs_replan_ = true;
      publishLocalMap();
    }
  }

  geometry_msgs::msg::Twist applyCommandLimits(const geometry_msgs::msg::Twist & desired)
  {
    geometry_msgs::msg::Twist limited = desired;
    limited.linear.x = clamp(limited.linear.x, -max_cmd_vel_x_, max_cmd_vel_x_);
    limited.linear.y = clamp(limited.linear.y, -max_cmd_vel_y_, max_cmd_vel_y_);
    limited.angular.z = clamp(limited.angular.z, -max_cmd_wz_, max_cmd_wz_);

    const rclcpp::Time time_now = now();
    const double dt = last_cmd_valid_ ?
      std::max(1.0 / control_frequency_, (time_now - last_cmd_time_).seconds()) :
      1.0 / control_frequency_;

    auto limit_delta = [dt](double target, double current, double max_acc) {
      const double delta = clamp(target - current, -max_acc * dt, max_acc * dt);
      return current + delta;
    };

    limited.linear.x = limit_delta(limited.linear.x, last_cmd_.linear.x, max_acc_x_);
    limited.linear.y = limit_delta(limited.linear.y, last_cmd_.linear.y, max_acc_y_);
    limited.angular.z = limit_delta(limited.angular.z, last_cmd_.angular.z, max_yaw_acc_);
    limited.angular.z = clamp(limited.angular.z, -max_yaw_rate_, max_yaw_rate_);
    return limited;
  }

  void publishZeroCommand()
  {
    geometry_msgs::msg::Twist cmd;
    cmd_pub_->publish(cmd);
    last_cmd_ = cmd;
    last_cmd_time_ = now();
    last_cmd_valid_ = true;
  }

  void publishTrajectory(const Trajectory & traj)
  {
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = map_frame_;
    path_msg.poses.reserve(traj.points.size());

    for (const auto & point : traj.points) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = point.x();
      pose.pose.position.y = point.y();
      pose.pose.position.z = point.z();
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }
    trajectory_pub_->publish(path_msg);

    visualization_msgs::msg::Marker marker;
    marker.header = path_msg.header;
    marker.ns = "ego_local_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.06;
    marker.color.r = 0.1F;
    marker.color.g = 0.8F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    marker.lifetime = durationFromSeconds(0.5);
    for (const auto & point : traj.points) {
      marker.points.push_back(toPointMsg(point));
    }
    trajectory_marker_pub_->publish(marker);
  }

  void publishTargetMarker(const Eigen::Vector3d & target)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_frame_;
    marker.ns = "ego_local_target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = toPointMsg(target);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.r = 1.0F;
    marker.color.g = 0.7F;
    marker.color.b = 0.1F;
    marker.color.a = 1.0F;
    marker.lifetime = durationFromSeconds(0.5);
    target_marker_pub_->publish(marker);
  }

  void publishLocalMap()
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(occupied_voxels_.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    for (const auto & key : occupied_voxels_) {
      const Eigen::Vector3d point = voxelKeyToWorld(key);
      *iter_x = static_cast<float>(point.x());
      *iter_y = static_cast<float>(point.y());
      *iter_z = static_cast<float>(point.z());
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    map_vis_pub_->publish(msg);
  }

  void setStatus(const std::string & status)
  {
    if (status == status_) {
      return;
    }
    status_ = status;
    std_msgs::msg::String msg;
    msg.data = status_;
    status_pub_->publish(msg);
  }

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string base_frame_candidates_param_;
  std::string lidar_frame_;
  std::string global_path_topic_;
  std::string pointcloud_topic_;
  std::string odom_topic_;
  std::string cmd_vel_topic_;
  std::string trajectory_topic_;
  std::string trajectory_marker_topic_;
  std::string map_vis_topic_;
  std::string target_marker_topic_;
  std::string status_topic_;
  std::string debug_topic_;
  std::string robot_model_;
  std::string additional_pointcloud_topics_;
  std::string pointcloud_frame_remaps_;
  std::string pointcloud_map_frame_aliases_param_;
  std::vector<std::string> pointcloud_topics_;
  std::vector<std::string> pointcloud_map_frame_aliases_;
  std::vector<std::string> base_frame_candidates_;

  bool allow_z_motion_{false};
  bool cloud_in_sensor_frame_{true};
  bool require_fresh_cloud_{true};
  double local_map_radius_{5.0};
  double local_map_height_{2.0};
  double voxel_resolution_{0.15};
  double inflation_radius_{0.35};
  double robot_clear_radius_{0.65};
  double astar_search_margin_{2.0};
  bool remove_ground_points_{true};
  bool ground_relative_to_base_{true};
  double ground_z_threshold_{0.08};
  double self_filter_radius_{0.45};
  double dynamic_obstacle_timeout_{0.5};
  double local_target_lookahead_{2.0};
  double min_local_target_lookahead_{0.8};
  double goal_reached_tolerance_{0.35};
  bool z_following_enabled_{true};
  double max_allowed_z_jump_{0.35};
  double max_vel_x_{0.6};
  double max_vel_y_{0.4};
  double max_acc_x_{0.5};
  double max_acc_y_{0.5};
  double max_yaw_rate_{1.0};
  double max_yaw_acc_{1.5};
  double traj_tracking_lookahead_time_{0.5};
  double traj_tracking_lookahead_distance_{0.6};
  double kp_x_{0.8};
  double kp_y_{0.8};
  double kp_yaw_{1.2};
  double min_cmd_vel_{0.03};
  double max_cmd_vel_x_{0.6};
  double max_cmd_vel_y_{0.4};
  double max_cmd_wz_{1.0};
  double replan_frequency_{5.0};
  double control_frequency_{20.0};
  double collision_check_frequency_{10.0};
  bool replan_on_dynamic_obstacle_{true};
  bool replan_when_path_blocked_{true};
  double path_block_check_distance_{2.0};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;

  rclcpp::TimerBase::SharedPtr replan_timer_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr collision_timer_;
  rclcpp::TimerBase::SharedPtr debug_timer_;

  std::vector<Eigen::Vector3d> global_path_;
  std::string global_path_frame_{"map"};
  bool has_path_{false};
  bool has_cloud_{false};
  bool has_odom_{false};
  nav_msgs::msg::Odometry last_odom_;
  rclcpp::Time last_cloud_time_{0, 0, RCL_ROS_TIME};

  std::vector<Eigen::Vector3d> raw_obstacle_points_;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels_;
  std::unordered_set<GridKey, GridKeyHash> occupied_xy_;
  Trajectory active_traj_;
  Eigen::Vector3d last_local_target_{Eigen::Vector3d::Zero()};
  bool needs_replan_{false};
  bool trajectory_collision_{false};
  std::string status_;
  std::string last_plan_failure_reason_;

  geometry_msgs::msg::Twist last_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool last_cmd_valid_{false};
};

}  // namespace ego_local_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ego_local_planner::EgoLocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
