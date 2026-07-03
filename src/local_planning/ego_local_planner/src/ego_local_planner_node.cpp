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
#include <visualization_msgs/msg/marker_array.hpp>

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
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          pointCloudCallback(msg, false);
        }));
    }
    auto static_cloud_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    for (const auto & topic : static_pointcloud_topics_) {
      static_pointcloud_subs_.push_back(create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, static_cloud_qos,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          pointCloudCallback(msg, true);
        }));
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
    candidate_trajectories_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      candidate_trajectories_marker_topic_, rclcpp::QoS(1));
    local_map_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      local_map_marker_topic_, rclcpp::QoS(1));
    collision_points_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      collision_points_marker_topic_, rclcpp::QoS(1));
    footprint_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      footprint_marker_topic_, rclcpp::QoS(1));
    cmd_vel_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      cmd_vel_marker_topic_, rclcpp::QoS(1));

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
    if (!static_pointcloud_topics_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "EGO local planner static obstacle clouds=%s",
        joinTopicList(static_pointcloud_topics_).c_str());
    }
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
    declare_parameter<std::string>("static_pointcloud_topics", "/global_points");
    declare_parameter<std::string>("pointcloud_frame_remaps", "body:livox_frame");
    declare_parameter<std::string>("pointcloud_map_frame_aliases", "camera_init");
    declare_parameter<bool>("require_fresh_cloud", true);
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    declare_parameter<std::string>("ego_local_trajectory_topic", "/ego_local_trajectory");
    declare_parameter<std::string>(
      "ego_local_trajectory_marker_topic", "/ego_local_trajectory_marker");
    declare_parameter<std::string>("ego_local_map_vis_topic", "/ego_local_map_vis");
    declare_parameter<std::string>("ego_target_marker_topic", "/ego_local_target_marker");
    declare_parameter<std::string>(
      "ego_candidate_trajectories_marker_topic", "/ego_candidate_trajectories_marker");
    declare_parameter<std::string>("ego_local_map_marker_topic", "/ego_local_map_marker");
    declare_parameter<std::string>("ego_collision_points_marker_topic", "/ego_collision_points_marker");
    declare_parameter<std::string>("ego_footprint_marker_topic", "/ego_footprint_marker");
    declare_parameter<std::string>("ego_cmd_vel_marker_topic", "/ego_cmd_vel_marker");
    declare_parameter<std::string>("status_topic", "/ego_local_planner/status");
    declare_parameter<std::string>("debug_topic", "/ego_debug_text");

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
    declare_parameter<bool>("ground_filter_use_path_z", true);
    declare_parameter<double>("self_filter_radius", 0.45);
    declare_parameter<double>("dynamic_obstacle_timeout", 0.5);

    declare_parameter<double>("local_target_lookahead", 2.0);
    declare_parameter<double>("min_local_target_lookahead", 0.8);
    declare_parameter<double>("goal_reached_tolerance", 0.35);
    declare_parameter<double>("goal_reached_z_tolerance", 0.45);
    declare_parameter<bool>("z_following_enabled", true);
    declare_parameter<double>("max_allowed_z_jump", 0.35);
    declare_parameter<bool>("z_aware_path_tracking", true);
    declare_parameter<double>("path_nearest_z_weight", 1.0);
    declare_parameter<double>("reference_z_weight", 1.0);
    declare_parameter<bool>("limit_local_target_by_z", true);
    declare_parameter<double>("local_target_max_z_delta", 0.35);

    declare_parameter<double>("max_vel_x", 0.6);
    declare_parameter<double>("max_vel_y", 0.4);
    declare_parameter<double>("max_acc_x", 0.5);
    declare_parameter<double>("max_acc_y", 0.5);
    declare_parameter<double>("max_yaw_rate", 1.0);
    declare_parameter<double>("max_yaw_acc", 1.5);

    declare_parameter<double>("traj_tracking_lookahead_time", 0.5);
    declare_parameter<double>("traj_tracking_lookahead_distance", 0.6);
    declare_parameter<bool>("line_of_sight_tracking", true);
    declare_parameter<double>("cmd_collision_check_time", 0.6);
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

    declare_parameter<bool>("use_global_path_risk", true);
    declare_parameter<bool>("allow_deviation_from_global_path", true);
    declare_parameter<double>("max_deviation_from_global_path", 0.8);
    declare_parameter<bool>("prefer_high_clearance_tracking", true);
    declare_parameter<double>("min_local_clearance", 0.35);
    declare_parameter<double>("preferred_local_clearance", 0.65);
    declare_parameter<double>("local_clearance_weight", 2.0);
    declare_parameter<double>("path_tracking_weight", 1.0);
    declare_parameter<bool>("slow_down_near_obstacle", true);
    declare_parameter<double>("obstacle_slow_distance", 0.8);
    declare_parameter<double>("obstacle_stop_distance", 0.35);
    declare_parameter<bool>("debug_visualization_enabled", true);
    declare_parameter<double>("footprint_height", 0.55);
    declare_parameter<double>("static_map_update_distance", 0.25);
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
    static_pointcloud_topics_param_ = get_parameter("static_pointcloud_topics").as_string();
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
    static_pointcloud_topics_ = splitTopicList(static_pointcloud_topics_param_);
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
    candidate_trajectories_marker_topic_ =
      get_parameter("ego_candidate_trajectories_marker_topic").as_string();
    local_map_marker_topic_ = get_parameter("ego_local_map_marker_topic").as_string();
    collision_points_marker_topic_ = get_parameter("ego_collision_points_marker_topic").as_string();
    footprint_marker_topic_ = get_parameter("ego_footprint_marker_topic").as_string();
    cmd_vel_marker_topic_ = get_parameter("ego_cmd_vel_marker_topic").as_string();
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
    ground_filter_use_path_z_ = get_parameter("ground_filter_use_path_z").as_bool();
    self_filter_radius_ = get_parameter("self_filter_radius").as_double();
    dynamic_obstacle_timeout_ = get_parameter("dynamic_obstacle_timeout").as_double();

    local_target_lookahead_ = get_parameter("local_target_lookahead").as_double();
    min_local_target_lookahead_ = get_parameter("min_local_target_lookahead").as_double();
    goal_reached_tolerance_ = get_parameter("goal_reached_tolerance").as_double();
    goal_reached_z_tolerance_ =
      std::max(0.0, get_parameter("goal_reached_z_tolerance").as_double());
    z_following_enabled_ = get_parameter("z_following_enabled").as_bool();
    max_allowed_z_jump_ = get_parameter("max_allowed_z_jump").as_double();
    z_aware_path_tracking_ = get_parameter("z_aware_path_tracking").as_bool();
    path_nearest_z_weight_ =
      std::max(0.0, get_parameter("path_nearest_z_weight").as_double());
    reference_z_weight_ =
      std::max(0.0, get_parameter("reference_z_weight").as_double());
    limit_local_target_by_z_ = get_parameter("limit_local_target_by_z").as_bool();
    local_target_max_z_delta_ =
      std::max(0.0, get_parameter("local_target_max_z_delta").as_double());

    max_vel_x_ = get_parameter("max_vel_x").as_double();
    max_vel_y_ = get_parameter("max_vel_y").as_double();
    max_acc_x_ = get_parameter("max_acc_x").as_double();
    max_acc_y_ = get_parameter("max_acc_y").as_double();
    max_yaw_rate_ = get_parameter("max_yaw_rate").as_double();
    max_yaw_acc_ = get_parameter("max_yaw_acc").as_double();

    traj_tracking_lookahead_time_ = get_parameter("traj_tracking_lookahead_time").as_double();
    traj_tracking_lookahead_distance_ =
      get_parameter("traj_tracking_lookahead_distance").as_double();
    line_of_sight_tracking_ = get_parameter("line_of_sight_tracking").as_bool();
    cmd_collision_check_time_ =
      std::max(0.0, get_parameter("cmd_collision_check_time").as_double());
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

    use_global_path_risk_ = get_parameter("use_global_path_risk").as_bool();
    allow_deviation_from_global_path_ =
      get_parameter("allow_deviation_from_global_path").as_bool();
    max_deviation_from_global_path_ =
      std::max(0.1, get_parameter("max_deviation_from_global_path").as_double());
    prefer_high_clearance_tracking_ =
      get_parameter("prefer_high_clearance_tracking").as_bool();
    min_local_clearance_ = std::max(0.0, get_parameter("min_local_clearance").as_double());
    preferred_local_clearance_ =
      std::max(min_local_clearance_, get_parameter("preferred_local_clearance").as_double());
    local_clearance_weight_ = get_parameter("local_clearance_weight").as_double();
    path_tracking_weight_ = get_parameter("path_tracking_weight").as_double();
    slow_down_near_obstacle_ = get_parameter("slow_down_near_obstacle").as_bool();
    obstacle_slow_distance_ =
      std::max(0.0, get_parameter("obstacle_slow_distance").as_double());
    obstacle_stop_distance_ =
      std::max(0.0, get_parameter("obstacle_stop_distance").as_double());
    debug_visualization_enabled_ = get_parameter("debug_visualization_enabled").as_bool();
    footprint_height_ = std::max(0.05, get_parameter("footprint_height").as_double());
    static_map_update_distance_ =
      std::max(0.05, get_parameter("static_map_update_distance").as_double());
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

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg, bool static_cloud)
  {
    geometry_msgs::msg::TransformStamped cloud_to_map;
    bool need_transform = false;
    if (!resolveCloudTransform(*msg, cloud_to_map, need_transform)) {
      return;
    }

    if (static_cloud) {
      static_map_points_.clear();
      static_map_points_.reserve(msg->width * msg->height);

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
        static_map_points_.push_back(point);
      }

      has_static_map_ = !static_map_points_.empty();
      needs_replan_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Loaded static obstacle cloud for EGO local planner: %zu points",
        static_map_points_.size());

      Pose3D base_pose;
      if (getCurrentPose(base_pose)) {
        rebuildLocalObstacleMap(base_pose, true);
        publishLocalMap();
      }
      return;
    }

    Pose3D base_pose;
    if (!getCurrentPose(base_pose)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cannot update EGO local map because current pose is unavailable.");
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
        double ground_reference = ground_relative_to_base_ ? base_pose.position.z() : 0.0;
        if (ground_filter_use_path_z_ && z_following_enabled_ && has_path_) {
          ground_reference = referenceZ(point.x(), point.y(), ground_reference);
        }
        if (point.z() - ground_reference < ground_z_threshold_) {
          continue;
        }
      }

      raw_points.push_back(point);
      insertInflatedPoint(point, occupied_voxels, occupied_xy);
    }

    dynamic_raw_obstacle_points_ = std::move(raw_points);
    dynamic_occupied_voxels_ = std::move(occupied_voxels);
    dynamic_occupied_xy_ = std::move(occupied_xy);
    has_cloud_ = true;
    last_cloud_time_ = now();
    needs_replan_ = needs_replan_ || replan_on_dynamic_obstacle_;

    rebuildLocalObstacleMap(base_pose, true);
    publishLocalMap();
  }

  bool resolveCloudTransform(
    const sensor_msgs::msg::PointCloud2 & msg,
    geometry_msgs::msg::TransformStamped & cloud_to_map,
    bool & need_transform)
  {
    const std::string raw_cloud_frame =
      msg.header.frame_id.empty() ? lidar_frame_ : msg.header.frame_id;
    std::string cloud_frame = remapFrameName(raw_cloud_frame, pointcloud_frame_remaps_);
    if (containsName(pointcloud_map_frame_aliases_, cloud_frame)) {
      cloud_frame = map_frame_;
    }

    need_transform = cloud_frame != map_frame_;
    if (!need_transform) {
      return true;
    }
    if (!lookupTransform(map_frame_, cloud_frame, cloud_to_map)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cannot transform point cloud from '%s' to '%s'.",
        cloud_frame.c_str(), map_frame_.c_str());
      return false;
    }
    return true;
  }

  bool isLocalObstaclePoint(const Eigen::Vector3d & point, const Pose3D & base_pose) const
  {
    const double dx = point.x() - base_pose.position.x();
    const double dy = point.y() - base_pose.position.y();
    const double dz = point.z() - base_pose.position.z();
    if (std::hypot(dx, dy) > local_map_radius_) {
      return false;
    }
    if (std::abs(dz) > local_map_height_) {
      return false;
    }
    if (std::hypot(dx, dy) < self_filter_radius_) {
      return false;
    }
    if (remove_ground_points_) {
      double ground_reference = ground_relative_to_base_ ? base_pose.position.z() : 0.0;
      if (ground_filter_use_path_z_ && z_following_enabled_ && has_path_) {
        ground_reference = referenceZ(point.x(), point.y(), ground_reference);
      }
      if (point.z() - ground_reference < ground_z_threshold_) {
        return false;
      }
    }
    return true;
  }

  void addLocalObstaclePoint(
    const Eigen::Vector3d & point,
    std::vector<Eigen::Vector3d> & raw_points,
    std::unordered_set<VoxelKey, VoxelKeyHash> & occupied_voxels,
    std::unordered_set<GridKey, GridKeyHash> & occupied_xy) const
  {
    raw_points.push_back(point);
    insertInflatedPoint(point, occupied_voxels, occupied_xy);
  }

  void rebuildLocalObstacleMap(const Pose3D & base_pose, bool force)
  {
    if (!force && !has_static_map_) {
      return;
    }
    if (!force && has_local_map_rebuild_pose_) {
      const double moved = (base_pose.position - last_local_map_rebuild_pose_).norm();
      if (moved < static_map_update_distance_) {
        return;
      }
    }

    std::vector<Eigen::Vector3d> raw_points;
    std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels;
    std::unordered_set<GridKey, GridKeyHash> occupied_xy;
    raw_points.reserve(dynamic_raw_obstacle_points_.size() + 4096);

    if (has_static_map_) {
      for (const auto & point : static_map_points_) {
        if (!isLocalObstaclePoint(point, base_pose)) {
          continue;
        }
        addLocalObstaclePoint(point, raw_points, occupied_voxels, occupied_xy);
      }
    }

    raw_points.insert(
      raw_points.end(),
      dynamic_raw_obstacle_points_.begin(),
      dynamic_raw_obstacle_points_.end());
    occupied_voxels.insert(dynamic_occupied_voxels_.begin(), dynamic_occupied_voxels_.end());
    occupied_xy.insert(dynamic_occupied_xy_.begin(), dynamic_occupied_xy_.end());

    clearRobotFootprint(base_pose.position, occupied_voxels, occupied_xy);
    raw_obstacle_points_ = std::move(raw_points);
    occupied_voxels_ = std::move(occupied_voxels);
    occupied_xy_ = std::move(occupied_xy);
    last_local_map_rebuild_pose_ = base_pose.position;
    has_local_map_rebuild_pose_ = true;
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
    rebuildLocalObstacleMap(current_pose, false);

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
    rebuildLocalObstacleMap(current_pose, false);

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

    const double min_obstacle_distance = minObstacleDistance(current_pose.position);
    if (slow_down_near_obstacle_ && std::isfinite(min_obstacle_distance) &&
      min_obstacle_distance <= obstacle_stop_distance_)
    {
      trajectory_collision_ = true;
      needs_replan_ = true;
      publishZeroCommand();
      setStatus("REPLAN");
      return;
    }

    cmd.linear.z = 0.0;
    cmd.angular.x = 0.0;
    cmd.angular.y = 0.0;
    cmd = applyCommandLimits(cmd);
    if (slow_down_near_obstacle_ && std::isfinite(min_obstacle_distance) &&
      min_obstacle_distance < obstacle_slow_distance_ &&
      obstacle_slow_distance_ > obstacle_stop_distance_)
    {
      const double scale = clamp(
        (min_obstacle_distance - obstacle_stop_distance_) /
        (obstacle_slow_distance_ - obstacle_stop_distance_),
        0.0, 1.0);
      cmd.linear.x *= scale;
      cmd.linear.y *= scale;
      cmd.angular.z *= std::max(0.35, scale);
    }
    if (!isCommandCollisionFree(current_pose, cmd, cmd_collision_check_time_)) {
      bool found_safe_scale = false;
      for (const double scale : {0.5, 0.25, 0.1}) {
        geometry_msgs::msg::Twist scaled = cmd;
        scaled.linear.x *= scale;
        scaled.linear.y *= scale;
        scaled.angular.z *= scale;
        if (isCommandCollisionFree(current_pose, scaled, cmd_collision_check_time_)) {
          cmd = scaled;
          found_safe_scale = true;
          break;
        }
      }
      if (!found_safe_scale) {
        needs_replan_ = true;
        publishZeroCommand();
        setStatus("REPLAN");
        return;
      }
    }
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
    if (has_pose) {
      rebuildLocalObstacleMap(pose, false);
    }

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
    ss << " global_path_received=" << (has_path_ ? "true" : "false");
    ss << " trajectory_valid=" << (!active_traj_.points.empty() && !trajectory_collision_ ? "true" : "false");
    ss << " collision_detected=" << (trajectory_collision_ ? "true" : "false");
    ss << " replan_triggered=" << (needs_replan_ ? "true" : "false");
    if (has_pose) {
      const double min_obstacle_distance = minObstacleDistance(pose.position);
      ss << " min_obstacle_distance="
        << (std::isfinite(min_obstacle_distance) ? min_obstacle_distance : -1.0);
      ss << " tracking_error=" << distanceToGlobalPath(pose.position.head<2>());
    }
    ss << " needs_replan=" << (needs_replan_ ? "true" : "false");
    ss << " trajectory_collision=" << (trajectory_collision_ ? "true" : "false");
    ss << " last_plan_failure=\"" << last_plan_failure_reason_ << "\"";
    ss << " cmd_vel_nav=(" << last_cmd_.linear.x << ", " << last_cmd_.linear.y
      << ", " << last_cmd_.angular.z << ")";
    ss << " state=" << status_;

    std_msgs::msg::String msg;
    msg.data = ss.str();
    debug_pub_->publish(msg);

    if (debug_visualization_enabled_ && has_pose) {
      if (!active_traj_.points.empty() && !trajectory_collision_) {
        publishTrajectory(active_traj_);
        publishTargetMarker(last_local_target_);
      }
      publishFootprintMarker(pose);
      publishCmdVelMarker(pose);
      publishCollisionPointsMarker();
      publishCandidateTrajectoriesMarker();
      publishLocalMapMarker();
    }
  }

  bool useZAwarePathTracking() const
  {
    return z_following_enabled_ && z_aware_path_tracking_;
  }

  double pathPointScore(
    const Eigen::Vector3d & path_point,
    const Eigen::Vector3d & reference) const
  {
    const double xy_distance = (path_point.head<2>() - reference.head<2>()).norm();
    if (!useZAwarePathTracking() || path_nearest_z_weight_ <= 0.0) {
      return xy_distance;
    }
    return std::hypot(xy_distance, path_nearest_z_weight_ * (path_point.z() - reference.z()));
  }

  std::size_t nearestPathIndex(
    const Eigen::Vector3d & current_pos,
    const std::vector<Eigen::Vector3d> & path) const
  {
    std::size_t nearest_index = 0;
    double nearest_score = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const double score = pathPointScore(path[i], current_pos);
      if (score < nearest_score) {
        nearest_score = score;
        nearest_index = i;
      }
    }
    return nearest_index;
  }

  Eigen::Vector3d selectLocalTargetFromGlobalPath(
    const Eigen::Vector3d & current_pos,
    const std::vector<Eigen::Vector3d> & global_path) const
  {
    if (global_path.empty()) {
      return current_pos;
    }

    const std::size_t nearest_index = nearestPathIndex(current_pos, global_path);

    const Eigen::Vector3d final_goal = global_path.back();
    const double final_xy_distance = (final_goal.head<2>() - current_pos.head<2>()).norm();
    const double final_z_distance = std::abs(final_goal.z() - current_pos.z());
    const double z_window = local_target_max_z_delta_ > 1.0e-6 ?
      local_target_max_z_delta_ : max_allowed_z_jump_;
    const bool final_z_reachable = !useZAwarePathTracking() ||
      final_z_distance <= std::max(z_window, goal_reached_z_tolerance_);
    if (final_xy_distance <= std::max(goal_reached_tolerance_, local_target_lookahead_) &&
      final_z_reachable)
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
      const bool reached_xy_lookahead = accumulated >= lookahead;
      const bool reached_z_window = limit_local_target_by_z_ && useZAwarePathTracking() &&
        z_window > 1.0e-6 && std::abs(selected.z() - current_pos.z()) >= z_window &&
        accumulated >= min_local_target_lookahead_;
      if (reached_xy_lookahead || reached_z_window) {
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
      const double distance = pathPointScore(traj.points[i], current_pose.position);
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

    if (line_of_sight_tracking_) {
      while (target_index > nearest_index + 1 &&
        !isSegmentCollisionFree(current_pose.position, traj.points[target_index]))
      {
        --target_index;
      }
      if (!isSegmentCollisionFree(current_pose.position, traj.points[target_index])) {
        return false;
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
    last_rejected_trajectory_points_.clear();
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
      last_rejected_trajectory_points_ = path;
      return false;
    }

    Trajectory candidate;
    candidate.points = std::move(path);
    last_candidate_trajectory_points_ = candidate.points;
    candidate.stamp = now();
    candidate.collision_free = isTrajectoryCollisionFree(candidate);
    if (!candidate.collision_free) {
      last_plan_failure_reason_ = "candidate trajectory became occupied during final collision check";
      last_rejected_trajectory_points_ = candidate.points;
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
        reconstructGridPath(records, start_key, goal_key, start.z(), target.z(), path);
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

        const Eigen::Vector2d next_xy = gridKeyToWorld(next);
        const double deviation = distanceToGlobalPath(next_xy);
        if (allow_deviation_from_global_path_ && deviation > max_deviation_from_global_path_ &&
          next != goal_key)
        {
          continue;
        }

        const double step_cost = std::hypot(static_cast<double>(step.x), static_cast<double>(step.y)) *
          voxel_resolution_;
        const double tracking_cost = use_global_path_risk_ ?
          path_tracking_weight_ * std::min(deviation, max_deviation_from_global_path_) : 0.0;
        const double clearance_cost = prefer_high_clearance_tracking_ ?
          local_clearance_weight_ * localClearancePenalty(next) : 0.0;
        const double new_g = current_record.g + step_cost + tracking_cost + clearance_cost;
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
    double start_z,
    double target_z,
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
    std::vector<double> accumulated_distance(keys.size(), 0.0);
    for (std::size_t i = 1; i < keys.size(); ++i) {
      accumulated_distance[i] = accumulated_distance[i - 1] +
        (gridKeyToWorld(keys[i]) - gridKeyToWorld(keys[i - 1])).norm();
    }
    const double total_distance = accumulated_distance.empty() ? 0.0 : accumulated_distance.back();

    path.clear();
    path.reserve(keys.size());
    for (std::size_t i = 0; i < keys.size(); ++i) {
      const auto & key = keys[i];
      const Eigen::Vector2d xy = gridKeyToWorld(key);
      const double ratio = total_distance > 1.0e-8 ?
        accumulated_distance[i] / total_distance :
        (keys.size() > 1 ? static_cast<double>(i) / static_cast<double>(keys.size() - 1) : 0.0);
      const double fallback_z = (1.0 - ratio) * start_z + ratio * target_z;
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

  bool isTrajectoryCollisionFree(const Trajectory & traj)
  {
    if (traj.points.empty()) {
      return false;
    }
    last_collision_points_.clear();
    for (const auto & point : traj.points) {
      if (!isPoseCollisionFree(point)) {
        last_collision_points_.push_back(point);
        return false;
      }
    }
    for (std::size_t i = 0; i + 1 < traj.points.size(); ++i) {
      if (!isSegmentCollisionFree(traj.points[i], traj.points[i + 1])) {
        last_collision_points_.push_back(traj.points[i]);
        last_collision_points_.push_back(traj.points[i + 1]);
        return false;
      }
    }
    return true;
  }

  bool isCommandCollisionFree(
    const Pose3D & current_pose,
    const geometry_msgs::msg::Twist & cmd,
    double horizon) const
  {
    if (horizon <= 1.0e-6) {
      return true;
    }
    const double cos_yaw = std::cos(current_pose.yaw);
    const double sin_yaw = std::sin(current_pose.yaw);
    const Eigen::Vector2d velocity_map(
      cos_yaw * cmd.linear.x - sin_yaw * cmd.linear.y,
      sin_yaw * cmd.linear.x + cos_yaw * cmd.linear.y);
    const Eigen::Vector3d end =
      current_pose.position + Eigen::Vector3d(
      velocity_map.x() * horizon,
      velocity_map.y() * horizon,
      0.0);
    return isSegmentCollisionFree(current_pose.position, end);
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

    const std::size_t nearest_index = nearestPathIndex(current_pos, global_path_);

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
    const Eigen::Vector3d goal = global_path_.back();
    if ((goal.head<2>() - current_pos.head<2>()).norm() > goal_reached_tolerance_) {
      return false;
    }
    if (useZAwarePathTracking() &&
      std::abs(goal.z() - current_pos.z()) > goal_reached_z_tolerance_)
    {
      return false;
    }
    return true;
  }

  bool isGridOccupied(const GridKey & key) const
  {
    return occupied_xy_.find(key) != occupied_xy_.end();
  }

  double distanceToGlobalPath(const Eigen::Vector2d & query) const
  {
    if (global_path_.empty()) {
      return 0.0;
    }
    if (global_path_.size() == 1) {
      return (global_path_.front().head<2>() - query).norm();
    }

    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i + 1 < global_path_.size(); ++i) {
      const Eigen::Vector2d a = global_path_[i].head<2>();
      const Eigen::Vector2d b = global_path_[i + 1].head<2>();
      const Eigen::Vector2d ab = b - a;
      const double length_sq = ab.squaredNorm();
      const double t = length_sq > 1.0e-8 ?
        clamp((query - a).dot(ab) / length_sq, 0.0, 1.0) : 0.0;
      best_distance = std::min(best_distance, (query - (a + t * ab)).norm());
    }
    return best_distance;
  }

  double distanceToNearestOccupiedGrid(const GridKey & key, double max_distance) const
  {
    if (occupied_xy_.empty()) {
      return std::numeric_limits<double>::infinity();
    }
    const int max_cells = std::max(1, static_cast<int>(std::ceil(max_distance / voxel_resolution_)));
    for (int radius = 0; radius <= max_cells; ++radius) {
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          if (std::max(std::abs(dx), std::abs(dy)) != radius) {
            continue;
          }
          if (isGridOccupied({key.x + dx, key.y + dy})) {
            return std::hypot(static_cast<double>(dx), static_cast<double>(dy)) *
                   voxel_resolution_;
          }
        }
      }
    }
    return std::numeric_limits<double>::infinity();
  }

  double localClearancePenalty(const GridKey & key) const
  {
    const double clearance = distanceToNearestOccupiedGrid(key, preferred_local_clearance_);
    if (!std::isfinite(clearance) || clearance >= preferred_local_clearance_) {
      return 0.0;
    }
    if (clearance <= min_local_clearance_) {
      return 1000.0 + (min_local_clearance_ - clearance) * 100.0;
    }
    const double span = std::max(1.0e-3, preferred_local_clearance_ - min_local_clearance_);
    const double ratio = (preferred_local_clearance_ - clearance) / span;
    return ratio * ratio;
  }

  double minObstacleDistance(const Eigen::Vector3d & query) const
  {
    double best = std::numeric_limits<double>::infinity();
    for (const auto & point : raw_obstacle_points_) {
      const double dz = std::abs(point.z() - query.z());
      if (dz > local_map_height_) {
        continue;
      }
      best = std::min(best, (point.head<2>() - query.head<2>()).norm());
    }
    return best;
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

    double best_score = std::numeric_limits<double>::infinity();
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
      const double z = (1.0 - t) * global_path_[i].z() + t * global_path_[i + 1].z();
      const double score = useZAwarePathTracking() && reference_z_weight_ > 0.0 ?
        std::hypot(distance, reference_z_weight_ * (z - fallback_z)) : distance;
      if (score < best_score) {
        best_score = score;
        best_z = z;
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
      dynamic_raw_obstacle_points_.clear();
      dynamic_occupied_voxels_.clear();
      dynamic_occupied_xy_.clear();
      Pose3D pose;
      if (getCurrentPose(pose)) {
        rebuildLocalObstacleMap(pose, true);
      } else {
        raw_obstacle_points_.clear();
        occupied_voxels_.clear();
        occupied_xy_.clear();
        has_local_map_rebuild_pose_ = false;
      }
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
    marker.lifetime = durationFromSeconds(1.2);
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
    marker.lifetime = durationFromSeconds(1.2);
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

  void publishLocalMapMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_frame_;
    marker.ns = "ego_local_map";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = std::max(0.04, voxel_resolution_ * 0.45);
    marker.scale.y = marker.scale.x;
    marker.color.r = 0.95F;
    marker.color.g = 0.15F;
    marker.color.b = 0.05F;
    marker.color.a = 0.55F;
    marker.lifetime = durationFromSeconds(0.6);

    std::size_t count = 0;
    const std::size_t max_points = 5000;
    const std::size_t stride = std::max<std::size_t>(1, occupied_voxels_.size() / max_points);
    for (const auto & key : occupied_voxels_) {
      if ((count++ % stride) != 0) {
        continue;
      }
      marker.points.push_back(toPointMsg(voxelKeyToWorld(key)));
    }
    local_map_marker_pub_->publish(marker);
  }

  void publishCandidateTrajectoriesMarker()
  {
    visualization_msgs::msg::MarkerArray array;
    const auto stamp = now();
    addTrajectoryMarker(array, active_traj_.points, stamp, "ego_candidate_trajectories", 0, 0.1F, 0.8F, 1.0F, 0.95F);
    addTrajectoryMarker(array, last_candidate_trajectory_points_, stamp, "ego_candidate_trajectories", 1, 0.1F, 1.0F, 0.25F, 0.80F);
    addTrajectoryMarker(array, last_rejected_trajectory_points_, stamp, "ego_candidate_trajectories", 2, 1.0F, 0.05F, 0.05F, 0.90F);
    candidate_trajectories_pub_->publish(array);
  }

  void addTrajectoryMarker(
    visualization_msgs::msg::MarkerArray & array,
    const std::vector<Eigen::Vector3d> & points,
    const rclcpp::Time & stamp,
    const std::string & ns,
    int id,
    float r,
    float g,
    float b,
    float a) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = points.empty() ? visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = id == 0 ? 0.07 : 0.045;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.lifetime = durationFromSeconds(0.6);
    for (const auto & point : points) {
      marker.points.push_back(toPointMsg(point));
    }
    array.markers.push_back(marker);
  }

  void publishCollisionPointsMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_frame_;
    marker.ns = "ego_collision_points";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = last_collision_points_.empty() ? visualization_msgs::msg::Marker::DELETE :
      visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.18;
    marker.scale.y = 0.18;
    marker.color.r = 1.0F;
    marker.color.g = 0.0F;
    marker.color.b = 0.0F;
    marker.color.a = 1.0F;
    marker.lifetime = durationFromSeconds(0.8);
    for (const auto & point : last_collision_points_) {
      marker.points.push_back(toPointMsg(point));
    }
    collision_points_pub_->publish(marker);
  }

  void publishFootprintMarker(const Pose3D & pose)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_frame_;
    marker.ns = "ego_footprint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = toPointMsg(pose.position);
    marker.pose.position.z += footprint_height_ * 0.5;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pose.yaw);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    const double radius = std::max(robot_clear_radius_, min_local_clearance_);
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = footprint_height_;
    marker.color.r = 0.1F;
    marker.color.g = 0.7F;
    marker.color.b = 1.0F;
    marker.color.a = 0.22F;
    marker.lifetime = durationFromSeconds(0.6);
    footprint_pub_->publish(marker);
  }

  void publishCmdVelMarker(const Pose3D & pose)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = map_frame_;
    marker.ns = "ego_cmd_vel";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.scale.y = 0.16;
    marker.scale.z = 0.18;
    marker.color.r = 0.95F;
    marker.color.g = 0.55F;
    marker.color.b = 0.05F;
    marker.color.a = 0.95F;
    marker.lifetime = durationFromSeconds(0.6);

    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    const Eigen::Vector2d velocity_map(
      cos_yaw * last_cmd_.linear.x - sin_yaw * last_cmd_.linear.y,
      sin_yaw * last_cmd_.linear.x + cos_yaw * last_cmd_.linear.y);
    const double speed = velocity_map.norm();
    const Eigen::Vector3d start = pose.position + Eigen::Vector3d(0.0, 0.0, 0.20);
    const Eigen::Vector3d end = start + Eigen::Vector3d(
      velocity_map.x(),
      velocity_map.y(),
      0.0) * (speed > 1.0e-4 ? std::max(0.5, 1.5 / std::max(0.1, speed)) : 0.0);
    marker.points.push_back(toPointMsg(start));
    marker.points.push_back(toPointMsg(end));
    cmd_vel_marker_pub_->publish(marker);
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
  std::string candidate_trajectories_marker_topic_;
  std::string local_map_marker_topic_;
  std::string collision_points_marker_topic_;
  std::string footprint_marker_topic_;
  std::string cmd_vel_marker_topic_;
  std::string status_topic_;
  std::string debug_topic_;
  std::string robot_model_;
  std::string additional_pointcloud_topics_;
  std::string static_pointcloud_topics_param_;
  std::string pointcloud_frame_remaps_;
  std::string pointcloud_map_frame_aliases_param_;
  std::vector<std::string> pointcloud_topics_;
  std::vector<std::string> static_pointcloud_topics_;
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
  bool ground_filter_use_path_z_{true};
  double self_filter_radius_{0.45};
  double dynamic_obstacle_timeout_{0.5};
  double local_target_lookahead_{2.0};
  double min_local_target_lookahead_{0.8};
  double goal_reached_tolerance_{0.35};
  double goal_reached_z_tolerance_{0.45};
  bool z_following_enabled_{true};
  double max_allowed_z_jump_{0.35};
  bool z_aware_path_tracking_{true};
  double path_nearest_z_weight_{1.0};
  double reference_z_weight_{1.0};
  bool limit_local_target_by_z_{true};
  double local_target_max_z_delta_{0.35};
  double max_vel_x_{0.6};
  double max_vel_y_{0.4};
  double max_acc_x_{0.5};
  double max_acc_y_{0.5};
  double max_yaw_rate_{1.0};
  double max_yaw_acc_{1.5};
  double traj_tracking_lookahead_time_{0.5};
  double traj_tracking_lookahead_distance_{0.6};
  bool line_of_sight_tracking_{true};
  double cmd_collision_check_time_{0.6};
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
  bool use_global_path_risk_{true};
  bool allow_deviation_from_global_path_{true};
  double max_deviation_from_global_path_{0.8};
  bool prefer_high_clearance_tracking_{true};
  double min_local_clearance_{0.35};
  double preferred_local_clearance_{0.65};
  double local_clearance_weight_{2.0};
  double path_tracking_weight_{1.0};
  bool slow_down_near_obstacle_{true};
  double obstacle_slow_distance_{0.8};
  double obstacle_stop_distance_{0.35};
  bool debug_visualization_enabled_{true};
  double footprint_height_{0.55};
  double static_map_update_distance_{0.25};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr global_path_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> pointcloud_subs_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> static_pointcloud_subs_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectory_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_vis_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_trajectories_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_map_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr collision_points_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr footprint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cmd_vel_marker_pub_;

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

  std::vector<Eigen::Vector3d> static_map_points_;
  bool has_static_map_{false};
  bool has_local_map_rebuild_pose_{false};
  Eigen::Vector3d last_local_map_rebuild_pose_{Eigen::Vector3d::Zero()};
  std::vector<Eigen::Vector3d> dynamic_raw_obstacle_points_;
  std::unordered_set<VoxelKey, VoxelKeyHash> dynamic_occupied_voxels_;
  std::unordered_set<GridKey, GridKeyHash> dynamic_occupied_xy_;
  std::vector<Eigen::Vector3d> raw_obstacle_points_;
  std::vector<Eigen::Vector3d> last_collision_points_;
  std::vector<Eigen::Vector3d> last_candidate_trajectory_points_;
  std::vector<Eigen::Vector3d> last_rejected_trajectory_points_;
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
