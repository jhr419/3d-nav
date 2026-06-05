#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <builtin_interfaces/msg/duration.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "local_planning/dwa_3d_local_planner.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2/time.h"
#include "tf2/utils.h"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace local_planning
{
namespace
{

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

std::vector<std::string> splitFrameList(const std::string & frames)
{
  std::vector<std::string> out;
  std::stringstream ss(frames);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item = trim(item);
    if (!item.empty() && std::find(out.begin(), out.end(), item) == out.end()) {
      out.push_back(item);
    }
  }
  return out;
}

std::string toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

bool endsWith(const std::string & value, const std::string & suffix)
{
  return value.size() >= suffix.size() &&
         value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

geometry_msgs::msg::Point makePoint(double x, double y, double z)
{
  geometry_msgs::msg::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

builtin_interfaces::msg::Duration makeDurationMsg(double seconds)
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

double groundDistance(const Pose3D & pose, const Eigen::Vector3d & point, bool use_3d)
{
  if (use_3d) {
    return (Eigen::Vector3d(pose.x, pose.y, pose.z) - point).norm();
  }
  return std::hypot(pose.x - point.x(), pose.y - point.y());
}

}  // namespace

class LocalPlannerNode : public rclcpp::Node
{
public:
  LocalPlannerNode()
  : Node("local_planner_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();
    planner_.setConfig(readPlannerConfig());

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    const auto global_path_topic = get_parameter("global_path_topic").as_string();
    const auto additional_global_path_topic =
      get_parameter("additional_global_path_topic").as_string();
    const auto local_obstacle_cloud_topic =
      get_parameter("local_obstacle_cloud_topic").as_string();
    const auto octomap_topic = get_parameter("octomap_topic").as_string();
    const auto odom_topic = get_parameter("odom_topic").as_string();
    const auto sport_mode_state_topic =
      get_parameter("sport_mode_state_topic").as_string();
    const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();

    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      global_path_topic, rclcpp::QoS(1).transient_local().reliable(),
      [this, global_path_topic](const nav_msgs::msg::Path::SharedPtr msg) {
        onPath(msg, global_path_topic);
      });
    if (!additional_global_path_topic.empty() && additional_global_path_topic != global_path_topic) {
      additional_path_sub_ = create_subscription<nav_msgs::msg::Path>(
        additional_global_path_topic, rclcpp::QoS(1).transient_local().reliable(),
        [this, additional_global_path_topic](const nav_msgs::msg::Path::SharedPtr msg) {
          onPath(msg, additional_global_path_topic);
        });
    }

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(30).reliable(),
      std::bind(&LocalPlannerNode::onOdom, this, std::placeholders::_1));
    sport_mode_state_sub_ = create_subscription<unitree_go::msg::SportModeState>(
      sport_mode_state_topic, rclcpp::QoS(30).reliable(),
      std::bind(&LocalPlannerNode::onSportModeState, this, std::placeholders::_1));
    obstacle_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      local_obstacle_cloud_topic, rclcpp::SensorDataQoS(),
      std::bind(&LocalPlannerNode::onObstacleCloud, this, std::placeholders::_1));
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&LocalPlannerNode::onOctomap, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    best_traj_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("local_trajectory_marker_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable());
    candidate_traj_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      get_parameter("candidate_trajectories_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable());
    local_goal_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("local_goal_marker_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable());
    recovery_traj_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("recovery_trajectory_marker_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable());
    path_corridor_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("path_corridor_marker_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable());
    debug_text_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("dwa_debug_text_topic").as_string(),
      rclcpp::QoS(10).reliable());

    loadOctomapFile(get_parameter("octomap_file").as_string());

    const double frequency = std::max(1.0, get_parameter("control_frequency").as_double());
    const auto period = std::chrono::duration<double>(1.0 / frequency);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LocalPlannerNode::onControlTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "local_planner_node started. path=%s alias=%s odom=%s sport_mode=%s obstacle_cloud=%s "
      "octomap=%s cmd_vel=%s map_frame=%s base_frame=%s obstacle_source=%s velocity_source=%s",
      global_path_topic.c_str(),
      additional_global_path_topic.empty() ? "(none)" : additional_global_path_topic.c_str(),
      odom_topic.c_str(), sport_mode_state_topic.c_str(), local_obstacle_cloud_topic.c_str(),
      octomap_topic.c_str(), cmd_vel_topic.c_str(),
      get_parameter("map_frame").as_string().c_str(), get_parameter("base_frame").as_string().c_str(),
      planner_.getConfig().obstacle_source.c_str(),
      get_parameter("velocity_source").as_string().c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>(
      "base_frame_candidates", "base_link,odin1_base_link,base_footprint");
    declare_parameter<std::string>("global_path_topic", "/planned_path");
    declare_parameter<std::string>("additional_global_path_topic", "/global_path_3d");
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<std::string>("velocity_source", "auto");
    declare_parameter<std::string>("sport_mode_state_topic", "/sportmodestate");
    declare_parameter<std::string>("local_obstacle_cloud_topic", "/local_obstacle_cloud");
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("local_trajectory_marker_topic", "/local_trajectory_marker");
    declare_parameter<std::string>("candidate_trajectories_topic", "/dwa_candidate_trajectories");
    declare_parameter<std::string>("local_goal_marker_topic", "/local_goal_marker");
    declare_parameter<std::string>("recovery_trajectory_marker_topic", "/recovery_trajectory_marker");
    declare_parameter<std::string>("path_corridor_marker_topic", "/path_corridor_marker");
    declare_parameter<std::string>("dwa_debug_text_topic", "/dwa_debug_text");
    declare_parameter<std::string>("robot_model", "ground_omni");
    declare_parameter<std::string>("obstacle_source", "octomap");
    declare_parameter<std::string>("octomap_file", "/home/jhr/3dnav_ws/maps/result_cleaned.bt");
    declare_parameter<bool>("unknown_as_occupied", true);
    declare_parameter<double>("robot_radius", 0.35);
    declare_parameter<double>("robot_height", 0.6);
    declare_parameter<double>("collision_z_offset", 0.10);
    declare_parameter<double>("safety_margin", 0.15);
    declare_parameter<double>("control_frequency", 10.0);
    declare_parameter<double>("sim_time", 2.0);
    declare_parameter<double>("sim_dt", 0.1);
    declare_parameter<double>("max_vx", 0.6);
    declare_parameter<double>("min_vx", -0.2);
    declare_parameter<double>("max_vy", 0.4);
    declare_parameter<double>("min_vy", -0.4);
    declare_parameter<double>("max_wz", 1.0);
    declare_parameter<double>("min_wz", -1.0);
    declare_parameter<double>("max_acc_vx", 0.5);
    declare_parameter<double>("max_acc_vy", 0.5);
    declare_parameter<double>("max_acc_wz", 1.5);
    declare_parameter<int>("vx_samples", 9);
    declare_parameter<int>("vy_samples", 9);
    declare_parameter<int>("wz_samples", 15);
    declare_parameter<double>("local_goal_lookahead", 1.5);
    declare_parameter<double>("goal_reached_tolerance", 0.3);
    declare_parameter<double>("path_prune_distance", 0.5);
    declare_parameter<double>("weight_path_distance", 1.0);
    declare_parameter<double>("weight_goal_distance", 1.2);
    declare_parameter<double>("weight_obstacle_distance", 1.5);
    declare_parameter<double>("weight_heading", 0.5);
    declare_parameter<double>("weight_velocity", 0.2);
    declare_parameter<double>("weight_smoothness", 0.2);
    declare_parameter<double>("obstacle_check_resolution", 0.1);
    declare_parameter<double>("min_obstacle_distance", 0.25);
    declare_parameter<double>("obstacle_score_distance", 2.0);
    declare_parameter<bool>("stop_on_no_valid_trajectory", true);
    declare_parameter<bool>("use_path_z_for_collision", true);
    declare_parameter<bool>("terrain_following_enabled", true);
    declare_parameter<double>("z_search_radius", 0.6);
    declare_parameter<double>("max_allowed_z_jump", 0.35);
    declare_parameter<double>("slope_edge_z_tolerance", 0.45);
    declare_parameter<std::string>("collision_model", "terrain_adaptive_cylinder");
    declare_parameter<double>("ground_clearance", 0.05);
    declare_parameter<double>("body_z_offset", 0.10);
    declare_parameter<bool>("ignore_ground_below_base", true);
    declare_parameter<double>("ground_ignore_depth", 0.15);
    declare_parameter<bool>("slope_edge_relaxation_enabled", true);
    declare_parameter<double>("slope_edge_relaxation_radius", 0.25);
    declare_parameter<std::string>("unknown_policy", "path_corridor_free");
    declare_parameter<double>("path_corridor_radius", 0.5);
    declare_parameter<bool>("adaptive_lookahead_enabled", true);
    declare_parameter<double>("min_local_goal_lookahead", 0.4);
    declare_parameter<double>("max_local_goal_lookahead", 1.8);
    declare_parameter<double>("z_change_slowdown_threshold", 0.25);
    declare_parameter<bool>("recovery_enabled", true);
    declare_parameter<bool>("stuck_detection_enabled", true);
    declare_parameter<double>("stuck_time_threshold", 2.0);
    declare_parameter<double>("min_progress_distance", 0.05);
    declare_parameter<double>("recovery_duration", 1.0);
    declare_parameter<bool>("enable_reverse_escape", true);
    declare_parameter<bool>("enable_lateral_escape", true);
    declare_parameter<bool>("enable_rotate_escape", true);
    declare_parameter<bool>("near_path_bonus_enabled", true);
    declare_parameter<double>("near_path_bonus_radius", 0.4);
    declare_parameter<double>("weight_path_corridor", 1.0);
    declare_parameter<double>("weight_z_consistency", 0.8);
    declare_parameter<double>("weight_progress", 1.0);
    declare_parameter<bool>("debug_dwa", true);
    declare_parameter<bool>("debug_collision", true);
    declare_parameter<bool>("publish_candidate_trajectories", true);
    declare_parameter<int>("candidate_marker_stride", 1);
    declare_parameter<double>("marker_lifetime", 0.4);
    declare_parameter<double>("tf_timeout", 0.05);
    declare_parameter<double>("odom_timeout", 0.5);
    declare_parameter<double>("sport_mode_timeout", 0.5);
  }

  DWA3DLocalPlanner::Config readPlannerConfig() const
  {
    DWA3DLocalPlanner::Config config;
    config.robot_model = get_parameter("robot_model").as_string();
    config.obstacle_source = get_parameter("obstacle_source").as_string();
    config.unknown_as_occupied = get_parameter("unknown_as_occupied").as_bool();
    config.robot_radius = get_parameter("robot_radius").as_double();
    config.robot_height = get_parameter("robot_height").as_double();
    config.collision_z_offset = get_parameter("collision_z_offset").as_double();
    config.safety_margin = get_parameter("safety_margin").as_double();
    config.control_frequency = get_parameter("control_frequency").as_double();
    config.sim_time = get_parameter("sim_time").as_double();
    config.sim_dt = get_parameter("sim_dt").as_double();
    config.max_vx = get_parameter("max_vx").as_double();
    config.min_vx = get_parameter("min_vx").as_double();
    config.max_vy = get_parameter("max_vy").as_double();
    config.min_vy = get_parameter("min_vy").as_double();
    config.max_wz = get_parameter("max_wz").as_double();
    config.min_wz = get_parameter("min_wz").as_double();
    config.max_acc_vx = get_parameter("max_acc_vx").as_double();
    config.max_acc_vy = get_parameter("max_acc_vy").as_double();
    config.max_acc_wz = get_parameter("max_acc_wz").as_double();
    config.vx_samples = static_cast<int>(get_parameter("vx_samples").as_int());
    config.vy_samples = static_cast<int>(get_parameter("vy_samples").as_int());
    config.wz_samples = static_cast<int>(get_parameter("wz_samples").as_int());
    config.local_goal_lookahead = get_parameter("local_goal_lookahead").as_double();
    config.goal_reached_tolerance = get_parameter("goal_reached_tolerance").as_double();
    config.path_prune_distance = get_parameter("path_prune_distance").as_double();
    config.weight_path_distance = get_parameter("weight_path_distance").as_double();
    config.weight_goal_distance = get_parameter("weight_goal_distance").as_double();
    config.weight_obstacle_distance = get_parameter("weight_obstacle_distance").as_double();
    config.weight_heading = get_parameter("weight_heading").as_double();
    config.weight_velocity = get_parameter("weight_velocity").as_double();
    config.weight_smoothness = get_parameter("weight_smoothness").as_double();
    config.obstacle_check_resolution = get_parameter("obstacle_check_resolution").as_double();
    config.min_obstacle_distance = get_parameter("min_obstacle_distance").as_double();
    config.obstacle_score_distance = get_parameter("obstacle_score_distance").as_double();
    config.stop_on_no_valid_trajectory = get_parameter("stop_on_no_valid_trajectory").as_bool();
    config.use_path_z_for_collision = get_parameter("use_path_z_for_collision").as_bool();
    config.terrain_following_enabled = get_parameter("terrain_following_enabled").as_bool();
    config.z_search_radius = get_parameter("z_search_radius").as_double();
    config.max_allowed_z_jump = get_parameter("max_allowed_z_jump").as_double();
    config.slope_edge_z_tolerance = get_parameter("slope_edge_z_tolerance").as_double();
    config.collision_model = get_parameter("collision_model").as_string();
    config.ground_clearance = get_parameter("ground_clearance").as_double();
    config.body_z_offset = get_parameter("body_z_offset").as_double();
    config.ignore_ground_below_base = get_parameter("ignore_ground_below_base").as_bool();
    config.ground_ignore_depth = get_parameter("ground_ignore_depth").as_double();
    config.slope_edge_relaxation_enabled =
      get_parameter("slope_edge_relaxation_enabled").as_bool();
    config.slope_edge_relaxation_radius =
      get_parameter("slope_edge_relaxation_radius").as_double();
    config.unknown_policy = get_parameter("unknown_policy").as_string();
    config.path_corridor_radius = get_parameter("path_corridor_radius").as_double();
    config.adaptive_lookahead_enabled = get_parameter("adaptive_lookahead_enabled").as_bool();
    config.min_local_goal_lookahead = get_parameter("min_local_goal_lookahead").as_double();
    config.max_local_goal_lookahead = get_parameter("max_local_goal_lookahead").as_double();
    config.z_change_slowdown_threshold =
      get_parameter("z_change_slowdown_threshold").as_double();
    config.recovery_enabled = get_parameter("recovery_enabled").as_bool();
    config.stuck_detection_enabled = get_parameter("stuck_detection_enabled").as_bool();
    config.stuck_time_threshold = get_parameter("stuck_time_threshold").as_double();
    config.min_progress_distance = get_parameter("min_progress_distance").as_double();
    config.recovery_duration = get_parameter("recovery_duration").as_double();
    config.enable_reverse_escape = get_parameter("enable_reverse_escape").as_bool();
    config.enable_lateral_escape = get_parameter("enable_lateral_escape").as_bool();
    config.enable_rotate_escape = get_parameter("enable_rotate_escape").as_bool();
    config.near_path_bonus_enabled = get_parameter("near_path_bonus_enabled").as_bool();
    config.near_path_bonus_radius = get_parameter("near_path_bonus_radius").as_double();
    config.weight_path_corridor = get_parameter("weight_path_corridor").as_double();
    config.weight_z_consistency = get_parameter("weight_z_consistency").as_double();
    config.weight_progress = get_parameter("weight_progress").as_double();
    config.debug_dwa = get_parameter("debug_dwa").as_bool();
    config.debug_collision = get_parameter("debug_collision").as_bool();
    return config;
  }

  void loadOctomapFile(const std::string & octomap_file)
  {
    if (octomap_file.empty()) {
      return;
    }

    std::ifstream input(octomap_file);
    if (!input.good()) {
      RCLCPP_WARN(get_logger(), "OctoMap file does not exist or is not readable: %s", octomap_file.c_str());
      return;
    }
    input.close();

    if (endsWith(octomap_file, ".bt")) {
      auto binary_tree = std::make_shared<octomap::OcTree>(0.1);
      if (binary_tree->readBinary(octomap_file)) {
        planner_.setOctomap(binary_tree);
        RCLCPP_INFO(
          get_logger(), "Loaded binary OctoMap file: %s resolution=%.3f nodes=%zu",
          octomap_file.c_str(), binary_tree->getResolution(), binary_tree->size());
        return;
      }
    } else {
      std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
        octomap::AbstractOcTree::read(octomap_file));
      if (abstract_tree) {
        auto * oc_tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
        if (oc_tree) {
          abstract_tree.release();
          planner_.setOctomap(std::shared_ptr<const octomap::OcTree>(oc_tree));
          RCLCPP_INFO(
            get_logger(), "Loaded OctoMap file: %s resolution=%.3f nodes=%zu",
            octomap_file.c_str(), oc_tree->getResolution(), oc_tree->size());
          return;
        }
      }
    }

    auto fallback_binary_tree = std::make_shared<octomap::OcTree>(0.1);
    if (fallback_binary_tree->readBinary(octomap_file)) {
      planner_.setOctomap(fallback_binary_tree);
      RCLCPP_INFO(
        get_logger(), "Loaded binary OctoMap file: %s resolution=%.3f nodes=%zu",
        octomap_file.c_str(), fallback_binary_tree->getResolution(), fallback_binary_tree->size());
      return;
    }

    RCLCPP_ERROR(get_logger(), "Failed to load OctoMap file: %s", octomap_file.c_str());
  }

  void onPath(const nav_msgs::msg::Path::SharedPtr msg, const std::string & topic_name)
  {
    if (msg->poses.empty()) {
      global_path_.clear();
      path_ready_ = false;
      goal_reached_logged_ = false;
      resetStuckState();
      publishZeroCmd();
      clearMarkers();
      RCLCPP_WARN(get_logger(), "Received empty path from %s. Local planner is idle.", topic_name.c_str());
      return;
    }

    const std::string map_frame = get_parameter("map_frame").as_string();
    std::string frame_id = msg->header.frame_id;
    if (frame_id.empty()) {
      frame_id = msg->poses.front().header.frame_id;
    }
    if (frame_id.empty()) {
      frame_id = map_frame;
    }

    if (frame_id != map_frame) {
      RCLCPP_WARN(
        get_logger(), "Ignored path from %s because frame is %s, expected %s.",
        topic_name.c_str(), frame_id.c_str(), map_frame.c_str());
      return;
    }

    std::vector<Eigen::Vector3d> path;
    path.reserve(msg->poses.size());
    for (const auto & pose : msg->poses) {
      if (!pose.header.frame_id.empty() && pose.header.frame_id != map_frame) {
        RCLCPP_WARN(
          get_logger(), "Ignored path from %s because a pose frame is %s, expected %s.",
          topic_name.c_str(), pose.header.frame_id.c_str(), map_frame.c_str());
        return;
      }
      path.emplace_back(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }

    global_path_ = std::move(path);
    path_ready_ = true;
    goal_reached_logged_ = false;
    resetStuckState();
    RCLCPP_INFO(
      get_logger(), "Received local-planner global path from %s with %zu poses.",
      topic_name.c_str(), global_path_.size());
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_vel_.vx = msg->twist.twist.linear.x;
    last_odom_vel_.vy = msg->twist.twist.linear.y;
    last_odom_vel_.vz = msg->twist.twist.linear.z;
    last_odom_vel_.wx = msg->twist.twist.angular.x;
    last_odom_vel_.wy = msg->twist.twist.angular.y;
    last_odom_vel_.wz = msg->twist.twist.angular.z;
    last_odom_stamp_ = now();
    has_odom_ = true;
  }

  void onSportModeState(const unitree_go::msg::SportModeState::SharedPtr msg)
  {
    last_sport_mode_vel_.vx = msg->velocity[0];
    last_sport_mode_vel_.vy = msg->velocity[1];
    last_sport_mode_vel_.vz = msg->velocity[2];
    last_sport_mode_vel_.wz = msg->yaw_speed;
    last_sport_mode_stamp_ = now();
    has_sport_mode_ = true;

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "SportModeState velocity vx=%.3f vy=%.3f vz=%.3f wz=%.3f",
      last_sport_mode_vel_.vx, last_sport_mode_vel_.vy,
      last_sport_mode_vel_.vz, last_sport_mode_vel_.wz);
  }

  void onObstacleCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<Eigen::Vector3d> points;
    points.reserve(static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height));

    const std::string map_frame = get_parameter("map_frame").as_string();
    const std::string cloud_frame = msg->header.frame_id.empty() ? map_frame : msg->header.frame_id;
    bool use_transform = cloud_frame != map_frame;
    geometry_msgs::msg::TransformStamped transform;
    if (use_transform) {
      try {
        transform = tf_buffer_.lookupTransform(
          map_frame, cloud_frame, tf2::TimePointZero,
          tf2::durationFromSec(get_parameter("tf_timeout").as_double()));
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Failed to transform local obstacle cloud %s -> %s: %s",
          cloud_frame.c_str(), map_frame.c_str(), ex.what());
        return;
      }
    }

    tf2::Quaternion q(0.0, 0.0, 0.0, 1.0);
    tf2::Vector3 t(0.0, 0.0, 0.0);
    if (use_transform) {
      tf2::fromMsg(transform.transform.rotation, q);
      t = tf2::Vector3(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);
    }
    const tf2::Matrix3x3 rotation(q);

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
          continue;
        }

        if (use_transform) {
          const tf2::Vector3 p(*iter_x, *iter_y, *iter_z);
          const tf2::Vector3 out = rotation * p + t;
          points.emplace_back(out.x(), out.y(), out.z());
        } else {
          points.emplace_back(*iter_x, *iter_y, *iter_z);
        }
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to read local obstacle cloud fields x/y/z: %s", ex.what());
      return;
    }

    planner_.setObstacleCloud(points);
    received_obstacle_cloud_ = true;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000, "Updated local obstacle cloud: %zu points", points.size());
  }

  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "OctoMap frame is %s, expected %s. Tree is still used without transform.",
        msg->header.frame_id.c_str(), map_frame.c_str());
    }

    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
    if (!abstract_tree) {
      RCLCPP_ERROR(get_logger(), "Failed to decode OctoMap message.");
      return;
    }

    auto * oc_tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (!oc_tree) {
      RCLCPP_ERROR(get_logger(), "Decoded OctoMap is not octomap::OcTree.");
      return;
    }

    abstract_tree.release();
    planner_.setOctomap(std::shared_ptr<const octomap::OcTree>(oc_tree));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000, "Updated OctoMap from topic: resolution=%.3f nodes=%zu",
      oc_tree->getResolution(), oc_tree->size());
  }

  void onControlTimer()
  {
    if (!path_ready_ || global_path_.empty()) {
      return;
    }

    Pose3D current_pose;
    if (!lookupCurrentPose(current_pose)) {
      publishZeroCmd();
      return;
    }

    prunePath(current_pose);
    if (global_path_.empty()) {
      publishZeroCmd();
      clearMarkers();
      return;
    }

    const bool use_3d_goal = planner_.getConfig().robot_model == "aerial_3d";
    const double final_goal_distance = groundDistance(current_pose, global_path_.back(), use_3d_goal);
    if (final_goal_distance <= planner_.getConfig().goal_reached_tolerance) {
      publishZeroCmd();
      clearCandidateMarkers();
      clearRecoveryTrajectoryMarker();
      publishLocalGoalMarker(global_path_.back());
      resetStuckState();
      if (!goal_reached_logged_) {
        RCLCPP_INFO(get_logger(), "Goal reached");
        goal_reached_logged_ = true;
      }
      return;
    }

    if (!isObstacleSourceReady()) {
      publishZeroCmd();
      return;
    }

    Velocity3D current_vel = getCurrentVelocity();
    Velocity3D cmd_vel;
    Trajectory3D best_traj;
    const auto compute_start = std::chrono::steady_clock::now();
    const bool ok = planner_.computeVelocityCommand(
      current_pose, current_vel, global_path_, cmd_vel, best_traj);
    const auto compute_end = std::chrono::steady_clock::now();
    const auto compute_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(compute_end - compute_start).count();
    const auto & candidates = planner_.getLastCandidateTrajectories();
    const auto valid_count = std::count_if(
      candidates.begin(), candidates.end(),
      [](const Trajectory3D & traj) { return traj.collision_free; });

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "DWA compute %.0f ms, candidates=%zu valid=%zu pose=(%.2f, %.2f, %.2f) "
      "vel=(%.2f, %.2f, %.2f)",
      static_cast<double>(compute_ms), candidates.size(), valid_count,
      current_pose.x, current_pose.y, current_pose.z,
      current_vel.vx, current_vel.vy, current_vel.wz);

    publishCandidateTrajectories(candidates);
    publishLocalGoalMarker(planner_.getLastLocalGoal());
    publishPathCorridorMarker();

    const bool stuck = isStuck(current_pose, ok);
    if (!ok || stuck) {
      Velocity3D recovery_cmd;
      Trajectory3D recovery_traj;
      if (planner_.computeRecoveryCommand(recovery_cmd, recovery_traj)) {
        publishCmd(recovery_cmd);
        publishRecoveryTrajectoryMarker(recovery_traj);
        if (!ok) {
          clearBestTrajectoryMarker();
        }
        publishDwaDebugText(current_pose, ok, stuck, compute_ms);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "DWA recovery cmd vx=%.3f vy=%.3f wz=%.3f state=%s",
          recovery_cmd.vx, recovery_cmd.vy, recovery_cmd.wz,
          planner_.getLastDebugInfo().recovery_state.c_str());
        return;
      }
    }

    clearRecoveryTrajectoryMarker();
    publishDwaDebugText(current_pose, ok, stuck, compute_ms);
    if (!ok) {
      publishZeroCmd();
      clearBestTrajectoryMarker();
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No valid DWA trajectory and no safe recovery command. Publishing zero velocity.");
      return;
    }

    publishCmd(cmd_vel);
    publishBestTrajectoryMarker(best_traj);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "DWA cmd vx=%.3f vy=%.3f wz=%.3f score=%.3f candidates=%zu",
      cmd_vel.vx, cmd_vel.vy, cmd_vel.wz, best_traj.score,
      planner_.getLastCandidateTrajectories().size());
  }

  bool lookupCurrentPose(Pose3D & pose)
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    std::vector<std::string> candidates =
      splitFrameList(get_parameter("base_frame_candidates").as_string());
    const std::string base_frame = get_parameter("base_frame").as_string();
    if (!base_frame.empty() && std::find(candidates.begin(), candidates.end(), base_frame) == candidates.end()) {
      candidates.insert(candidates.begin(), base_frame);
    }
    if (candidates.empty()) {
      candidates.push_back("base_link");
    }

    std::string last_error;
    for (const auto & candidate : candidates) {
      try {
        const auto tf = tf_buffer_.lookupTransform(
          map_frame, candidate, tf2::TimePointZero,
          tf2::durationFromSec(get_parameter("tf_timeout").as_double()));
        if (active_base_frame_ != candidate) {
          active_base_frame_ = candidate;
          RCLCPP_INFO(get_logger(), "Using local planner base frame: %s", candidate.c_str());
        }
        pose.x = tf.transform.translation.x;
        pose.y = tf.transform.translation.y;
        pose.z = tf.transform.translation.z;
        pose.yaw = tf2::getYaw(tf.transform.rotation);
        return true;
      } catch (const tf2::TransformException & ex) {
        last_error = ex.what();
      }
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Failed to lookup robot pose in %s. Last TF error: %s",
      map_frame.c_str(), last_error.c_str());
    return false;
  }

  Velocity3D getCurrentVelocity()
  {
    const auto source = toLower(trim(get_parameter("velocity_source").as_string()));
    const bool sport_mode_fresh =
      has_sport_mode_ &&
      (now() - last_sport_mode_stamp_).seconds() <= get_parameter("sport_mode_timeout").as_double();
    const bool odom_fresh =
      has_odom_ && (now() - last_odom_stamp_).seconds() <= get_parameter("odom_timeout").as_double();

    if (source == "auto") {
      if (sport_mode_fresh) {
        return last_sport_mode_vel_;
      }
      return odom_fresh ? last_odom_vel_ : last_cmd_vel_;
    }
    if (source == "sport_mode" || source == "sportmodestate" || source == "unitree") {
      if (sport_mode_fresh) {
        return last_sport_mode_vel_;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "velocity_source=%s, but /sportmodestate is not fresh. Using zero velocity.",
        source.c_str());
      return Velocity3D();
    }
    if (source == "odom") {
      return odom_fresh ? last_odom_vel_ : Velocity3D();
    }
    if (source == "last_cmd" || source == "cmd_vel") {
      return last_cmd_vel_;
    }
    if (source == "zero" || source == "none") {
      return Velocity3D();
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Unknown velocity_source=%s. Valid values: auto, sport_mode, odom, last_cmd, zero. "
      "Falling back to last_cmd.",
      source.c_str());
    return last_cmd_vel_;
  }

  bool isObstacleSourceReady()
  {
    const auto & config = planner_.getConfig();
    const bool wants_octomap = config.obstacle_source == "octomap" || config.obstacle_source == "both";
    const bool wants_pointcloud = config.obstacle_source == "pointcloud" || config.obstacle_source == "both";

    if (config.obstacle_source != "octomap" &&
      config.obstacle_source != "pointcloud" &&
      config.obstacle_source != "both")
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Unknown obstacle_source=%s. Valid values: octomap, pointcloud, both.",
        config.obstacle_source.c_str());
      return false;
    }

    if (wants_octomap && planner_.hasOctomap()) {
      return true;
    }
    if (wants_pointcloud && received_obstacle_cloud_) {
      return true;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Obstacle source %s is not ready. Holding zero velocity.",
      config.obstacle_source.c_str());
    return false;
  }

  void prunePath(const Pose3D & current_pose)
  {
    if (global_path_.size() < 2) {
      return;
    }

    const bool use_3d = planner_.getConfig().robot_model == "aerial_3d";
    std::size_t nearest_index = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < global_path_.size(); ++i) {
      const double distance = groundDistance(current_pose, global_path_[i], use_3d);
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_index = i;
      }
    }

    if (nearest_index > 0 &&
      nearest_distance <= planner_.getConfig().path_prune_distance &&
      nearest_index < global_path_.size())
    {
      global_path_.erase(global_path_.begin(), global_path_.begin() + static_cast<std::ptrdiff_t>(nearest_index));
    }
  }

  void resetStuckState()
  {
    progress_pose_initialized_ = false;
    no_valid_active_ = false;
    stuck_state_ = false;
  }

  bool isStuck(const Pose3D & current_pose, bool has_valid_trajectory)
  {
    const auto & config = planner_.getConfig();
    if (!config.stuck_detection_enabled) {
      stuck_state_ = false;
      return false;
    }

    const auto stamp = now();
    if (!progress_pose_initialized_) {
      last_progress_pose_ = current_pose;
      last_progress_stamp_ = stamp;
      progress_pose_initialized_ = true;
      if (!has_valid_trajectory) {
        no_valid_since_ = stamp;
        no_valid_active_ = true;
      }
      stuck_state_ = false;
      return false;
    }

    const double progress = std::hypot(
      current_pose.x - last_progress_pose_.x,
      current_pose.y - last_progress_pose_.y);
    if (progress >= config.min_progress_distance) {
      last_progress_pose_ = current_pose;
      last_progress_stamp_ = stamp;
    }

    if (!has_valid_trajectory) {
      if (!no_valid_active_) {
        no_valid_since_ = stamp;
        no_valid_active_ = true;
      }
    } else {
      no_valid_active_ = false;
    }

    const double commanded_planar = std::hypot(last_cmd_vel_.vx, last_cmd_vel_.vy);
    const bool command_active = commanded_planar > 0.03 || std::abs(last_cmd_vel_.wz) > 0.08;
    const double stuck_threshold = std::max(0.1, config.stuck_time_threshold);
    const bool no_valid_stuck =
      no_valid_active_ && (stamp - no_valid_since_).seconds() >= stuck_threshold;
    const bool low_progress_stuck =
      command_active && (stamp - last_progress_stamp_).seconds() >= stuck_threshold;

    stuck_state_ = no_valid_stuck || low_progress_stuck;
    return stuck_state_;
  }

  void publishDwaDebugText(
    const Pose3D & current_pose,
    bool dwa_ok,
    bool stuck,
    std::int64_t compute_ms)
  {
    if (!planner_.getConfig().debug_dwa) {
      return;
    }

    const auto & debug = planner_.getLastDebugInfo();
    const auto & local_goal = planner_.getLastLocalGoal();
    std::ostringstream ss;
    ss << "valid_trajectories=" << debug.valid_trajectories << '\n';
    ss << "collision_trajectories=" << debug.collision_trajectories << '\n';
    ss << "unknown_blocked_count=" << debug.unknown_blocked_count << '\n';
    ss << "ground_blocked_count=" << debug.ground_blocked_count << '\n';
    ss << "best_score=";
    if (std::isfinite(debug.best_score)) {
      ss << debug.best_score;
    } else {
      ss << "none";
    }
    ss << '\n';
    ss << "stuck_state=" << (stuck ? "true" : "false") << '\n';
    ss << "recovery_state=" << debug.recovery_state << '\n';
    ss << "dwa_ok=" << (dwa_ok ? "true" : "false") << '\n';
    ss << "compute_ms=" << compute_ms << '\n';
    ss << "local_goal_xyz="
       << local_goal.x() << ',' << local_goal.y() << ',' << local_goal.z() << '\n';
    ss << "current_pose_xyz="
       << current_pose.x << ',' << current_pose.y << ',' << current_pose.z;

    std_msgs::msg::String msg;
    msg.data = ss.str();
    debug_text_pub_->publish(msg);
  }

  void publishCmd(const Velocity3D & cmd)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd.vx;
    twist.linear.y = cmd.vy;
    twist.linear.z = cmd.vz;
    twist.angular.x = cmd.wx;
    twist.angular.y = cmd.wy;
    twist.angular.z = cmd.wz;
    cmd_pub_->publish(twist);
    last_cmd_vel_ = cmd;
  }

  void publishZeroCmd()
  {
    publishCmd(Velocity3D());
  }

  void publishBestTrajectoryMarker(const Trajectory3D & traj)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.ns = "dwa_best_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.055;
    marker.color.r = 0.05f;
    marker.color.g = 0.95f;
    marker.color.b = 1.0f;
    marker.color.a = 0.95f;
    marker.lifetime = makeDurationMsg(get_parameter("marker_lifetime").as_double());

    marker.points.reserve(traj.poses.size());
    for (const auto & pose : traj.poses) {
      marker.points.push_back(makePoint(pose.x, pose.y, pose.z));
    }
    best_traj_pub_->publish(marker);
  }

  void clearBestTrajectoryMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.ns = "dwa_best_trajectory";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    best_traj_pub_->publish(marker);
  }

  void publishRecoveryTrajectoryMarker(const Trajectory3D & traj)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.ns = "dwa_recovery_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.065;
    marker.color.r = 1.0f;
    marker.color.g = 0.55f;
    marker.color.b = 0.05f;
    marker.color.a = 0.95f;
    marker.lifetime = makeDurationMsg(get_parameter("marker_lifetime").as_double());
    marker.points.reserve(traj.poses.size());
    for (const auto & pose : traj.poses) {
      marker.points.push_back(makePoint(pose.x, pose.y, pose.z));
    }
    recovery_traj_pub_->publish(marker);
  }

  void clearRecoveryTrajectoryMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.ns = "dwa_recovery_trajectory";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    recovery_traj_pub_->publish(marker);
  }

  void publishPathCorridorMarker()
  {
    if (global_path_.empty()) {
      clearPathCorridorMarker();
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.ns = "dwa_path_corridor";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = std::max(0.03, 2.0 * planner_.getConfig().path_corridor_radius);
    marker.color.r = 0.25f;
    marker.color.g = 0.55f;
    marker.color.b = 1.0f;
    marker.color.a = 0.18f;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = makeDurationMsg(get_parameter("marker_lifetime").as_double());
    marker.points.reserve(global_path_.size());
    for (const auto & point : global_path_) {
      marker.points.push_back(makePoint(point.x(), point.y(), point.z()));
    }
    path_corridor_pub_->publish(marker);
  }

  void clearPathCorridorMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.ns = "dwa_path_corridor";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    path_corridor_pub_->publish(marker);
  }

  void publishCandidateTrajectories(const std::vector<Trajectory3D> & trajectories)
  {
    if (!get_parameter("publish_candidate_trajectories").as_bool()) {
      clearCandidateMarkers();
      return;
    }

    const int stride = std::max(1, static_cast<int>(get_parameter("candidate_marker_stride").as_int()));
    visualization_msgs::msg::MarkerArray array;
    int marker_id = 0;
    for (std::size_t i = 0; i < trajectories.size(); i += static_cast<std::size_t>(stride)) {
      const auto & traj = trajectories[i];
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now();
      marker.header.frame_id = get_parameter("map_frame").as_string();
      marker.ns = "dwa_candidate_trajectories";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = traj.collision_free ? 0.018 : 0.012;
      marker.color.r = traj.collision_free ? 0.25f : 1.0f;
      marker.color.g = traj.collision_free ? 0.75f : 0.18f;
      marker.color.b = traj.collision_free ? 0.35f : 0.10f;
      marker.color.a = traj.collision_free ? 0.28f : 0.12f;
      marker.lifetime = makeDurationMsg(get_parameter("marker_lifetime").as_double());
      marker.points.reserve(traj.poses.size());
      for (const auto & pose : traj.poses) {
        marker.points.push_back(makePoint(pose.x, pose.y, pose.z));
      }
      array.markers.push_back(std::move(marker));
    }

    for (int id = marker_id; id < previous_candidate_marker_count_; ++id) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now();
      marker.header.frame_id = get_parameter("map_frame").as_string();
      marker.ns = "dwa_candidate_trajectories";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(std::move(marker));
    }

    previous_candidate_marker_count_ = marker_id;
    candidate_traj_pub_->publish(array);
  }

  void clearCandidateMarkers()
  {
    if (previous_candidate_marker_count_ <= 0) {
      return;
    }

    visualization_msgs::msg::MarkerArray array;
    for (int id = 0; id < previous_candidate_marker_count_; ++id) {
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = now();
      marker.header.frame_id = get_parameter("map_frame").as_string();
      marker.ns = "dwa_candidate_trajectories";
      marker.id = id;
      marker.action = visualization_msgs::msg::Marker::DELETE;
      array.markers.push_back(std::move(marker));
    }
    previous_candidate_marker_count_ = 0;
    candidate_traj_pub_->publish(array);
  }

  void publishLocalGoalMarker(const Eigen::Vector3d & point)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.ns = "dwa_local_goal";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = makePoint(point.x(), point.y(), point.z());
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.22;
    marker.scale.y = 0.22;
    marker.scale.z = 0.22;
    marker.color.r = 1.0f;
    marker.color.g = 0.72f;
    marker.color.b = 0.05f;
    marker.color.a = 0.95f;
    marker.lifetime = makeDurationMsg(get_parameter("marker_lifetime").as_double());
    local_goal_pub_->publish(marker);
  }

  void clearLocalGoalMarker()
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.ns = "dwa_local_goal";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    local_goal_pub_->publish(marker);
  }

  void clearMarkers()
  {
    clearBestTrajectoryMarker();
    clearCandidateMarkers();
    clearLocalGoalMarker();
    clearRecoveryTrajectoryMarker();
    clearPathCorridorMarker();
  }

  DWA3DLocalPlanner planner_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr additional_path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sport_mode_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr obstacle_cloud_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr best_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr candidate_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr recovery_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_corridor_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_text_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::vector<Eigen::Vector3d> global_path_;
  Velocity3D last_odom_vel_;
  Velocity3D last_sport_mode_vel_;
  Velocity3D last_cmd_vel_;
  rclcpp::Time last_odom_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sport_mode_stamp_{0, 0, RCL_ROS_TIME};
  bool path_ready_{false};
  bool has_odom_{false};
  bool has_sport_mode_{false};
  bool received_obstacle_cloud_{false};
  bool goal_reached_logged_{false};
  int previous_candidate_marker_count_{0};
  std::string active_base_frame_;
  Pose3D last_progress_pose_;
  rclcpp::Time last_progress_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time no_valid_since_{0, 0, RCL_ROS_TIME};
  bool progress_pose_initialized_{false};
  bool no_valid_active_{false};
  bool stuck_state_{false};
};

}  // namespace local_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<local_planning::LocalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
