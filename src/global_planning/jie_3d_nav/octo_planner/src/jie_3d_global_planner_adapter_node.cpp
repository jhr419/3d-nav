#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/time.h"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

using namespace std::chrono_literals;

namespace
{
rclcpp::QoS latchedQos()
{
  return rclcpp::QoS(1).transient_local().reliable();
}

std::string trim(const std::string & value)
{
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

bool isAbsolutePath(const std::string & path)
{
  return !path.empty() && path.front() == '/';
}

std::filesystem::path findWorkspaceRoot()
{
  const char * env_value = std::getenv("NAV3D_WS");
  if (env_value != nullptr && *env_value != '\0') {
    std::filesystem::path env_path(env_value);
    if (std::filesystem::is_directory(env_path / "src") &&
      std::filesystem::is_directory(env_path / "maps"))
    {
      return env_path;
    }
  }

  std::filesystem::path cwd = std::filesystem::current_path();
  for (auto candidate = cwd; !candidate.empty(); candidate = candidate.parent_path()) {
    if (std::filesystem::is_directory(candidate / "src") &&
      std::filesystem::is_directory(candidate / "maps"))
    {
      return candidate;
    }
    if (candidate == candidate.root_path()) {
      break;
    }
  }

  return cwd;
}

std::string resolveWorkspacePath(const std::string & raw_path)
{
  const std::string path = trim(raw_path);
  if (path.empty()) {
    return "";
  }
  if (isAbsolutePath(path)) {
    return path;
  }
  if (path.front() == '~') {
    const char * home = std::getenv("HOME");
    if (home != nullptr) {
      return (std::filesystem::path(home) / path.substr(1)).lexically_normal().string();
    }
  }
  return (findWorkspaceRoot() / path).lexically_normal().string();
}

bool validQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double norm =
    q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
  return std::isfinite(norm) && norm > 1.0e-12;
}

}  // namespace

class Jie3DGlobalPlannerAdapterNode : public rclcpp::Node
{
public:
  Jie3DGlobalPlannerAdapterNode()
  : Node("jie_3d_global_planner_adapter"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
  {
    declareParameters();
    readParameters();

    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, latchedQos());
    start_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      internal_start_topic_, latchedQos());
    goal_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      internal_goal_point_topic_, latchedQos());
    goal_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      internal_goal_pose_topic_, latchedQos());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(publish_path_topic_, latchedQos());
    if (!publish_alias_path_topic_.empty() && publish_alias_path_topic_ != publish_path_topic_) {
      alias_path_pub_ =
        create_publisher<nav_msgs::msg::Path>(publish_alias_path_topic_, latchedQos());
    }
    marker_pub_ =
      create_publisher<visualization_msgs::msg::Marker>(publish_marker_topic_, latchedQos());
    octomap_pub_ =
      create_publisher<octomap_msgs::msg::Octomap>(octomap_topic_, latchedQos());

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_, latchedQos(),
      std::bind(&Jie3DGlobalPlannerAdapterNode::onOctomap, this, std::placeholders::_1));
    raw_path_sub_ = create_subscription<nav_msgs::msg::Path>(
      raw_path_topic_, latchedQos(),
      std::bind(&Jie3DGlobalPlannerAdapterNode::onRawPath, this, std::placeholders::_1));

    if (!goal_pose_topic_.empty()) {
      goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_, latchedQos(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          onGoalPose(msg, false, "goal_pose_topic");
        });
    }
    if (!rviz_2d_goal_topic_.empty() && rviz_2d_goal_topic_ != goal_pose_topic_) {
      rviz_2d_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        rviz_2d_goal_topic_, latchedQos(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          onGoalPose(msg, true, "rviz_2d_goal_topic");
        });
    }
    if (!goal_point_topic_.empty()) {
      goal_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        goal_point_topic_, latchedQos(),
        std::bind(&Jie3DGlobalPlannerAdapterNode::onGoalPoint, this, std::placeholders::_1));
    }
    if (!manual_start_point_topic_.empty()) {
      manual_start_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        manual_start_point_topic_, latchedQos(),
        std::bind(&Jie3DGlobalPlannerAdapterNode::onManualStartPoint, this, std::placeholders::_1));
    }
    if (!manual_start_pose_topic_.empty()) {
      manual_start_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        manual_start_pose_topic_, latchedQos(),
        std::bind(&Jie3DGlobalPlannerAdapterNode::onManualStartPose, this, std::placeholders::_1));
    }

    status_timer_ = create_wall_timer(1s, [this]() { publishStatus(); });
    planning_timeout_timer_ = create_wall_timer(
      500ms, std::bind(&Jie3DGlobalPlannerAdapterNode::checkPlanningTimeout, this));

    setStatus("WAITING_FOR_MAP");
    loadOctomapFileIfAvailable();

    RCLCPP_INFO(
      get_logger(),
      "jie_3d_global_planner_adapter ready. start_source=%s start_internal=%s "
      "goal=[%s, %s] raw_path=%s output=[%s, %s] marker=%s octomap=%s",
      start_source_.c_str(), internal_start_topic_.c_str(), goal_pose_topic_.c_str(),
      goal_point_topic_.c_str(), raw_path_topic_.c_str(), publish_path_topic_.c_str(),
      publish_alias_path_topic_.c_str(), publish_marker_topic_.c_str(), octomap_topic_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("start_source", "tf");
    declare_parameter<std::string>("goal_pose_topic", "/goal_pose_3d");
    declare_parameter<std::string>("goal_point_topic", "/goal_point_3d");
    declare_parameter<std::string>("rviz_2d_goal_topic", "/goal_pose");
    declare_parameter<std::string>("manual_start_point_topic", "/jie_3d_nav/manual_start_point");
    declare_parameter<std::string>("manual_start_pose_topic", "/jie_3d_nav/manual_start_pose");
    declare_parameter<std::string>("internal_start_topic", "/jie_3d_nav/start_point");
    declare_parameter<std::string>("internal_goal_point_topic", "/jie_3d_nav/goal_point");
    declare_parameter<std::string>("internal_goal_pose_topic", "/jie_3d_nav/goal_pose");
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("octomap_file", "maps/map_preprocessed.bt");
    declare_parameter<std::string>("map_pcd_file", "maps/map_preprocessed.pcd");
    declare_parameter<double>("resolution", 0.2);
    declare_parameter<bool>("publish_octomap_from_file", true);
    declare_parameter<double>("octomap_publish_rate", 1.0);
    declare_parameter<std::string>("raw_path_topic", "/jie_3d_nav/planned_path_raw");
    declare_parameter<std::string>("publish_path_topic", "/planned_path");
    declare_parameter<std::string>("publish_alias_path_topic", "/path");
    declare_parameter<std::string>("publish_marker_topic", "/planned_path_marker");
    declare_parameter<std::string>("status_topic", "/jie_3d_global_planner/status");
    declare_parameter<double>("tf_lookup_timeout", 0.25);
    declare_parameter<double>("planning_timeout", 8.0);
    declare_parameter<double>("marker_line_width", 0.08);
    declare_parameter<double>("default_goal_z", 0.0);
    declare_parameter<bool>("start_z_override_enabled", false);
    declare_parameter<double>("start_z_override", 0.0);
  }

  void readParameters()
  {
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    start_source_ = trim(get_parameter("start_source").as_string());
    std::transform(start_source_.begin(), start_source_.end(), start_source_.begin(), ::tolower);
    if (start_source_.empty()) {
      start_source_ = "tf";
    }
    goal_pose_topic_ = get_parameter("goal_pose_topic").as_string();
    goal_point_topic_ = get_parameter("goal_point_topic").as_string();
    rviz_2d_goal_topic_ = get_parameter("rviz_2d_goal_topic").as_string();
    manual_start_point_topic_ = get_parameter("manual_start_point_topic").as_string();
    manual_start_pose_topic_ = get_parameter("manual_start_pose_topic").as_string();
    internal_start_topic_ = get_parameter("internal_start_topic").as_string();
    internal_goal_point_topic_ = get_parameter("internal_goal_point_topic").as_string();
    internal_goal_pose_topic_ = get_parameter("internal_goal_pose_topic").as_string();
    octomap_topic_ = get_parameter("octomap_topic").as_string();
    octomap_file_ = resolveWorkspacePath(get_parameter("octomap_file").as_string());
    map_pcd_file_ = resolveWorkspacePath(get_parameter("map_pcd_file").as_string());
    resolution_ = get_parameter("resolution").as_double();
    publish_octomap_from_file_ = get_parameter("publish_octomap_from_file").as_bool();
    octomap_publish_rate_ = std::max(0.1, get_parameter("octomap_publish_rate").as_double());
    raw_path_topic_ = get_parameter("raw_path_topic").as_string();
    publish_path_topic_ = get_parameter("publish_path_topic").as_string();
    publish_alias_path_topic_ = get_parameter("publish_alias_path_topic").as_string();
    publish_marker_topic_ = get_parameter("publish_marker_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    tf_lookup_timeout_ = get_parameter("tf_lookup_timeout").as_double();
    planning_timeout_ = get_parameter("planning_timeout").as_double();
    marker_line_width_ = get_parameter("marker_line_width").as_double();
    default_goal_z_ = get_parameter("default_goal_z").as_double();
    start_z_override_enabled_ = get_parameter("start_z_override_enabled").as_bool();
    start_z_override_ = get_parameter("start_z_override").as_double();
  }

  void loadOctomapFileIfAvailable()
  {
    if (!publish_octomap_from_file_ || octomap_file_.empty()) {
      explainExternalMapSource();
      return;
    }
    if (!std::filesystem::is_regular_file(octomap_file_)) {
      explainExternalMapSource();
      return;
    }

    auto tree = std::make_shared<octomap::OcTree>(resolution_);
    if (!tree->readBinary(octomap_file_)) {
      RCLCPP_ERROR(get_logger(), "Failed to read OctoMap binary file: %s", octomap_file_.c_str());
      setStatus("MAP_ERROR");
      return;
    }
    tree->updateInnerOccupancy();
    file_octree_ = tree;
    map_ready_ = true;
    setStatus("WAITING_FOR_GOAL");
    publishFileOctomap();
    const auto period = std::chrono::duration<double>(1.0 / octomap_publish_rate_);
    octomap_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Jie3DGlobalPlannerAdapterNode::publishFileOctomap, this));
    RCLCPP_INFO(get_logger(), "Loaded OctoMap file: %s", octomap_file_.c_str());
  }

  void explainExternalMapSource()
  {
    if (std::filesystem::is_regular_file(map_pcd_file_)) {
      RCLCPP_WARN(
        get_logger(),
        "OctoMap file is not available (%s). Waiting for PCD-to-OctoMap publisher from: %s",
        octomap_file_.c_str(), map_pcd_file_.c_str());
      return;
    }
    RCLCPP_ERROR(
      get_logger(),
      "No OctoMap or PCD map found. Expected octomap_file=%s or map_pcd_file=%s",
      octomap_file_.c_str(), map_pcd_file_.c_str());
    setStatus("MAP_ERROR");
  }

  void publishFileOctomap()
  {
    if (!file_octree_) {
      return;
    }
    octomap_msgs::msg::Octomap msg;
    if (!octomap_msgs::binaryMapToMsg(*file_octree_, msg)) {
      RCLCPP_ERROR(get_logger(), "Failed to convert OctoMap file to ROS message.");
      setStatus("MAP_ERROR");
      return;
    }
    msg.header.stamp = now();
    msg.header.frame_id = map_frame_;
    octomap_pub_->publish(msg);
  }

  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    if (msg->data.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Received empty OctoMap.");
      return;
    }

    const bool was_ready = map_ready_;
    map_ready_ = true;
    if (!has_goal_ && status_ != "PLANNING" && status_ != "SUCCESS") {
      setStatus("WAITING_FOR_GOAL");
    }
    if (!was_ready && pending_goal_pose_) {
      RCLCPP_INFO(get_logger(), "OctoMap is ready; planning to the pending goal.");
      sendGoalToCore(*pending_goal_pose_);
    }
  }

  void onManualStartPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    manual_start_ = transformPointToMap(*msg);
    if (manual_start_) {
      RCLCPP_INFO(
        get_logger(), "Manual start set: [%.3f, %.3f, %.3f]",
        manual_start_->point.x, manual_start_->point.y, manual_start_->point.z);
    }
  }

  void onManualStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PointStamped point;
    point.header = msg->header;
    point.point = msg->pose.position;
    manual_start_ = transformPointToMap(point);
    if (manual_start_) {
      RCLCPP_INFO(
        get_logger(), "Manual start pose set: [%.3f, %.3f, %.3f]",
        manual_start_->point.x, manual_start_->point.y, manual_start_->point.z);
    }
  }

  void onGoalPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    const auto goal_point = transformPointToMap(*msg);
    if (!goal_point) {
      setStatus("TF_ERROR");
      return;
    }

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header = goal_point->header;
    goal_pose.pose.position = goal_point->point;
    goal_pose.pose.orientation.w = 1.0;
    pending_goal_pose_ = goal_pose;
    has_goal_ = true;
    RCLCPP_INFO(
      get_logger(), "Received 3D goal point: [%.3f, %.3f, %.3f]",
      goal_point->point.x, goal_point->point.y, goal_point->point.z);
    sendGoalToCore(goal_pose);
  }

  void onGoalPose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    bool force_default_z,
    const std::string & source)
  {
    geometry_msgs::msg::PoseStamped pose = *msg;
    if (force_default_z) {
      pose.pose.position.z = default_goal_z_;
    }
    const auto goal_pose = transformPoseToMap(pose);
    if (!goal_pose) {
      setStatus("TF_ERROR");
      return;
    }

    pending_goal_pose_ = *goal_pose;
    has_goal_ = true;
    RCLCPP_INFO(
      get_logger(), "Received goal from %s: [%.3f, %.3f, %.3f]",
      source.c_str(), goal_pose->pose.position.x, goal_pose->pose.position.y,
      goal_pose->pose.position.z);
    sendGoalToCore(*goal_pose);
  }

  void sendGoalToCore(const geometry_msgs::msg::PoseStamped & goal_pose)
  {
    if (!map_ready_) {
      RCLCPP_WARN(get_logger(), "Goal received, but OctoMap is not ready yet.");
      setStatus("WAITING_FOR_MAP");
      return;
    }

    const auto start = currentStart();
    if (!start) {
      setStatus(start_source_ == "tf" ? "TF_ERROR" : "FAILED");
      return;
    }

    geometry_msgs::msg::PoseStamped normalized_goal = goal_pose;
    normalized_goal.header.stamp = now();
    normalized_goal.header.frame_id = map_frame_;
    if (!validQuaternion(normalized_goal.pose.orientation)) {
      normalized_goal.pose.orientation.w = 1.0;
    }

    geometry_msgs::msg::PointStamped goal_point;
    goal_point.header = normalized_goal.header;
    goal_point.point = normalized_goal.pose.position;

    start_pub_->publish(*start);
    goal_pose_pub_->publish(normalized_goal);
    goal_point_pub_->publish(goal_point);

    planning_ = true;
    planning_started_ = now();
    setStatus("PLANNING");
  }

  std::optional<geometry_msgs::msg::PointStamped> currentStart()
  {
    geometry_msgs::msg::PointStamped start;
    if (start_source_ == "tf") {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero,
          tf2::durationFromSec(std::max(0.0, tf_lookup_timeout_)));
        start.header.stamp = now();
        start.header.frame_id = map_frame_;
        start.point.x = transform.transform.translation.x;
        start.point.y = transform.transform.translation.y;
        start.point.z = transform.transform.translation.z;
      } catch (const std::exception & exc) {
        RCLCPP_WARN(
          get_logger(), "Cannot query TF %s -> %s for planner start: %s",
          map_frame_.c_str(), base_frame_.c_str(), exc.what());
        return std::nullopt;
      }
    } else if (start_source_ == "manual" || start_source_ == "topic") {
      if (!manual_start_) {
        RCLCPP_WARN(
          get_logger(), "start_source=%s but no manual/topic start has been received.",
          start_source_.c_str());
        return std::nullopt;
      }
      start = *manual_start_;
      start.header.stamp = now();
    } else {
      RCLCPP_ERROR(get_logger(), "Unsupported start_source='%s'. Use tf, manual, or topic.", start_source_.c_str());
      return std::nullopt;
    }

    if (start_z_override_enabled_) {
      start.point.z = start_z_override_;
    }
    return start;
  }

  std::optional<geometry_msgs::msg::PointStamped> transformPointToMap(
    const geometry_msgs::msg::PointStamped & input)
  {
    const std::string source_frame = input.header.frame_id.empty() ? map_frame_ : input.header.frame_id;
    if (source_frame == map_frame_) {
      geometry_msgs::msg::PointStamped out = input;
      out.header.frame_id = map_frame_;
      out.header.stamp = now();
      return out;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(std::max(0.0, tf_lookup_timeout_)));
      geometry_msgs::msg::PointStamped out;
      tf2::doTransform(input, out, transform);
      out.header.frame_id = map_frame_;
      out.header.stamp = now();
      return out;
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        get_logger(), "Cannot transform point from %s to %s: %s",
        source_frame.c_str(), map_frame_.c_str(), exc.what());
      return std::nullopt;
    }
  }

  std::optional<geometry_msgs::msg::PoseStamped> transformPoseToMap(
    const geometry_msgs::msg::PoseStamped & input)
  {
    const std::string source_frame = input.header.frame_id.empty() ? map_frame_ : input.header.frame_id;
    if (source_frame == map_frame_) {
      geometry_msgs::msg::PoseStamped out = input;
      out.header.frame_id = map_frame_;
      out.header.stamp = now();
      return out;
    }

    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_, source_frame, tf2::TimePointZero,
        tf2::durationFromSec(std::max(0.0, tf_lookup_timeout_)));
      geometry_msgs::msg::PoseStamped out;
      tf2::doTransform(input, out, transform);
      out.header.frame_id = map_frame_;
      out.header.stamp = now();
      return out;
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        get_logger(), "Cannot transform pose from %s to %s: %s",
        source_frame.c_str(), map_frame_.c_str(), exc.what());
      return std::nullopt;
    }
  }

  void onRawPath(const nav_msgs::msg::Path::SharedPtr msg)
  {
    nav_msgs::msg::Path path = normalizePath(*msg);
    if (path.poses.empty()) {
      publishPath(path);
      publishDeleteMarker(path.header);
      if (planning_) {
        planning_ = false;
        setStatus("FAILED");
      } else if (map_ready_) {
        setStatus(has_goal_ ? "FAILED" : "WAITING_FOR_GOAL");
      }
      return;
    }

    publishPath(path);
    publishMarker(path);
    planning_ = false;
    setStatus("SUCCESS");
    RCLCPP_INFO(get_logger(), "Published jie_octomap global path: poses=%zu", path.poses.size());
  }

  nav_msgs::msg::Path normalizePath(const nav_msgs::msg::Path & input)
  {
    nav_msgs::msg::Path output;
    output.header.stamp = now();
    output.header.frame_id = map_frame_;
    output.poses.reserve(input.poses.size());

    const std::string path_frame = input.header.frame_id.empty() ? map_frame_ : input.header.frame_id;
    for (const auto & pose : input.poses) {
      geometry_msgs::msg::PoseStamped normalized = pose;
      if (normalized.header.frame_id.empty()) {
        normalized.header.frame_id = path_frame;
      }

      if (normalized.header.frame_id != map_frame_) {
        const auto transformed = transformPoseToMap(normalized);
        if (!transformed) {
          continue;
        }
        normalized = *transformed;
      }
      normalized.header = output.header;
      if (!validQuaternion(normalized.pose.orientation)) {
        normalized.pose.orientation.w = 1.0;
      }
      output.poses.push_back(normalized);
    }
    return output;
  }

  void publishPath(const nav_msgs::msg::Path & path)
  {
    path_pub_->publish(path);
    if (alias_path_pub_) {
      alias_path_pub_->publish(path);
    }
  }

  void publishMarker(const nav_msgs::msg::Path & path)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "jie_3d_global_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_line_width_;
    marker.color.r = 0.1F;
    marker.color.g = 0.95F;
    marker.color.b = 0.95F;
    marker.color.a = 1.0F;
    marker.points.reserve(path.poses.size());
    for (const auto & pose : path.poses) {
      geometry_msgs::msg::Point point;
      point.x = pose.pose.position.x;
      point.y = pose.pose.position.y;
      point.z = pose.pose.position.z;
      marker.points.push_back(point);
    }
    marker_pub_->publish(marker);
  }

  void publishDeleteMarker(const std_msgs::msg::Header & header)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "jie_3d_global_path";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker_pub_->publish(marker);
  }

  void checkPlanningTimeout()
  {
    if (!planning_) {
      return;
    }
    const double elapsed = (now() - planning_started_).seconds();
    if (elapsed <= planning_timeout_) {
      return;
    }
    planning_ = false;
    RCLCPP_WARN(get_logger(), "jie_octomap planning timed out after %.2f seconds.", elapsed);
    setStatus("FAILED");
  }

  void setStatus(const std::string & status)
  {
    status_ = status;
    publishStatus();
  }

  void publishStatus()
  {
    if (!status_pub_) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = status_;
    status_pub_->publish(msg);
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<octomap::OcTree> file_octree_;

  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string start_source_;
  std::string goal_pose_topic_;
  std::string goal_point_topic_;
  std::string rviz_2d_goal_topic_;
  std::string manual_start_point_topic_;
  std::string manual_start_pose_topic_;
  std::string internal_start_topic_;
  std::string internal_goal_point_topic_;
  std::string internal_goal_pose_topic_;
  std::string octomap_topic_;
  std::string octomap_file_;
  std::string map_pcd_file_;
  std::string raw_path_topic_;
  std::string publish_path_topic_;
  std::string publish_alias_path_topic_;
  std::string publish_marker_topic_;
  std::string status_topic_;
  std::string status_{"IDLE"};
  double resolution_{0.2};
  bool publish_octomap_from_file_{true};
  double octomap_publish_rate_{1.0};
  double tf_lookup_timeout_{0.25};
  double planning_timeout_{8.0};
  double marker_line_width_{0.08};
  double default_goal_z_{0.0};
  bool start_z_override_enabled_{false};
  double start_z_override_{0.0};
  bool map_ready_{false};
  bool has_goal_{false};
  bool planning_{false};
  rclcpp::Time planning_started_{0, 0, RCL_ROS_TIME};

  std::optional<geometry_msgs::msg::PointStamped> manual_start_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_goal_pose_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr alias_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr raw_path_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_2d_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr manual_start_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr manual_start_pose_sub_;
  rclcpp::TimerBase::SharedPtr octomap_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::TimerBase::SharedPtr planning_timeout_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Jie3DGlobalPlannerAdapterNode>());
  rclcpp::shutdown();
  return 0;
}
