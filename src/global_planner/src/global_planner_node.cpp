#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "global_planner/global_planner_core.hpp"
#include "global_planner/octomap_voxel_map.hpp"
#include "nav_msgs/msg/path.hpp"
#include "octomap/AbstractOcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#endif
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace global_planner
{

namespace
{
Eigen::Vector3d toEigen(const geometry_msgs::msg::Point & point)
{
  return Eigen::Vector3d(point.x, point.y, point.z);
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d & point)
{
  geometry_msgs::msg::Point out;
  out.x = point.x();
  out.y = point.y();
  out.z = point.z();
  return out;
}

double pathLength(const std::vector<Eigen::Vector3d> & path)
{
  double length = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    length += (path[i] - path[i - 1]).norm();
  }
  return length;
}
}  // namespace

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declareParameters();

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    const auto goal_pose_topic = get_parameter("goal_pose_topic").as_string();
    const auto goal_point_topic = get_parameter("goal_point_topic").as_string();
    const auto initial_pose_topic = get_parameter("initial_pose_topic").as_string();
    const auto octomap_topic = get_parameter("octomap_topic").as_string();

    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic, rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onGoalPose, this, std::placeholders::_1));
    goal_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      goal_point_topic, rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onGoalPoint, this, std::placeholders::_1));
    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      initial_pose_topic, rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onInitialPose, this, std::placeholders::_1));
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&GlobalPlannerNode::onOctomap, this, std::placeholders::_1));

    global_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("global_path_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    global_path_3d_pub_ = create_publisher<nav_msgs::msg::Path>(
      get_parameter("global_path_3d_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    path_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("global_path_marker_topic").as_string(),
      rclcpp::QoS(1).transient_local().reliable());
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable());
    occupied_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      get_parameter("octomap_cloud_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());
    occupied_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      get_parameter("occupied_marker_topic").as_string(), rclcpp::QoS(1).transient_local().reliable());

    loadMapFromParameter();
    publish_map_timer_ = create_wall_timer(
      std::chrono::seconds(2), std::bind(&GlobalPlannerNode::publishMapProducts, this));

    RCLCPP_INFO(
      get_logger(),
      "global_planner_node started. goal_pose=%s goal_point=%s initial_pose=%s "
      "start_source=%s path=%s path_3d=%s marker=%s",
      goal_pose_topic.c_str(), goal_point_topic.c_str(), initial_pose_topic.c_str(),
      get_parameter("start_source").as_string().c_str(),
      get_parameter("global_path_topic").as_string().c_str(),
      get_parameter("global_path_3d_topic").as_string().c_str(),
      get_parameter("global_path_marker_topic").as_string().c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("map_file", "/home/jhr/jhr/fast_ws/maps/result_cleaned.bt");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<double>("map_resolution", 0.2);
    declare_parameter<double>("inflation_radius", 0.3);
    declare_parameter<bool>("unknown_as_occupied", false);
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("octomap_cloud_topic", "/octomap_points");
    declare_parameter<std::string>("occupied_marker_topic", "/octomap_occupied_markers");
    declare_parameter<bool>("accept_octomap_topic", false);
    declare_parameter<bool>("publish_octomap", true);
    declare_parameter<bool>("publish_occupied_cloud", true);
    declare_parameter<bool>("publish_occupied_marker", true);

    declare_parameter<std::string>("goal_pose_topic", "/goal_pose_3d");
    declare_parameter<std::string>("goal_point_topic", "/goal_point_3d");
    declare_parameter<std::string>("initial_pose_topic", "/initial_pose_3d");
    declare_parameter<std::string>("global_path_topic", "/global_path");
    declare_parameter<std::string>("global_path_3d_topic", "/global_path_3d");
    declare_parameter<std::string>("global_path_marker_topic", "/global_path_marker");

    declare_parameter<std::string>("start_source", "tf");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<bool>("start_z_override_enabled", false);
    declare_parameter<double>("start_z_override", 0.0);
    declare_parameter<std::vector<double>>("manual_start", std::vector<double>{0.0, 0.0, 0.5});

    declare_parameter<int>("max_iterations", 500000);
    declare_parameter<bool>("enable_path_smoothing", true);
    declare_parameter<std::string>("smoothing_method", "line_of_sight");
    declare_parameter<double>("path_resample_resolution", 0.2);
    declare_parameter<double>("path_marker_width", 0.08);
  }

  void loadMapFromParameter()
  {
    const auto map_file = get_parameter("map_file").as_string();
    if (map_file.empty()) {
      RCLCPP_WARN(get_logger(), "map_file is empty. Waiting for OctoMap on %s.",
        get_parameter("octomap_topic").as_string().c_str());
      return;
    }

    std::string error_message;
    const bool ok = map_.load(
      map_file,
      get_parameter("map_resolution").as_double(),
      get_parameter("inflation_radius").as_double(),
      get_parameter("unknown_as_occupied").as_bool(),
      &error_message);
    if (!ok) {
      RCLCPP_ERROR(
        get_logger(), "Failed to load map_file='%s': %s. Waiting for OctoMap topic as fallback.",
        map_file.c_str(), error_message.c_str());
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Loaded OctoMap: %s resolution=%.3f inflation=%.3f unknown_as_occupied=%s "
      "bounds_min=[%.3f, %.3f, %.3f] bounds_max=[%.3f, %.3f, %.3f] inflated_cells=%zu",
      map_file.c_str(), map_.getResolution(), map_.getInflationRadius(),
      map_.unknownAsOccupied() ? "true" : "false",
      map_.mapMin().x(), map_.mapMin().y(), map_.mapMin().z(),
      map_.mapMax().x(), map_.mapMax().y(), map_.mapMax().z(),
      map_.inflatedCellCount());
    publishMapProducts();
  }

  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    if (!get_parameter("accept_octomap_topic").as_bool()) {
      return;
    }

    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::msgToMap(*msg));
    if (!abstract_tree) {
      RCLCPP_ERROR(get_logger(), "Failed to decode incoming OctoMap message.");
      return;
    }

    auto * raw_tree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (!raw_tree) {
      RCLCPP_ERROR(get_logger(), "Incoming OctoMap message is not an octomap::OcTree.");
      return;
    }
    abstract_tree.release();

    std::string error_message;
    const bool ok = map_.setTree(
      std::shared_ptr<octomap::OcTree>(raw_tree),
      get_parameter("inflation_radius").as_double(),
      get_parameter("unknown_as_occupied").as_bool(),
      &error_message);
    if (!ok) {
      RCLCPP_ERROR(get_logger(), "Failed to accept incoming OctoMap: %s", error_message.c_str());
      return;
    }

    if (!msg->header.frame_id.empty() && msg->header.frame_id != get_parameter("map_frame").as_string()) {
      set_parameter(rclcpp::Parameter("map_frame", msg->header.frame_id));
    }
    RCLCPP_INFO(
      get_logger(),
      "Accepted OctoMap from topic. resolution=%.3f bounds_min=[%.3f, %.3f, %.3f] "
      "bounds_max=[%.3f, %.3f, %.3f] inflated_cells=%zu",
      map_.getResolution(),
      map_.mapMin().x(), map_.mapMin().y(), map_.mapMin().z(),
      map_.mapMax().x(), map_.mapMax().y(), map_.mapMax().z(),
      map_.inflatedCellCount());
    publishMapProducts();
  }

  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPoseToMap(*msg, transformed)) {
      return;
    }

    goal_ = toEigen(transformed.pose.position);
    goal_orientation_ = transformed.pose.orientation;
    RCLCPP_INFO(
      get_logger(), "Received 3D goal pose: [%.3f, %.3f, %.3f]",
      goal_->x(), goal_->y(), goal_->z());
    tryPlan();
  }

  void onGoalPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PointStamped transformed;
    if (!transformPointToMap(*msg, transformed)) {
      return;
    }

    goal_ = toEigen(transformed.point);
    goal_orientation_.reset();
    RCLCPP_INFO(
      get_logger(), "Received 3D goal point: [%.3f, %.3f, %.3f]",
      goal_->x(), goal_->y(), goal_->z());
    tryPlan();
  }

  void onInitialPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PoseStamped transformed;
    if (!transformPoseToMap(*msg, transformed)) {
      return;
    }

    topic_start_ = toEigen(transformed.pose.position);
    RCLCPP_INFO(
      get_logger(), "Received 3D initial pose: [%.3f, %.3f, %.3f]",
      topic_start_->x(), topic_start_->y(), topic_start_->z());
    if (goal_.has_value() && get_parameter("start_source").as_string() == "topic") {
      tryPlan();
    }
  }

  bool transformPoseToMap(
    const geometry_msgs::msg::PoseStamped & input,
    geometry_msgs::msg::PoseStamped & output)
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    geometry_msgs::msg::PoseStamped pose = input;
    if (pose.header.frame_id.empty()) {
      pose.header.frame_id = map_frame;
    }
    if (pose.header.frame_id == map_frame) {
      output = pose;
      return true;
    }

    try {
      output = tf_buffer_.transform(pose, map_frame, tf2::durationFromSec(0.2));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to transform pose from %s to %s: %s",
        pose.header.frame_id.c_str(), map_frame.c_str(), ex.what());
      return false;
    }
  }

  bool transformPointToMap(
    const geometry_msgs::msg::PointStamped & input,
    geometry_msgs::msg::PointStamped & output)
  {
    const std::string map_frame = get_parameter("map_frame").as_string();
    geometry_msgs::msg::PointStamped point = input;
    if (point.header.frame_id.empty()) {
      point.header.frame_id = map_frame;
    }
    if (point.header.frame_id == map_frame) {
      output = point;
      return true;
    }

    try {
      output = tf_buffer_.transform(point, map_frame, tf2::durationFromSec(0.2));
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to transform point from %s to %s: %s",
        point.header.frame_id.c_str(), map_frame.c_str(), ex.what());
      return false;
    }
  }

  std::optional<Eigen::Vector3d> resolveStart()
  {
    const auto start_source = get_parameter("start_source").as_string();
    if (start_source == "manual") {
      const auto values = get_parameter("manual_start").as_double_array();
      if (values.size() < 3) {
        RCLCPP_ERROR(get_logger(), "manual_start must contain [x, y, z].");
        return std::nullopt;
      }
      return Eigen::Vector3d(values[0], values[1], values[2]);
    }

    if (start_source == "topic") {
      if (!topic_start_.has_value()) {
        RCLCPP_WARN(get_logger(), "Waiting for /initial_pose_3d because start_source=topic.");
        return std::nullopt;
      }
      return topic_start_;
    }

    if (start_source != "tf") {
      RCLCPP_WARN(
        get_logger(), "Unknown start_source='%s'. Falling back to tf.", start_source.c_str());
    }

    const auto map_frame = get_parameter("map_frame").as_string();
    const auto base_frame = get_parameter("base_frame").as_string();
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.2));
      Eigen::Vector3d start(
        transform.transform.translation.x,
        transform.transform.translation.y,
        transform.transform.translation.z);
      if (get_parameter("start_z_override_enabled").as_bool()) {
        start.z() = get_parameter("start_z_override").as_double();
      }
      return start;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        get_logger(), "Failed to resolve start from TF %s -> %s: %s",
        map_frame.c_str(), base_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void tryPlan()
  {
    if (!map_.isReady()) {
      RCLCPP_WARN(get_logger(), "Cannot plan yet: map not ready.");
      return;
    }
    if (!goal_.has_value()) {
      RCLCPP_WARN(get_logger(), "Cannot plan yet: goal not set.");
      return;
    }

    const auto start = resolveStart();
    if (!start.has_value()) {
      return;
    }

    GlobalPlannerCore::Options options;
    options.max_iterations = get_parameter("max_iterations").as_int();
    options.enable_path_smoothing = get_parameter("enable_path_smoothing").as_bool();
    options.smoothing_method = get_parameter("smoothing_method").as_string();
    options.path_resample_resolution = get_parameter("path_resample_resolution").as_double();

    GlobalPlannerCore planner(map_, options);
    std::vector<Eigen::Vector3d> path;
    if (!planner.plan(*start, *goal_, path)) {
      clearPathOutputs();
      RCLCPP_ERROR(
        get_logger(),
        "3D global planning failed: %s. start=[%.3f, %.3f, %.3f] goal=[%.3f, %.3f, %.3f] "
        "iterations=%d",
        planner.lastFailureReason().c_str(),
        start->x(), start->y(), start->z(),
        goal_->x(), goal_->y(), goal_->z(),
        planner.lastIterations());
      return;
    }

    publishPath(path);
    RCLCPP_INFO(
      get_logger(),
      "3D global path published. points=%zu length=%.3f m iterations=%d start=[%.3f, %.3f, %.3f] "
      "goal=[%.3f, %.3f, %.3f]",
      path.size(), pathLength(path), planner.lastIterations(),
      start->x(), start->y(), start->z(), goal_->x(), goal_->y(), goal_->z());
  }

  void publishPath(const std::vector<Eigen::Vector3d> & path)
  {
    const auto stamp = now();
    const std::string frame_id = get_parameter("map_frame").as_string();

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = frame_id;
    path_msg.poses.reserve(path.size());

    visualization_msgs::msg::Marker marker;
    marker.header = path_msg.header;
    marker.ns = "global_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = get_parameter("path_marker_width").as_double();
    marker.color.r = 0.05F;
    marker.color.g = 0.9F;
    marker.color.b = 1.0F;
    marker.color.a = 1.0F;
    marker.points.reserve(path.size());

    for (std::size_t i = 0; i < path.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position = toPoint(path[i]);
      pose.pose.orientation.w = 1.0;
      if (goal_orientation_.has_value() && i + 1 == path.size()) {
        pose.pose.orientation = *goal_orientation_;
      }
      path_msg.poses.push_back(pose);
      marker.points.push_back(pose.pose.position);
    }

    global_path_pub_->publish(path_msg);
    global_path_3d_pub_->publish(path_msg);
    path_marker_pub_->publish(marker);
  }

  void clearPathOutputs()
  {
    const auto stamp = now();
    const std::string frame_id = get_parameter("map_frame").as_string();
    nav_msgs::msg::Path empty_path;
    empty_path.header.stamp = stamp;
    empty_path.header.frame_id = frame_id;
    global_path_pub_->publish(empty_path);
    global_path_3d_pub_->publish(empty_path);

    visualization_msgs::msg::Marker marker;
    marker.header = empty_path.header;
    marker.ns = "global_path";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    path_marker_pub_->publish(marker);
  }

  void publishMapProducts()
  {
    if (!map_.isReady()) {
      return;
    }
    if (get_parameter("publish_octomap").as_bool()) {
      publishOctomap();
    }
    if (get_parameter("publish_occupied_cloud").as_bool()) {
      publishOccupiedCloud();
    }
    if (get_parameter("publish_occupied_marker").as_bool()) {
      publishOccupiedMarker();
    }
  }

  void publishOctomap()
  {
    auto tree = map_.tree();
    if (!tree) {
      return;
    }

    octomap_msgs::msg::Octomap msg;
    if (!octomap_msgs::binaryMapToMsg(*tree, msg)) {
      RCLCPP_ERROR(get_logger(), "Failed to convert OctoMap to message.");
      return;
    }
    msg.header.stamp = now();
    msg.header.frame_id = get_parameter("map_frame").as_string();
    octomap_pub_->publish(msg);
  }

  void publishOccupiedCloud()
  {
    const auto points = map_.occupiedVoxelCenters();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = get_parameter("map_frame").as_string();

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
    for (const auto & point : points) {
      *iter_x = static_cast<float>(point.x());
      *iter_y = static_cast<float>(point.y());
      *iter_z = static_cast<float>(point.z());
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    occupied_cloud_pub_->publish(cloud);
  }

  void publishOccupiedMarker()
  {
    const auto points = map_.occupiedVoxelCenters();
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = now();
    marker.header.frame_id = get_parameter("map_frame").as_string();
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = map_.getResolution();
    marker.scale.y = map_.getResolution();
    marker.scale.z = map_.getResolution();
    marker.color.r = 0.95F;
    marker.color.g = 0.48F;
    marker.color.b = 0.14F;
    marker.color.a = 0.85F;
    marker.points.reserve(points.size());

    for (const auto & point : points) {
      marker.points.push_back(toPoint(point));
    }
    occupied_marker_pub_->publish(marker);
  }

  OctomapVoxelMap map_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::optional<Eigen::Vector3d> goal_;
  std::optional<Eigen::Vector3d> topic_start_;
  std::optional<geometry_msgs::msg::Quaternion> goal_orientation_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr global_path_3d_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_cloud_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr occupied_marker_pub_;
  rclcpp::TimerBase::SharedPtr publish_map_timer_;
};

}  // namespace global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planner::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
