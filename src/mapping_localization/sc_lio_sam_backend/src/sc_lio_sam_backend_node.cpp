#include "sc_lio_sam_backend/sc_lio_sam_backend_node.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <sstream>

namespace sc_lio_sam_backend
{

namespace
{
std::array<double, 6> vectorToArray6(
  const std::vector<double> & values,
  const std::array<double, 6> & fallback)
{
  if (values.size() != 6) {
    return fallback;
  }

  std::array<double, 6> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}
}  // namespace

ScLioSamBackendNode::ScLioSamBackendNode(const rclcpp::NodeOptions & options)
: Node("sc_lio_sam_backend", options)
{
  declareAndLoadParameters();
  createRosInterfaces();

  RCLCPP_INFO(
    get_logger(),
    "SC-LIO-SAM backend started. odom_topic=%s cloud_topic=%s map_frame=%s odom_frame=%s base_frame=%s",
    odom_topic_.c_str(),
    cloud_topic_.c_str(),
    map_frame_.c_str(),
    odom_frame_.c_str(),
    base_frame_.c_str());
}

ScLioSamBackendNode::~ScLioSamBackendNode()
{
  if (!save_pcd_) {
    return;
  }

  std::vector<Keyframe> snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    snapshot = keyframe_manager_.keyframes();
  }

  std::string message;
  if (saveOptimizedMap(snapshot, &message)) {
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
  } else {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
  }
}

void ScLioSamBackendNode::declareAndLoadParameters()
{
  odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
  cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
  frontend_map_topic_ = declare_parameter<std::string>("frontend_map_topic", "/Laser_map");
  frontend_path_topic_ = declare_parameter<std::string>("frontend_path_topic", "/path");
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/livox/imu");
  map_frame_ = declare_parameter<std::string>("map_frame", "map");
  odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
  base_frame_ = declare_parameter<std::string>("base_frame", "body");
  lidar_frame_ = declare_parameter<std::string>("lidar_frame", "body");
  cloud_frame_mode_ = declare_parameter<std::string>("cloud_frame_mode", "auto");

  use_imu_ = declare_parameter<bool>("use_imu", false);
  use_scan_context_ = declare_parameter<bool>("use_scan_context", true);
  enable_loop_closure_ = declare_parameter<bool>("enable_loop_closure", true);
  enable_pose_graph_optimization_ =
    declare_parameter<bool>("enable_pose_graph_optimization", true);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  publish_map_to_odom_ = declare_parameter<bool>("publish_map_to_odom", true);
  passthrough_frontend_map_ = declare_parameter<bool>("passthrough_frontend_map", true);
  passthrough_frontend_path_when_loop_disabled_ =
    declare_parameter<bool>("passthrough_frontend_path_when_loop_disabled", true);
  save_pcd_ = declare_parameter<bool>("save_pcd", false);
  save_directory_ =
    declare_parameter<std::string>("save_directory", "/home/jhr/3dnav_ws/maps/sc_lio_sam");

  keyframe_distance_threshold_ =
    declare_parameter<double>("keyframe_distance_threshold", 1.0);
  keyframe_angle_threshold_deg_ =
    declare_parameter<double>("keyframe_angle_threshold_deg", 10.0);
  loop_search_radius_ = declare_parameter<double>("loop_search_radius", 15.0);
  loop_search_time_diff_threshold_ =
    declare_parameter<double>("loop_search_time_diff_threshold", 30.0);
  loop_fitness_score_threshold_ =
    declare_parameter<double>("loop_fitness_score_threshold", 0.3);
  loop_keyframe_search_num_ = declare_parameter<int>("loop_keyframe_search_num", 5);
  keyframe_voxel_leaf_size_ = declare_parameter<double>("keyframe_voxel_leaf_size", 0.4);
  icp_voxel_leaf_size_ = declare_parameter<double>("icp_voxel_leaf_size", 0.5);
  map_voxel_leaf_size_ = declare_parameter<double>("map_voxel_leaf_size", 0.3);
  loop_detection_period_sec_ = declare_parameter<double>("loop_detection_period_sec", 1.0);
  map_publish_period_sec_ = declare_parameter<double>("map_publish_period_sec", 3.0);
  max_odom_cloud_time_diff_ = declare_parameter<double>("max_odom_cloud_time_diff", 0.2);
  icp_max_correspondence_distance_ =
    declare_parameter<double>("icp_max_correspondence_distance", 30.0);
  icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 80);
  min_icp_source_points_ = declare_parameter<int>("min_icp_source_points", 100);
  min_icp_target_points_ = declare_parameter<int>("min_icp_target_points", 300);
  max_loop_translation_correction_ =
    declare_parameter<double>("max_loop_translation_correction", 0.0);
  max_loop_rotation_correction_deg_ =
    declare_parameter<double>("max_loop_rotation_correction_deg", 0.0);

  ScanContextConfig scan_context_config;
  scan_context_config.ring_num = declare_parameter<int>("scan_context_ring_num", 20);
  scan_context_config.sector_num = declare_parameter<int>("scan_context_sector_num", 60);
  scan_context_config.max_radius = declare_parameter<double>("scan_context_max_radius", 80.0);
  scan_context_config.lidar_height = declare_parameter<double>("scan_context_lidar_height", 2.0);
  scan_context_config.exclude_recent_num =
    declare_parameter<int>("scan_context_exclude_recent_num", 30);
  scan_context_config.distance_threshold =
    declare_parameter<double>("scan_context_distance_threshold", 0.3);
  scan_context_config.search_ratio =
    declare_parameter<double>("scan_context_search_ratio", 0.1);

  PoseGraphConfig pose_graph_config;
  pose_graph_config.prior_noise = vectorToArray6(
    declare_parameter<std::vector<double>>(
      "prior_noise_sigmas",
      std::vector<double>{1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2}),
    pose_graph_config.prior_noise);
  pose_graph_config.odom_noise = vectorToArray6(
    declare_parameter<std::vector<double>>(
      "odom_noise_sigmas",
      std::vector<double>{1e-2, 1e-2, 1e-2, 5e-2, 5e-2, 5e-2}),
    pose_graph_config.odom_noise);
  pose_graph_config.loop_noise = vectorToArray6(
    declare_parameter<std::vector<double>>(
      "loop_noise_sigmas",
      std::vector<double>{5e-2, 5e-2, 5e-2, 2e-1, 2e-1, 2e-1}),
    pose_graph_config.loop_noise);
  pose_graph_config.use_robust_loop_noise =
    declare_parameter<bool>("use_robust_loop_noise", true);
  pose_graph_config.robust_loop_kernel_size =
    declare_parameter<double>("robust_loop_kernel_size", 1.0);

  keyframe_manager_.configure(keyframe_distance_threshold_, keyframe_angle_threshold_deg_);
  scan_context_manager_.configure(scan_context_config);
  pose_graph_optimizer_.configure(pose_graph_config);

  if (use_imu_) {
    RCLCPP_WARN(
      get_logger(),
      "use_imu is true, but this backend only uses LiDAR odometry and keyframe clouds for loop closure.");
  }
}

void ScLioSamBackendNode::createRosInterfaces()
{
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScLioSamBackendNode::odomCallback, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScLioSamBackendNode::cloudCallback, this, std::placeholders::_1));

  frontend_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    frontend_map_topic_,
    rclcpp::SensorDataQoS(),
    std::bind(&ScLioSamBackendNode::frontendMapCallback, this, std::placeholders::_1));

  frontend_path_sub_ = create_subscription<nav_msgs::msg::Path>(
    frontend_path_topic_,
    rclcpp::QoS(10),
    std::bind(&ScLioSamBackendNode::frontendPathCallback, this, std::placeholders::_1));

  optimized_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/sc_lio_sam_backend/optimized_path",
    rclcpp::QoS(10).reliable());
  raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/sc_lio_sam_backend/raw_keyframe_path",
    rclcpp::QoS(10).reliable());
  optimized_odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
    "/sc_lio_sam_backend/optimized_odometry",
    rclcpp::QoS(20).reliable());
  loop_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/sc_lio_sam_backend/loop_markers",
    rclcpp::QoS(10).reliable());
  optimized_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/sc_lio_sam_backend/optimized_map",
    rclcpp::QoS(1).reliable());

  save_map_service_ = create_service<std_srvs::srv::Trigger>(
    "/sc_lio_sam_backend/save_map",
    std::bind(
      &ScLioSamBackendNode::saveMapCallback,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  loop_timer_ = create_wall_timer(
    std::chrono::duration<double>(std::max(0.1, loop_detection_period_sec_)),
    std::bind(&ScLioSamBackendNode::loopTimerCallback, this));
  map_timer_ = create_wall_timer(
    std::chrono::duration<double>(std::max(0.5, map_publish_period_sec_)),
    std::bind(&ScLioSamBackendNode::mapTimerCallback, this));
}

void ScLioSamBackendNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_odom_ = msg;
}

void ScLioSamBackendNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  nav_msgs::msg::Odometry odom_msg;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!latest_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Waiting for odometry on %s before accepting keyframe clouds.",
        odom_topic_.c_str());
      return;
    }
    odom_msg = *latest_odom_;
  }

  const rclcpp::Time cloud_time(msg->header.stamp);
  const rclcpp::Time odom_time(odom_msg.header.stamp);
  if (max_odom_cloud_time_diff_ > 0.0 &&
    std::abs((cloud_time - odom_time).seconds()) > max_odom_cloud_time_diff_)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      3000,
      "Skipping cloud because odom/cloud time diff %.3fs exceeds %.3fs.",
      std::abs((cloud_time - odom_time).seconds()),
      max_odom_cloud_time_diff_);
    return;
  }

  const Eigen::Isometry3d raw_pose = poseMsgToEigen(odom_msg.pose.pose);
  PointCloudPtr local_cloud = pointCloud2ToPclLocal(*msg, odom_msg, raw_pose);
  if (!local_cloud || local_cloud->empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Skipping empty keyframe cloud.");
    return;
  }
  local_cloud = downsampleCloud(local_cloud, keyframe_voxel_leaf_size_);
  if (local_cloud->empty()) {
    return;
  }

  const ScanContextDescriptor descriptor =
    use_scan_context_ ? scan_context_manager_.makeDescriptor(*local_cloud) : ScanContextDescriptor();

  std::lock_guard<std::mutex> lock(data_mutex_);
  if (!keyframe_manager_.shouldCreateKeyframe(raw_pose)) {
    return;
  }

  addKeyframe(msg->header.stamp, cloud_time.seconds(), raw_pose, local_cloud, descriptor);
  publishRawPathLocked();
  publishOptimizedPathLocked();
  publishOptimizedOdometryLocked();
  publishLoopMarkersLocked();
  publishTfLocked();
}

void ScLioSamBackendNode::frontendMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  bool use_passthrough = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_frontend_map_ = msg;
    use_passthrough = passthrough_frontend_map_ && (!enable_loop_closure_ || loop_edges_.empty());
  }

  if (use_passthrough) {
    optimized_map_pub_->publish(*msg);
  }
}

void ScLioSamBackendNode::frontendPathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_frontend_path_ = msg;
  }

  if (passthrough_frontend_path_when_loop_disabled_ && !enable_loop_closure_) {
    optimized_path_pub_->publish(*msg);
  }
}

void ScLioSamBackendNode::addKeyframe(
  const builtin_interfaces::msg::Time & stamp,
  double stamp_sec,
  const Eigen::Isometry3d & raw_pose,
  const PointCloudPtr & local_cloud,
  const ScanContextDescriptor & descriptor)
{
  const int id =
    keyframe_manager_.addKeyframe(stamp, stamp_sec, raw_pose, local_cloud, descriptor);

  if (enable_pose_graph_optimization_) {
    if (id == 0) {
      pose_graph_optimizer_.addPriorFactor(id, raw_pose);
    } else {
      const auto & previous = keyframe_manager_.keyframes().at(id - 1);
      const Eigen::Isometry3d relative_pose = previous.raw_pose.inverse() * raw_pose;
      pose_graph_optimizer_.addOdometryFactor(id - 1, id, relative_pose, raw_pose);
    }
    optimizeGraphAndUpdateKeyframes();
  }

  RCLCPP_INFO(
    get_logger(),
    "New keyframe %d, total=%zu, points=%zu",
    id,
    keyframe_manager_.size(),
    local_cloud->size());
}

bool ScLioSamBackendNode::optimizeGraphAndUpdateKeyframes()
{
  if (!enable_pose_graph_optimization_) {
    return false;
  }

  const bool updated = pose_graph_optimizer_.update();
  if (!updated) {
    return false;
  }

  for (auto & keyframe : keyframe_manager_.keyframes()) {
    if (pose_graph_optimizer_.hasPose(keyframe.id)) {
      keyframe.optimized_pose = pose_graph_optimizer_.pose(keyframe.id);
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "GTSAM optimization completed with %zu keyframes and %zu loop edges.",
    keyframe_manager_.size(),
    loop_edges_.size());
  return true;
}

void ScLioSamBackendNode::loopTimerCallback()
{
  if (!enable_loop_closure_ || !use_scan_context_ || !enable_pose_graph_optimization_) {
    return;
  }

  std::vector<Keyframe> snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    snapshot = keyframe_manager_.keyframes();
  }

  if (snapshot.size() < 3) {
    return;
  }

  const int current_id = snapshot.back().id;
  const LoopCandidate candidate = scan_context_manager_.detectLoopCandidate(
    snapshot,
    current_id,
    loop_search_radius_,
    loop_search_time_diff_threshold_);
  if (!candidate.valid) {
    return;
  }

  const auto edge_pair = std::make_pair(candidate.candidate_id, candidate.current_id);
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (loop_edge_pairs_.count(edge_pair) > 0) {
      return;
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Scan Context candidate: current=%d previous=%d distance=%.3f yaw=%.2fdeg",
    candidate.current_id,
    candidate.candidate_id,
    candidate.scan_context_distance,
    candidate.yaw_diff_rad * 180.0 / M_PI);

  PointCloudPtr source = buildKeyframeMap(snapshot, candidate.current_id, 0);
  PointCloudPtr target = buildKeyframeMap(snapshot, candidate.candidate_id, loop_keyframe_search_num_);
  source = downsampleCloud(source, icp_voxel_leaf_size_);
  target = downsampleCloud(target, icp_voxel_leaf_size_);

  if (source->size() < static_cast<std::size_t>(min_icp_source_points_) ||
    target->size() < static_cast<std::size_t>(min_icp_target_points_))
  {
    RCLCPP_WARN(
      get_logger(),
      "Reject loop %d -> %d: not enough ICP points source=%zu target=%zu.",
      candidate.candidate_id,
      candidate.current_id,
      source->size(),
      target->size());
    return;
  }

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
  icp.setMaximumIterations(icp_max_iterations_);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);
  icp.setInputSource(source);
  icp.setInputTarget(target);

  PointCloud aligned;
  icp.align(aligned);
  const double fitness = icp.getFitnessScore();
  if (!icp.hasConverged() || fitness > loop_fitness_score_threshold_) {
    RCLCPP_WARN(
      get_logger(),
      "ICP rejected loop %d -> %d: converged=%s fitness=%.4f threshold=%.4f.",
      candidate.candidate_id,
      candidate.current_id,
      icp.hasConverged() ? "true" : "false",
      fitness,
      loop_fitness_score_threshold_);
    return;
  }

  Eigen::Isometry3d correction = Eigen::Isometry3d::Identity();
  correction.matrix() = icp.getFinalTransformation().cast<double>();
  const double correction_translation = correction.translation().norm();
  const double correction_rotation_deg =
    std::abs(Eigen::AngleAxisd(correction.rotation()).angle()) * 180.0 / M_PI;
  if ((max_loop_translation_correction_ > 0.0 &&
      correction_translation > max_loop_translation_correction_) ||
    (max_loop_rotation_correction_deg_ > 0.0 &&
      correction_rotation_deg > max_loop_rotation_correction_deg_))
  {
    RCLCPP_WARN(
      get_logger(),
      "ICP rejected loop %d -> %d: correction %.2fm / %.2fdeg exceeds limits %.2fm / %.2fdeg.",
      candidate.candidate_id,
      candidate.current_id,
      correction_translation,
      correction_rotation_deg,
      max_loop_translation_correction_,
      max_loop_rotation_correction_deg_);
    return;
  }

  const Eigen::Isometry3d corrected_current_pose =
    correction * snapshot.at(candidate.current_id).optimized_pose;
  const Eigen::Isometry3d relative_pose =
    snapshot.at(candidate.candidate_id).optimized_pose.inverse() * corrected_current_pose;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (loop_edge_pairs_.count(edge_pair) > 0) {
      return;
    }
    pose_graph_optimizer_.addLoopFactor(candidate.candidate_id, candidate.current_id, relative_pose);
    loop_edge_pairs_.insert(edge_pair);
    loop_edges_.push_back(LoopEdge{candidate.candidate_id, candidate.current_id, fitness});
    optimizeGraphAndUpdateKeyframes();
    publishRawPathLocked();
    publishOptimizedPathLocked();
    publishOptimizedOdometryLocked();
    publishLoopMarkersLocked();
    publishTfLocked();
  }

  RCLCPP_INFO(
    get_logger(),
    "Added loop factor %d -> %d with ICP fitness %.4f.",
    candidate.candidate_id,
    candidate.current_id,
    fitness);
}

void ScLioSamBackendNode::mapTimerCallback()
{
  sensor_msgs::msg::PointCloud2::SharedPtr frontend_map;
  bool use_passthrough = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    frontend_map = latest_frontend_map_;
    use_passthrough = passthrough_frontend_map_ && (!enable_loop_closure_ || loop_edges_.empty());
  }

  if (use_passthrough) {
    if (frontend_map) {
      optimized_map_pub_->publish(*frontend_map);
    }
    return;
  }

  if (optimized_map_pub_->get_subscription_count() == 0) {
    return;
  }

  std::vector<Keyframe> snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    snapshot = keyframe_manager_.keyframes();
  }

  if (snapshot.empty()) {
    return;
  }

  PointCloudPtr map = buildGlobalMap(snapshot);
  map = downsampleCloud(map, map_voxel_leaf_size_);
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*map, msg);
  msg.header.stamp = now();
  msg.header.frame_id = map_frame_;
  optimized_map_pub_->publish(msg);

  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    10000,
    "Published optimized map with %zu points.",
    map->size());
}

void ScLioSamBackendNode::saveMapCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;

  std::vector<Keyframe> snapshot;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    snapshot = keyframe_manager_.keyframes();
  }

  response->success = saveOptimizedMap(snapshot, &response->message);
}

void ScLioSamBackendNode::publishOptimizedPathLocked()
{
  if (passthrough_frontend_path_when_loop_disabled_ && !enable_loop_closure_ && latest_frontend_path_) {
    optimized_path_pub_->publish(*latest_frontend_path_);
    return;
  }

  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = map_frame_;

  for (const auto & keyframe : keyframe_manager_.keyframes()) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = keyframe.stamp;
    pose.header.frame_id = map_frame_;
    pose.pose = eigenToPoseMsg(keyframe.optimized_pose);
    path.poses.push_back(pose);
  }

  optimized_path_pub_->publish(path);
}

void ScLioSamBackendNode::publishRawPathLocked()
{
  nav_msgs::msg::Path path;
  path.header.stamp = now();
  path.header.frame_id = odom_frame_;

  for (const auto & keyframe : keyframe_manager_.keyframes()) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = keyframe.stamp;
    pose.header.frame_id = odom_frame_;
    pose.pose = eigenToPoseMsg(keyframe.raw_pose);
    path.poses.push_back(pose);
  }

  raw_path_pub_->publish(path);
}

void ScLioSamBackendNode::publishOptimizedOdometryLocked()
{
  if (keyframe_manager_.empty()) {
    return;
  }

  const auto & keyframe = keyframe_manager_.keyframes().back();
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = keyframe.stamp;
  odom.header.frame_id = map_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose = eigenToPoseMsg(keyframe.optimized_pose);
  optimized_odom_pub_->publish(odom);
}

void ScLioSamBackendNode::publishLoopMarkersLocked()
{
  visualization_msgs::msg::MarkerArray marker_array;

  visualization_msgs::msg::Marker clear_marker;
  clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(clear_marker);

  visualization_msgs::msg::Marker nodes;
  nodes.header.stamp = now();
  nodes.header.frame_id = map_frame_;
  nodes.ns = "loop_nodes";
  nodes.id = 0;
  nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  nodes.action = visualization_msgs::msg::Marker::ADD;
  nodes.scale.x = 0.8;
  nodes.scale.y = 0.8;
  nodes.scale.z = 0.8;
  nodes.color.r = 0.0;
  nodes.color.g = 0.8;
  nodes.color.b = 1.0;
  nodes.color.a = 0.9;

  visualization_msgs::msg::Marker edges;
  edges.header = nodes.header;
  edges.ns = "loop_edges";
  edges.id = 1;
  edges.type = visualization_msgs::msg::Marker::LINE_LIST;
  edges.action = visualization_msgs::msg::Marker::ADD;
  edges.scale.x = 0.15;
  edges.color.r = 1.0;
  edges.color.g = 0.5;
  edges.color.b = 0.0;
  edges.color.a = 0.9;

  const auto & keyframes = keyframe_manager_.keyframes();
  for (const auto & loop_edge : loop_edges_) {
    if (loop_edge.from < 0 || loop_edge.to < 0 ||
      loop_edge.from >= static_cast<int>(keyframes.size()) ||
      loop_edge.to >= static_cast<int>(keyframes.size()))
    {
      continue;
    }

    geometry_msgs::msg::Point p_from;
    p_from.x = keyframes.at(loop_edge.from).optimized_pose.translation().x();
    p_from.y = keyframes.at(loop_edge.from).optimized_pose.translation().y();
    p_from.z = keyframes.at(loop_edge.from).optimized_pose.translation().z();
    geometry_msgs::msg::Point p_to;
    p_to.x = keyframes.at(loop_edge.to).optimized_pose.translation().x();
    p_to.y = keyframes.at(loop_edge.to).optimized_pose.translation().y();
    p_to.z = keyframes.at(loop_edge.to).optimized_pose.translation().z();
    nodes.points.push_back(p_from);
    nodes.points.push_back(p_to);
    edges.points.push_back(p_from);
    edges.points.push_back(p_to);
  }

  marker_array.markers.push_back(nodes);
  marker_array.markers.push_back(edges);
  loop_markers_pub_->publish(marker_array);
}

void ScLioSamBackendNode::publishTfLocked()
{
  if (!publish_tf_ || keyframe_manager_.empty()) {
    return;
  }

  const auto & keyframe = keyframe_manager_.keyframes().back();
  if (publish_map_to_odom_) {
    if (normalizedFrame(map_frame_) == normalizedFrame(odom_frame_)) {
      return;
    }
    const Eigen::Isometry3d map_to_odom =
      keyframe.optimized_pose * keyframe.raw_pose.inverse();
    tf_broadcaster_->sendTransform(
      eigenToTransformMsg(map_to_odom, keyframe.stamp, map_frame_, odom_frame_));
    return;
  }

  tf_broadcaster_->sendTransform(
    eigenToTransformMsg(keyframe.optimized_pose, keyframe.stamp, map_frame_, base_frame_));
}

PointCloudPtr ScLioSamBackendNode::pointCloud2ToPclLocal(
  const sensor_msgs::msg::PointCloud2 & msg,
  const nav_msgs::msg::Odometry & odom_msg,
  const Eigen::Isometry3d & raw_pose) const
{
  PointCloudPtr cloud(new PointCloud());
  pcl::fromROSMsg(msg, *cloud);

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  if (cloudIsInOdomFrame(msg.header.frame_id, odom_msg)) {
    return transformCloud(cloud, raw_pose.inverse());
  }

  return cloud;
}

bool ScLioSamBackendNode::cloudIsInOdomFrame(
  const std::string & cloud_frame,
  const nav_msgs::msg::Odometry & odom_msg) const
{
  const std::string mode = cloud_frame_mode_;
  if (mode == "odom" || mode == "world" || mode == "map") {
    return true;
  }
  if (mode == "local" || mode == "body" || mode == "lidar") {
    return false;
  }

  const std::string normalized_cloud = normalizedFrame(cloud_frame);
  if (normalized_cloud.empty()) {
    return false;
  }

  if (normalized_cloud == normalizedFrame(odom_msg.header.frame_id) ||
    normalized_cloud == normalizedFrame(odom_frame_) ||
    normalized_cloud == normalizedFrame(map_frame_))
  {
    return true;
  }

  if (normalized_cloud == normalizedFrame(odom_msg.child_frame_id) ||
    normalized_cloud == normalizedFrame(base_frame_) ||
    normalized_cloud == normalizedFrame(lidar_frame_))
  {
    return false;
  }

  if (!warned_unknown_cloud_frame_) {
    RCLCPP_WARN(
      get_logger(),
      "Cloud frame '%s' does not match odom or body frames. Treating it as a local keyframe cloud.",
      cloud_frame.c_str());
    warned_unknown_cloud_frame_ = true;
  }
  return false;
}

PointCloudPtr ScLioSamBackendNode::downsampleCloud(
  const PointCloudConstPtr & cloud,
  double leaf_size) const
{
  PointCloudPtr output(new PointCloud());
  if (!cloud || cloud->empty()) {
    return output;
  }
  if (leaf_size <= 0.0) {
    *output = *cloud;
    return output;
  }

  pcl::VoxelGrid<PointT> voxel;
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.setInputCloud(cloud);
  voxel.filter(*output);
  return output;
}

PointCloudPtr ScLioSamBackendNode::transformCloud(
  const PointCloudConstPtr & cloud,
  const Eigen::Isometry3d & pose) const
{
  PointCloudPtr output(new PointCloud());
  if (!cloud || cloud->empty()) {
    return output;
  }
  pcl::transformPointCloud(*cloud, *output, pose.matrix().cast<float>());
  return output;
}

PointCloudPtr ScLioSamBackendNode::buildKeyframeMap(
  const std::vector<Keyframe> & keyframes,
  int center_id,
  int search_num) const
{
  PointCloudPtr map(new PointCloud());
  if (keyframes.empty() || center_id < 0 || center_id >= static_cast<int>(keyframes.size())) {
    return map;
  }

  const int first = std::max(0, center_id - search_num);
  const int last = std::min(static_cast<int>(keyframes.size()) - 1, center_id + search_num);
  for (int id = first; id <= last; ++id) {
    const auto & keyframe = keyframes.at(id);
    PointCloudPtr transformed = transformCloud(keyframe.cloud, keyframe.optimized_pose);
    *map += *transformed;
  }
  return map;
}

PointCloudPtr ScLioSamBackendNode::buildGlobalMap(const std::vector<Keyframe> & keyframes) const
{
  PointCloudPtr map(new PointCloud());
  for (const auto & keyframe : keyframes) {
    PointCloudPtr transformed = transformCloud(keyframe.cloud, keyframe.optimized_pose);
    *map += *transformed;
  }
  return map;
}

bool ScLioSamBackendNode::saveOptimizedMap(
  const std::vector<Keyframe> & keyframes,
  std::string * message) const
{
  if (keyframes.empty()) {
    if (message) {
      *message = "No keyframes available; map was not saved.";
    }
    return false;
  }

  std::filesystem::create_directories(save_directory_);
  PointCloudPtr map = buildGlobalMap(keyframes);
  map = downsampleCloud(map, map_voxel_leaf_size_);

  const std::filesystem::path output_path =
    std::filesystem::path(save_directory_) / "optimized_map.pcd";
  const int result = pcl::io::savePCDFileBinary(output_path.string(), *map);
  if (result != 0) {
    if (message) {
      *message = "Failed to save optimized map to " + output_path.string();
    }
    return false;
  }

  if (message) {
    std::ostringstream oss;
    oss << "Saved optimized map with " << map->size() << " points to " << output_path.string();
    *message = oss.str();
  }
  return true;
}

Eigen::Isometry3d ScLioSamBackendNode::poseMsgToEigen(
  const geometry_msgs::msg::Pose & pose_msg)
{
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond q(
    pose_msg.orientation.w,
    pose_msg.orientation.x,
    pose_msg.orientation.y,
    pose_msg.orientation.z);
  if (q.norm() < std::numeric_limits<double>::epsilon()) {
    q = Eigen::Quaterniond::Identity();
  } else {
    q.normalize();
  }
  pose.linear() = q.toRotationMatrix();
  pose.translation() =
    Eigen::Vector3d(pose_msg.position.x, pose_msg.position.y, pose_msg.position.z);
  return pose;
}

geometry_msgs::msg::Pose ScLioSamBackendNode::eigenToPoseMsg(
  const Eigen::Isometry3d & pose)
{
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = pose.translation().x();
  pose_msg.position.y = pose.translation().y();
  pose_msg.position.z = pose.translation().z();

  Eigen::Quaterniond q(pose.rotation());
  q.normalize();
  pose_msg.orientation.x = q.x();
  pose_msg.orientation.y = q.y();
  pose_msg.orientation.z = q.z();
  pose_msg.orientation.w = q.w();
  return pose_msg;
}

geometry_msgs::msg::TransformStamped ScLioSamBackendNode::eigenToTransformMsg(
  const Eigen::Isometry3d & pose,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & parent,
  const std::string & child)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = pose.translation().x();
  transform.transform.translation.y = pose.translation().y();
  transform.transform.translation.z = pose.translation().z();

  Eigen::Quaterniond q(pose.rotation());
  q.normalize();
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  return transform;
}

std::string ScLioSamBackendNode::normalizedFrame(std::string frame)
{
  while (!frame.empty() && frame.front() == '/') {
    frame.erase(frame.begin());
  }
  return frame;
}

}  // namespace sc_lio_sam_backend

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sc_lio_sam_backend::ScLioSamBackendNode>());
  rclcpp::shutdown();
  return 0;
}
