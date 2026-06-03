#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/OcTree.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

namespace global_planner
{

struct GridIndex
{
  int x{};
  int y{};

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct GridIndex3D
{
  int ix{};
  int iy{};
  int iz{};

  bool operator==(const GridIndex3D & other) const
  {
    return ix == other.ix && iy == other.iy && iz == other.iz;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & index) const
  {
    const auto hx = std::hash<int>{}(index.x);
    const auto hy = std::hash<int>{}(index.y);
    return hx ^ (hy << 1U);
  }
};

struct GridIndex3DHash
{
  std::size_t operator()(const GridIndex3D & index) const
  {
    const auto hx = std::hash<int>{}(index.ix);
    const auto hy = std::hash<int>{}(index.iy);
    const auto hz = std::hash<int>{}(index.iz);
    return hx ^ (hy << 1U) ^ (hz << 2U);
  }
};

struct QueueNode
{
  GridIndex index;
  double f{};
  double g{};
};

struct QueueNode3D
{
  GridIndex3D index;
  double f{};
  double g{};
};

struct QueueCompare
{
  bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

struct QueueCompare3D
{
  bool operator()(const QueueNode3D & lhs, const QueueNode3D & rhs) const
  {
    return lhs.f > rhs.f;
  }
};

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_node"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    octomap_file_ = declare_parameter<std::string>("octomap_file", "");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    planning_mode_ = declare_parameter<std::string>("planning_mode", "3d");
    resolution_ = declare_parameter<double>("resolution", 0.0);
    robot_height_min_ = declare_parameter<double>("robot_height_min", 0.1);
    robot_height_max_ = declare_parameter<double>("robot_height_max", 1.0);
    unknown_as_occupied_ = declare_parameter<bool>("unknown_as_occupied", false);
    start_z_override_enabled_ = declare_parameter<bool>("start_z_override_enabled", false);
    start_z_override_ = declare_parameter<double>("start_z_override", 0.0);
    max_iterations_ = declare_parameter<int>("max_iterations", 500000);
    publish_occupied_map_cloud_ =
      declare_parameter<bool>("publish_occupied_map_cloud", true);
    occupied_map_cloud_topic_ =
      declare_parameter<std::string>("occupied_map_cloud_topic", "/global_planner/occupied_map");

    if (planning_mode_ != "2.5d" && planning_mode_ != "3d") {
      RCLCPP_WARN(
        get_logger(),
        "Unsupported planning_mode '%s'; falling back to '3d'",
        planning_mode_.c_str());
      planning_mode_ = "3d";
    }

    path_pub_ = create_publisher<nav_msgs::msg::Path>("/global_path", rclcpp::QoS(1).transient_local());
    path_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "/global_path_markers", rclcpp::QoS(1).transient_local());
    occupied_map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      occupied_map_cloud_topic_, rclcpp::QoS(1).transient_local().reliable());

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose",
      rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onGoal2D, this, std::placeholders::_1));
    goal_3d_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose_3d",
      rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onGoal3D, this, std::placeholders::_1));
    goal_point_3d_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/goal_point_3d",
      rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onGoalPoint3D, this, std::placeholders::_1));

    if (!loadOctomap()) {
      RCLCPP_ERROR(get_logger(), "Global planner started without a valid OctoMap");
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Global planner ready: map='%s' frame=%s base=%s mode=%s resolution=%.3f height=[%.3f, %.3f] unknown_as_occupied=%s",
      octomap_file_.c_str(),
      map_frame_.c_str(),
      base_frame_.c_str(),
      planning_mode_.c_str(),
      resolution_,
      robot_height_min_,
      robot_height_max_,
      unknown_as_occupied_ ? "true" : "false");
  }

private:
  bool loadOctomap()
  {
    if (octomap_file_.empty()) {
      RCLCPP_ERROR(get_logger(), "Parameter 'octomap_file' is empty");
      return false;
    }

    auto tree = std::make_shared<octomap::OcTree>(0.1);
    if (!tree->readBinary(octomap_file_)) {
      RCLCPP_ERROR(get_logger(), "Failed to load OctoMap .bt file: %s", octomap_file_.c_str());
      return false;
    }

    octree_ = tree;
    resolution_ = octree_->getResolution();
    if (resolution_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid OctoMap resolution: %.6f", resolution_);
      return false;
    }
    if (robot_height_max_ < robot_height_min_) {
      RCLCPP_WARN(get_logger(), "robot_height_max is smaller than robot_height_min; swapping them");
      std::swap(robot_height_min_, robot_height_max_);
    }

    octree_->getMetricMin(min_x_, min_y_, min_z_);
    octree_->getMetricMax(max_x_, max_y_, max_z_);
    width_ = static_cast<int>(std::floor((max_x_ - min_x_) / resolution_)) + 1;
    height_ = static_cast<int>(std::floor((max_y_ - min_y_) / resolution_)) + 1;
    depth_ = static_cast<int>(std::floor((max_z_ - min_z_) / resolution_)) + 1;

    RCLCPP_INFO(
      get_logger(),
      "Loaded OctoMap: resolution=%.3f bounds x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f] grid=%dx%dx%d",
      resolution_,
      min_x_,
      max_x_,
      min_y_,
      max_y_,
      min_z_,
      max_z_,
      width_,
      height_,
      depth_);
    publishOccupiedMapCloud();
    return true;
  }

  void publishOccupiedMapCloud()
  {
    if (!publish_occupied_map_cloud_ || !octree_) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = map_frame_;
    cloud_msg.height = 1;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;
    cloud_msg.point_step = 3 * sizeof(float);

    cloud_msg.fields.resize(3);
    const std::vector<std::string> field_names{"x", "y", "z"};
    for (std::size_t i = 0; i < field_names.size(); ++i) {
      cloud_msg.fields[i].name = field_names[i];
      cloud_msg.fields[i].offset = static_cast<std::uint32_t>(i * sizeof(float));
      cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[i].count = 1;
    }

    std::vector<float> points;
    for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
      if (!octree_->isNodeOccupied(*it)) {
        continue;
      }
      points.push_back(static_cast<float>(it.getX()));
      points.push_back(static_cast<float>(it.getY()));
      points.push_back(static_cast<float>(it.getZ()));
    }

    cloud_msg.width = static_cast<std::uint32_t>(points.size() / 3U);
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    cloud_msg.data.resize(points.size() * sizeof(float));
    if (!points.empty()) {
      std::memcpy(cloud_msg.data.data(), points.data(), cloud_msg.data.size());
    }

    occupied_map_cloud_pub_->publish(cloud_msg);
    RCLCPP_INFO(
      get_logger(),
      "Published occupied OctoMap cloud: topic=%s points=%u",
      occupied_map_cloud_topic_.c_str(),
      cloud_msg.width);
  }

  bool isOccupied3D(double x, double y, double z) const
  {
    if (!octree_) {
      return true;
    }

    if (x < min_x_ || x > max_x_ || y < min_y_ || y > max_y_ || z < min_z_ || z > max_z_) {
      return true;
    }

    const octomap::OcTreeNode * node = octree_->search(x, y, z);
    if (node == nullptr) {
      return unknown_as_occupied_;
    }
    return octree_->isNodeOccupied(node);
  }

  bool isFree3D(double x, double y, double z) const
  {
    return !isOccupied3D(x, y, z);
  }

  bool isTraversable2D(double x, double y) const
  {
    if (x < min_x_ || x > max_x_ || y < min_y_ || y > max_y_) {
      return false;
    }

    for (double z = robot_height_min_; z <= robot_height_max_ + 1e-9; z += resolution_) {
      if (isOccupied3D(x, y, z)) {
        return false;
      }
    }
    return true;
  }

  GridIndex worldToGrid(double x, double y) const
  {
    return GridIndex{
      static_cast<int>(std::floor((x - min_x_) / resolution_)),
      static_cast<int>(std::floor((y - min_y_) / resolution_))};
  }

  GridIndex3D worldToGrid(double x, double y, double z) const
  {
    return GridIndex3D{
      static_cast<int>(std::floor((x - min_x_) / resolution_)),
      static_cast<int>(std::floor((y - min_y_) / resolution_)),
      static_cast<int>(std::floor((z - min_z_) / resolution_))};
  }

  void gridToWorld(const GridIndex & index, double & x, double & y) const
  {
    x = min_x_ + (static_cast<double>(index.x) + 0.5) * resolution_;
    y = min_y_ + (static_cast<double>(index.y) + 0.5) * resolution_;
  }

  geometry_msgs::msg::Point gridToWorld(const GridIndex3D & index) const
  {
    geometry_msgs::msg::Point point;
    point.x = min_x_ + (static_cast<double>(index.ix) + 0.5) * resolution_;
    point.y = min_y_ + (static_cast<double>(index.iy) + 0.5) * resolution_;
    point.z = min_z_ + (static_cast<double>(index.iz) + 0.5) * resolution_;
    return point;
  }

  bool isInsideGrid(const GridIndex & index) const
  {
    return index.x >= 0 && index.y >= 0 && index.x < width_ && index.y < height_;
  }

  bool isInsideGrid(const GridIndex3D & index) const
  {
    return index.ix >= 0 && index.iy >= 0 && index.iz >= 0 &&
           index.ix < width_ && index.iy < height_ && index.iz < depth_;
  }

  bool isTraversable2D(const GridIndex & index) const
  {
    if (!isInsideGrid(index)) {
      return false;
    }

    double x{};
    double y{};
    gridToWorld(index, x, y);
    return isTraversable2D(x, y);
  }

  bool isFree3D(const GridIndex3D & index) const
  {
    if (!isInsideGrid(index)) {
      return false;
    }

    const auto point = gridToWorld(index);
    return isFree3D(point.x, point.y, point.z);
  }

  double heuristic(const GridIndex & a, const GridIndex & b) const
  {
    const double dx = static_cast<double>(a.x - b.x) * resolution_;
    const double dy = static_cast<double>(a.y - b.y) * resolution_;
    return std::sqrt(dx * dx + dy * dy);
  }

  double heuristic(const GridIndex3D & a, const GridIndex3D & b) const
  {
    const double dx = static_cast<double>(a.ix - b.ix) * resolution_;
    const double dy = static_cast<double>(a.iy - b.iy) * resolution_;
    const double dz = static_cast<double>(a.iz - b.iz) * resolution_;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const
  {
    std::vector<GridIndex> path;
    path.push_back(current);

    auto parent = came_from.find(current);
    while (parent != came_from.end()) {
      current = parent->second;
      path.push_back(current);
      parent = came_from.find(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
  }

  std::vector<GridIndex3D> reconstructPath3D(
    const std::unordered_map<GridIndex3D, GridIndex3D, GridIndex3DHash> & came_from,
    GridIndex3D current) const
  {
    std::vector<GridIndex3D> path;
    path.push_back(current);

    auto parent = came_from.find(current);
    while (parent != came_from.end()) {
      current = parent->second;
      path.push_back(current);
      parent = came_from.find(current);
    }

    std::reverse(path.begin(), path.end());
    return path;
  }

  bool planAStar2D(
    const GridIndex & start,
    const GridIndex & goal,
    std::vector<GridIndex> & path,
    int & iterations) const
  {
    static const std::vector<GridIndex> neighbors{
      {-1, -1}, {0, -1}, {1, -1},
      {-1, 0},           {1, 0},
      {-1, 1},  {0, 1},  {1, 1}};

    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open_set;
    std::unordered_map<GridIndex, double, GridIndexHash> g_score;
    std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
    std::unordered_set<GridIndex, GridIndexHash> closed_set;

    g_score[start] = 0.0;
    open_set.push(QueueNode{start, heuristic(start, goal), 0.0});
    iterations = 0;

    while (!open_set.empty() && iterations < max_iterations_) {
      const QueueNode current = open_set.top();
      open_set.pop();
      ++iterations;

      if (closed_set.find(current.index) != closed_set.end()) {
        continue;
      }
      closed_set.insert(current.index);

      if (current.index == goal) {
        path = reconstructPath(came_from, current.index);
        return true;
      }

      for (const auto & step : neighbors) {
        const GridIndex next{current.index.x + step.x, current.index.y + step.y};
        if (closed_set.find(next) != closed_set.end() || !isTraversable2D(next)) {
          continue;
        }

        const double step_cost =
          (step.x != 0 && step.y != 0) ? std::sqrt(2.0) * resolution_ : resolution_;
        const double tentative_g = current.g + step_cost;
        const auto score_it = g_score.find(next);
        if (score_it == g_score.end() || tentative_g < score_it->second) {
          came_from[next] = current.index;
          g_score[next] = tentative_g;
          open_set.push(QueueNode{next, tentative_g + heuristic(next, goal), tentative_g});
        }
      }
    }

    return false;
  }

  bool planAStar3D(
    const GridIndex3D & start,
    const GridIndex3D & goal,
    std::vector<GridIndex3D> & path,
    int & iterations) const
  {
    std::priority_queue<QueueNode3D, std::vector<QueueNode3D>, QueueCompare3D> open_set;
    std::unordered_map<GridIndex3D, double, GridIndex3DHash> g_score;
    std::unordered_map<GridIndex3D, GridIndex3D, GridIndex3DHash> came_from;
    std::unordered_set<GridIndex3D, GridIndex3DHash> closed_set;

    g_score[start] = 0.0;
    open_set.push(QueueNode3D{start, heuristic(start, goal), 0.0});
    iterations = 0;

    while (!open_set.empty() && iterations < max_iterations_) {
      const QueueNode3D current = open_set.top();
      open_set.pop();
      ++iterations;

      if (closed_set.find(current.index) != closed_set.end()) {
        continue;
      }
      closed_set.insert(current.index);

      if (current.index == goal) {
        path = reconstructPath3D(came_from, current.index);
        return true;
      }

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) {
              continue;
            }

            const GridIndex3D next{
              current.index.ix + dx,
              current.index.iy + dy,
              current.index.iz + dz};
            if (closed_set.find(next) != closed_set.end() || !isFree3D(next)) {
              continue;
            }

            const double step_cost =
              std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz)) * resolution_;
            const double tentative_g = current.g + step_cost;
            const auto score_it = g_score.find(next);
            if (score_it == g_score.end() || tentative_g < score_it->second) {
              came_from[next] = current.index;
              g_score[next] = tentative_g;
              open_set.push(QueueNode3D{next, tentative_g + heuristic(next, goal), tentative_g});
            }
          }
        }
      }
    }

    return false;
  }

  bool getCurrentRobotPose(double & x, double & y, double & z) const
  {
    try {
      const auto transform = tf_buffer_.lookupTransform(
        map_frame_,
        base_frame_,
        tf2::TimePointZero,
        tf2::durationFromSec(0.5));
      x = transform.transform.translation.x;
      y = transform.transform.translation.y;
      z = start_z_override_enabled_ ? start_z_override_ : transform.transform.translation.z;
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "Failed to lookup TF %s -> %s: %s",
        map_frame_.c_str(), base_frame_.c_str(), ex.what());
      return false;
    }
  }

  void onGoal2D(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received 2D goal from /goal_pose");
    planFromGoal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, "2D goal");
  }

  void onGoal3D(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received 3D goal from /goal_pose_3d");
    if (planning_mode_ == "2.5d") {
      RCLCPP_WARN(
        get_logger(),
        "Current planning_mode is '2.5d'; goal z=%.3f will be ignored. Set planning_mode='3d' for true 3D A*.",
        msg->pose.position.z);
    }
    planFromGoal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z, "3D goal");
  }

  void onGoalPoint3D(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received 3D point goal from /goal_point_3d");
    if (planning_mode_ == "2.5d") {
      RCLCPP_WARN(
        get_logger(),
        "Current planning_mode is '2.5d'; point z=%.3f will be ignored. Set planning_mode='3d' for true 3D A*.",
        msg->point.z);
    }
    planFromGoal(msg->point.x, msg->point.y, msg->point.z, "3D point goal");
  }

  void planFromGoal(double goal_x, double goal_y, double goal_z, const std::string & source)
  {
    if (!octree_) {
      RCLCPP_WARN(get_logger(), "No OctoMap loaded; cannot plan");
      return;
    }

    double start_x{};
    double start_y{};
    double start_z{};
    if (!getCurrentRobotPose(start_x, start_y, start_z)) {
      return;
    }

    if (planning_mode_ == "3d") {
      plan3D(start_x, start_y, start_z, goal_x, goal_y, goal_z, source);
    } else {
      plan2D(start_x, start_y, goal_x, goal_y, source);
    }
  }

  void plan2D(
    double start_x,
    double start_y,
    double goal_x,
    double goal_y,
    const std::string & source)
  {
    const GridIndex start = worldToGrid(start_x, start_y);
    const GridIndex goal = worldToGrid(goal_x, goal_y);

    RCLCPP_INFO(
      get_logger(),
      "Planning 2.5D A*: source=%s start=(%.3f, %.3f) goal=(%.3f, %.3f)",
      source.c_str(),
      start_x,
      start_y,
      goal_x,
      goal_y);

    if (!isTraversable2D(start)) {
      RCLCPP_WARN(get_logger(), "Start is not traversable: world=(%.3f, %.3f) grid=(%d, %d)",
        start_x, start_y, start.x, start.y);
      return;
    }
    if (!isTraversable2D(goal)) {
      RCLCPP_WARN(get_logger(), "Goal is not traversable: world=(%.3f, %.3f) grid=(%d, %d)",
        goal_x, goal_y, goal.x, goal.y);
      return;
    }

    std::vector<GridIndex> path_indices;
    int iterations{};
    if (!planAStar2D(start, goal, path_indices, iterations)) {
      RCLCPP_WARN(
        get_logger(),
        "2.5D A* failed: start_grid=(%d, %d) goal_grid=(%d, %d) iterations=%d max_iterations=%d",
        start.x,
        start.y,
        goal.x,
        goal.y,
        iterations,
        max_iterations_);
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = map_frame_;
    path_msg.poses.reserve(path_indices.size());

    for (const auto & index : path_indices) {
      double x{};
      double y{};
      gridToWorld(index, x, y);

      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = robot_height_min_;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    publishPath(path_msg, iterations);
  }

  void plan3D(
    double start_x,
    double start_y,
    double start_z,
    double goal_x,
    double goal_y,
    double goal_z,
    const std::string & source)
  {
    const GridIndex3D start = worldToGrid(start_x, start_y, start_z);
    const GridIndex3D goal = worldToGrid(goal_x, goal_y, goal_z);

    RCLCPP_INFO(
      get_logger(),
      "Planning 3D A*: source=%s start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f)",
      source.c_str(),
      start_x,
      start_y,
      start_z,
      goal_x,
      goal_y,
      goal_z);

    if (!isFree3D(start)) {
      RCLCPP_WARN(
        get_logger(),
        "Start is occupied or outside map: world=(%.3f, %.3f, %.3f) grid=(%d, %d, %d)",
        start_x,
        start_y,
        start_z,
        start.ix,
        start.iy,
        start.iz);
      return;
    }
    if (!isFree3D(goal)) {
      RCLCPP_WARN(
        get_logger(),
        "Goal is occupied or outside map: world=(%.3f, %.3f, %.3f) grid=(%d, %d, %d)",
        goal_x,
        goal_y,
        goal_z,
        goal.ix,
        goal.iy,
        goal.iz);
      return;
    }

    std::vector<GridIndex3D> path_indices;
    int iterations{};
    if (!planAStar3D(start, goal, path_indices, iterations)) {
      RCLCPP_WARN(
        get_logger(),
        "3D A* failed: start_grid=(%d, %d, %d) goal_grid=(%d, %d, %d) iterations=%d max_iterations=%d",
        start.ix,
        start.iy,
        start.iz,
        goal.ix,
        goal.iy,
        goal.iz,
        iterations,
        max_iterations_);
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = map_frame_;
    path_msg.poses.reserve(path_indices.size());

    for (const auto & index : path_indices) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position = gridToWorld(index);
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    publishPath(path_msg, iterations);
  }

  void publishPath(const nav_msgs::msg::Path & path_msg, int iterations)
  {
    path_pub_->publish(path_msg);
    publishPathMarker(path_msg);
    RCLCPP_INFO(
      get_logger(),
      "Published global path: mode=%s waypoints=%zu iterations=%d",
      planning_mode_.c_str(),
      path_msg.poses.size(),
      iterations);
  }

  void publishPathMarker(const nav_msgs::msg::Path & path_msg)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = path_msg.header;
    marker.header.frame_id = map_frame_;
    marker.ns = "global_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.points.reserve(path_msg.poses.size());

    for (const auto & pose : path_msg.poses) {
      marker.points.push_back(pose.pose.position);
    }

    path_marker_pub_->publish(marker);
  }

  std::string octomap_file_;
  std::string map_frame_{"map"};
  std::string base_frame_{"base_link"};
  std::string planning_mode_{"3d"};
  double resolution_{0.0};
  double robot_height_min_{0.1};
  double robot_height_max_{1.0};
  bool unknown_as_occupied_{false};
  bool start_z_override_enabled_{false};
  double start_z_override_{0.0};
  int max_iterations_{500000};
  bool publish_occupied_map_cloud_{true};
  std::string occupied_map_cloud_topic_{"/global_planner/occupied_map"};

  std::shared_ptr<octomap::OcTree> octree_;
  double min_x_{0.0};
  double min_y_{0.0};
  double min_z_{0.0};
  double max_x_{0.0};
  double max_y_{0.0};
  double max_z_{0.0};
  int width_{0};
  int height_{0};
  int depth_{0};

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_map_cloud_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_3d_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_3d_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

}  // namespace global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planner::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
