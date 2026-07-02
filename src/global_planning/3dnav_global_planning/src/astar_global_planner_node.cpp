#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <queue>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <octomap/OcTree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/color_rgba.hpp"
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
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr double kInf = std::numeric_limits<double>::infinity();

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

std::string lower(const std::string & value)
{
  std::string out = value;
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return out;
}

bool isAbsolutePath(const std::string & path)
{
  return !path.empty() && path.front() == '/';
}

std::filesystem::path findWorkspaceRoot()
{
  const char * env_value = std::getenv("NAV3D_WS");
  if (env_value != nullptr && *env_value != '\0') {
    const std::filesystem::path env_path(env_value);
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

std::string resolveProjectPath(const std::string & raw_path)
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

double distance3d(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return (a - b).norm();
}

double pathLength(const std::vector<Eigen::Vector3d> & points)
{
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    length += distance3d(points[i - 1], points[i]);
  }
  return length;
}

geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d & point)
{
  geometry_msgs::msg::Point msg;
  msg.x = point.x();
  msg.y = point.y();
  msg.z = point.z();
  return msg;
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
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

struct GridKey
{
  int x{0};
  int y{0};
  int z{0};

  bool operator==(const GridKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
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
    auto combine = [&seed](int value) {
      seed ^= std::hash<int>{}(value) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    };
    combine(key.x);
    combine(key.y);
    combine(key.z);
    return seed;
  }
};

struct XyKey
{
  int x{0};
  int y{0};

  bool operator==(const XyKey & other) const
  {
    return x == other.x && y == other.y;
  }
};

struct XyKeyHash
{
  std::size_t operator()(const XyKey & key) const
  {
    std::size_t seed = 0;
    seed ^= std::hash<int>{}(key.x) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    seed ^= std::hash<int>{}(key.y) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct PathMetrics
{
  double length{0.0};
  double min_clearance{kInf};
  double avg_clearance{kInf};
};

struct TraversableSample
{
  Eigen::Vector3d point;
  double cost{0.0};
};

struct TraversableLayer
{
  double z{0.0};
  double cost{0.0};
};

}  // namespace

class AStarPlannerAdapter
{
public:
  bool initialize(const rclcpp::Node::SharedPtr & node)
  {
    node_ = node;
    readParameters();
    clearMap();

    const std::string requested_source = lower(trim(map_source_));
    const bool octomap_exists = !octomap_file_.empty() &&
      std::filesystem::is_regular_file(resolveProjectPath(octomap_file_));
    const bool pcd_exists = !pcd_file_.empty() &&
      std::filesystem::is_regular_file(resolveProjectPath(pcd_file_));

    bool loaded = false;
    if ((requested_source == "octomap" || requested_source.empty()) && octomap_exists) {
      loaded = loadOctomap(resolveProjectPath(octomap_file_));
    }
    if (!loaded && pcd_exists) {
      if (requested_source == "octomap") {
        RCLCPP_WARN(
          node_->get_logger(),
          "OctoMap file is missing or unreadable; building A* grid from PCD: %s",
          resolveProjectPath(pcd_file_).c_str());
      }
      loaded = loadPcd(resolveProjectPath(pcd_file_));
    }

    if (!loaded) {
      last_error_ =
        "No usable map found. Expected octomap_file='" + resolveProjectPath(octomap_file_) +
        "' or pcd_file='" + resolveProjectPath(pcd_file_) + "'.";
      RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
      initialized_ = false;
      return false;
    }

    finalizeMap();
    if (occupied_.empty()) {
      last_error_ = "Loaded map contains no occupied voxels.";
      RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
      initialized_ = false;
      return false;
    }

    initialized_ = true;
    last_error_.clear();
    RCLCPP_INFO(
      node_->get_logger(),
      "A* planner initialized from %s: occupied_voxels=%zu resolution=%.3f mode=%s bounds=[%d,%d]x[%d,%d]x[%d,%d]",
      loaded_map_path_.c_str(), occupied_.size(), resolution_, planning_mode_.c_str(),
      map_min_.x, map_max_.x, map_min_.y, map_max_.y, map_min_.z, map_max_.z);
    return true;
  }

  bool plan(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal,
    std::vector<Eigen::Vector3d> & path)
  {
    path.clear();
    expanded_debug_points_.clear();
    closed_debug_points_.clear();
    last_metrics_ = PathMetrics{};
    last_expanded_nodes_ = 0;
    last_planning_time_sec_ = 0.0;
    last_error_.clear();

    if (!initialized_) {
      last_error_ = "A* planner adapter is not initialized.";
      return false;
    }
    if (!start.allFinite() || !goal.allFinite()) {
      last_error_ = "Start or goal contains non-finite coordinates.";
      return false;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const bool use_3d = planning_mode_ == "3d";
    GridKey start_key = worldToKey(start);
    GridKey goal_key = worldToKey(goal);
    if (!use_3d) {
      start_key.z = 0;
      goal_key.z = 0;
    }
    updateActiveTraversableZRange(start.z(), goal.z(), use_3d);
    updateSearchBounds(start_key, goal_key, use_3d);

    if (!isStateValid(start, start_key, use_3d)) {
      last_error_ = invalidStateReason("START_INVALID", start, start_key, use_3d);
      return false;
    }
    if (!isStateValid(goal, goal_key, use_3d)) {
      last_error_ = invalidStateReason("GOAL_INVALID", goal, goal_key, use_3d);
      return false;
    }
    if (start_key == goal_key) {
      path = {start, goal};
      last_metrics_ = computeMetrics(path);
      return true;
    }

    struct SearchRecord
    {
      double g{kInf};
      double f{kInf};
      GridKey parent;
      bool has_parent{false};
      bool closed{false};
    };
    struct QueueItem
    {
      double f{0.0};
      double g{0.0};
      GridKey key;
      bool operator<(const QueueItem & other) const
      {
        return f > other.f;
      }
    };

    std::priority_queue<QueueItem> open;
    std::unordered_map<GridKey, SearchRecord, GridKeyHash> records;
    records.reserve(4096);

    auto & start_record = records[start_key];
    start_record.g = 0.0;
    start_record.f = heuristicCost(start_key, goal_key, use_3d);
    open.push({start_record.f, start_record.g, start_key});

    bool found = false;
    GridKey final_key = start_key;
    const auto timeout = std::chrono::duration<double>(search_timeout_sec_);

    while (!open.empty()) {
      const auto now = std::chrono::steady_clock::now();
      if (now - t0 > timeout) {
        last_error_ = "A* search timed out after " + std::to_string(search_timeout_sec_) + "s.";
        break;
      }
      if (last_expanded_nodes_ >= max_iterations_) {
        last_error_ = "A* search reached max_iterations=" + std::to_string(max_iterations_) + ".";
        break;
      }

      const QueueItem current_item = open.top();
      open.pop();
      auto current_it = records.find(current_item.key);
      if (current_it == records.end() || current_it->second.closed) {
        continue;
      }
      SearchRecord & current_record = current_it->second;
      if (current_item.g > current_record.g + 1.0e-9) {
        continue;
      }

      current_record.closed = true;
      ++last_expanded_nodes_;
      rememberDebugPoint(current_item.key, use_3d, closed_debug_points_);

      if (current_item.key == goal_key) {
        found = true;
        final_key = current_item.key;
        break;
      }

      for (const GridKey & neighbor : neighbors(current_item.key, use_3d)) {
        if (!withinSearchBounds(neighbor, use_3d)) {
          continue;
        }
        const Eigen::Vector3d neighbor_point = keyToWorld(neighbor, start, goal, use_3d);
        if (!isStateValid(neighbor_point, neighbor, use_3d)) {
          continue;
        }
        const Eigen::Vector3d current_point = keyToWorld(current_item.key, start, goal, use_3d);
        if (!isTransitionValid(current_point, neighbor_point, use_3d)) {
          continue;
        }

        auto & neighbor_record = records[neighbor];
        if (neighbor_record.closed) {
          continue;
        }

        const double step = transitionCost(
          current_item.key,
          neighbor,
          neighbor_point,
          current_record,
          goal_key,
          use_3d);
        if (!std::isfinite(step)) {
          continue;
        }
        const double tentative_g = current_record.g + step;
        if (tentative_g + 1.0e-9 >= neighbor_record.g) {
          continue;
        }

        neighbor_record.g = tentative_g;
        neighbor_record.parent = current_item.key;
        neighbor_record.has_parent = true;
        neighbor_record.f = tentative_g + heuristicCost(neighbor, goal_key, use_3d);
        open.push({neighbor_record.f, neighbor_record.g, neighbor});
        rememberDebugPoint(neighbor, use_3d, expanded_debug_points_);
      }
    }

    last_planning_time_sec_ =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    if (!found) {
      if (last_error_.empty()) {
        std::ostringstream ss;
        ss << "A* search exhausted open set without reaching the goal."
           << " expanded_nodes=" << last_expanded_nodes_
           << " start_clearance=" << queryObstacleDistance(start, use_3d)
           << " goal_clearance=" << queryObstacleDistance(goal, use_3d)
           << " start_hard_collision_distance=" << queryHardCollisionDistance(start, use_3d)
           << " goal_hard_collision_distance=" << queryHardCollisionDistance(goal, use_3d)
           << " hard_collision_clearance=" << hardCollisionClearance()
           << " preferred_required=" << requiredClearance();
        last_error_ = ss.str();
      }
      return false;
    }

    path = reconstructPath(records, start_key, final_key, start, goal, use_3d);
    if (path.empty()) {
      last_error_ = "A* path reconstruction produced an empty path.";
      return false;
    }
    last_metrics_ = computeMetrics(path);
    return true;
  }

  bool isTransitionValid(
    const Eigen::Vector3d & from,
    const Eigen::Vector3d & to,
    bool use_3d) const
  {
    const double length = distance3d(from, to);
    const double sample_resolution = std::max(0.03, resolution_ * 0.5);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / sample_resolution)));
    for (int step = 1; step < steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      const Eigen::Vector3d sample = from + t * (to - from);
      GridKey sample_key = worldToKey(sample);
      if (!use_3d) {
        sample_key.z = 0;
      }
      if (!isStateValid(sample, sample_key, use_3d)) {
        return false;
      }
    }
    return true;
  }

  bool validatePath(const std::vector<Eigen::Vector3d> & path) const
  {
    if (path.empty()) {
      return false;
    }
    const bool use_3d = planning_mode_ == "3d";
    const double sample_resolution = std::max(0.05, resolution_ * 0.5);
    for (std::size_t i = 0; i < path.size(); ++i) {
      GridKey key = worldToKey(path[i]);
      if (!use_3d) {
        key.z = 0;
      }
      if (!isStateValid(path[i], key, use_3d)) {
        return false;
      }
      if (i + 1 >= path.size()) {
        continue;
      }
      const double segment = distance3d(path[i], path[i + 1]);
      const int steps = std::max(1, static_cast<int>(std::ceil(segment / sample_resolution)));
      for (int step = 1; step < steps; ++step) {
        const double t = static_cast<double>(step) / static_cast<double>(steps);
        const Eigen::Vector3d sample = path[i] + t * (path[i + 1] - path[i]);
        GridKey sample_key = worldToKey(sample);
        if (!use_3d) {
          sample_key.z = 0;
        }
        if (!isStateValid(sample, sample_key, use_3d)) {
          return false;
        }
      }
    }
    return true;
  }

  double queryObstacleDistance(const Eigen::Vector3d & p) const
  {
    return queryObstacleDistance(p, planning_mode_ == "3d");
  }

  double queryHardCollisionDistance(const Eigen::Vector3d & p) const
  {
    return queryHardCollisionDistance(p, planning_mode_ == "3d");
  }

  void setTraversableSupport(const std::vector<TraversableSample> & points)
  {
    traversable_xy_.clear();
    traversable_cost_.clear();
    traversable_layers_.clear();
    std::unordered_map<XyKey, double, XyKeyHash> candidate_costs;
    std::unordered_map<XyKey, std::vector<TraversableLayer>, XyKeyHash> candidate_layers;
    candidate_costs.reserve(points.size());
    for (const auto & sample : points) {
      const GridKey key = worldToKey(sample.point);
      const XyKey xy{key.x, key.y};
      const auto it = candidate_costs.find(xy);
      if (it == candidate_costs.end()) {
        candidate_costs.emplace(xy, sample.cost);
      } else {
        it->second = std::min(it->second, sample.cost);
      }
      candidate_layers[xy].push_back({sample.point.z(), sample.cost});
    }

    std::vector<XyKey> dense_cells;
    dense_cells.reserve(candidate_costs.size());
    const int neighbor_radius_cells = std::max(0, static_cast<int>(std::ceil(
      traversable_neighbor_radius_ / std::max(1.0e-6, resolution_))));
    for (const auto & candidate : candidate_costs) {
      const XyKey & cell = candidate.first;
      int neighbors = 0;
      for (int dx = -neighbor_radius_cells; dx <= neighbor_radius_cells; ++dx) {
        for (int dy = -neighbor_radius_cells; dy <= neighbor_radius_cells; ++dy) {
          const double d = resolution_ * std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          if (d > traversable_neighbor_radius_ + 1.0e-9) {
            continue;
          }
          if (candidate_costs.find({cell.x + dx, cell.y + dy}) != candidate_costs.end()) {
            ++neighbors;
          }
        }
      }
      if (neighbors >= traversable_min_neighbors_) {
        dense_cells.push_back(cell);
      }
    }

    const int radius_cells = std::max(0, static_cast<int>(std::ceil(
      traversable_support_radius_ / std::max(1.0e-6, resolution_))));
    for (const auto & center : dense_cells) {
      const auto center_layers_it = candidate_layers.find(center);
      if (center_layers_it == candidate_layers.end()) {
        continue;
      }
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
          const double d = resolution_ * std::hypot(static_cast<double>(dx), static_cast<double>(dy));
          if (d <= traversable_support_radius_ + 1.0e-9) {
            const XyKey support_xy{center.x + dx, center.y + dy};
            traversable_xy_.insert(support_xy);
            auto & support_layers = traversable_layers_[support_xy];
            for (const auto & layer : center_layers_it->second) {
              support_layers.push_back(layer);
              const auto cost_it = traversable_cost_.find(support_xy);
              if (cost_it == traversable_cost_.end()) {
                traversable_cost_.emplace(support_xy, layer.cost);
              } else {
                cost_it->second = std::min(cost_it->second, layer.cost);
              }
            }
          }
        }
      }
    }
    traversable_support_loaded_ = !traversable_xy_.empty();
    RCLCPP_INFO(
      node_->get_logger(),
      "A* traversable support updated from tomogram: source_points=%zu candidate_cells=%zu dense_cells=%zu support_cells=%zu radius=%.3f neighbor_radius=%.3f min_neighbors=%d",
      points.size(), candidate_costs.size(), dense_cells.size(), traversable_xy_.size(),
      traversable_support_radius_, traversable_neighbor_radius_, traversable_min_neighbors_);
  }

  bool needsTraversableSupport() const
  {
    return require_traversable_support_ && planning_mode_ != "3d";
  }

  bool hasTraversableSupport() const
  {
    return traversable_support_loaded_;
  }

  void updateActiveTraversableZRange(double start_z, double goal_z, bool use_3d)
  {
    if (use_3d || !tomogram_z_aware_support_) {
      active_traversable_z_range_enabled_ = false;
      active_traversable_min_z_ = -kInf;
      active_traversable_max_z_ = kInf;
      return;
    }
    const double tolerance = std::max(tomogram_support_z_tolerance_, resolution_);
    active_traversable_z_range_enabled_ = true;
    active_traversable_min_z_ = std::min(start_z, goal_z) - tolerance;
    active_traversable_max_z_ = std::max(start_z, goal_z) + tolerance;
  }

  PathMetrics computeMetrics(const std::vector<Eigen::Vector3d> & path) const
  {
    PathMetrics metrics;
    metrics.length = pathLength(path);
    if (path.empty()) {
      return metrics;
    }

    double total_clearance = 0.0;
    std::size_t count = 0;
    for (const auto & point : path) {
      const double clearance = queryObstacleDistance(point);
      if (std::isfinite(clearance)) {
        metrics.min_clearance = std::min(metrics.min_clearance, clearance);
        total_clearance += clearance;
        ++count;
      }
    }
    if (count > 0) {
      metrics.avg_clearance = total_clearance / static_cast<double>(count);
    }
    return metrics;
  }

  bool initialized() const {return initialized_;}
  const std::string & lastError() const {return last_error_;}
  const PathMetrics & lastMetrics() const {return last_metrics_;}
  std::size_t lastExpandedNodes() const {return last_expanded_nodes_;}
  double lastPlanningTimeSec() const {return last_planning_time_sec_;}
  const std::vector<Eigen::Vector3d> & expandedDebugPoints() const {return expanded_debug_points_;}
  const std::vector<Eigen::Vector3d> & closedDebugPoints() const {return closed_debug_points_;}
  double resolution() const {return resolution_;}
  double hardMinClearance() const {return hard_min_clearance_;}
  double preferredClearance() const {return preferred_clearance_;}

private:
  void readParameters()
  {
    map_frame_ = param<std::string>("map_frame", "map");
    map_source_ = param<std::string>("map_source", "octomap");
    octomap_file_ = param<std::string>("octomap_file", "maps/map_preprocessed.bt");
    pcd_file_ = param<std::string>("pcd_file", "maps/map_preprocessed.pcd");

    resolution_ = std::max(0.02, param<double>("resolution", 0.2));
    planning_mode_ = lower(trim(param<std::string>("planning_mode", "2.5d")));
    if (planning_mode_ != "2.5d" && planning_mode_ != "3d") {
      RCLCPP_WARN(
        node_->get_logger(),
        "Unsupported planning_mode='%s'; using 2.5d.", planning_mode_.c_str());
      planning_mode_ = "2.5d";
    }
    unknown_as_occupied_ = param<bool>("unknown_as_occupied", true);
    robot_radius_ = std::max(0.0, param<double>("robot_radius", 0.35));
    safety_margin_ = std::max(0.0, param<double>("safety_margin", 0.15));
    inflation_radius_ = std::max(0.0, param<double>("inflation_radius", 0.45));
    robot_height_min_ = param<double>("robot_height_min", 0.05);
    robot_height_max_ = std::max(robot_height_min_, param<double>("robot_height_max", 0.70));
    obstacle_min_relative_z_ = param<double>("obstacle_min_relative_z", 0.20);
    obstacle_max_relative_z_ = std::max(
      obstacle_min_relative_z_,
      param<double>("obstacle_max_relative_z", 1.40));
    terrain_following_enabled_ = param<bool>("terrain_following_enabled", true);
    default_path_z_ = param<double>("default_path_z", 0.0);

    heuristic_weight_ = std::max(0.0, param<double>("heuristic_weight", 1.0));
    max_iterations_ = std::max<std::size_t>(1, static_cast<std::size_t>(param<int>("max_iterations", 500000)));
    search_timeout_sec_ = std::max(0.01, param<double>("search_timeout_sec", 5.0));
    neighbor_mode_ = param<int>("neighbor_mode", 26);
    if (neighbor_mode_ != 6 && neighbor_mode_ != 18 && neighbor_mode_ != 26) {
      neighbor_mode_ = 26;
    }
    allow_diagonal_ = param<bool>("allow_diagonal", true);

    clearance_cost_enabled_ = param<bool>("clearance_cost_enabled", true);
    hard_min_clearance_ = std::max(0.0, param<double>("hard_min_clearance", 0.35));
    preferred_clearance_ = std::max(hard_min_clearance_, param<double>("preferred_clearance", 0.75));
    clearance_weight_ = std::max(0.0, param<double>("clearance_weight", 2.0));
    path_length_weight_ = std::max(0.0, param<double>("path_length_weight", 1.0));
    smoothness_weight_ = std::max(0.0, param<double>("smoothness_weight", 1.0));
    search_bounds_padding_ = std::max(0.0, param<double>("search_bounds_padding", 2.0));
    search_bounds_use_full_map_ = param<bool>("search_bounds_use_full_map", false);
    max_debug_marker_points_ = std::max(0, param<int>("max_debug_marker_points", 20000));
    pcd_voxel_min_points_ = std::max(1, param<int>("pcd_voxel_min_points", 1));
    map_distance_max_points_ = std::max(1, param<int>("map_distance_max_points", 250000));
    require_traversable_support_ = param<bool>("require_traversable_support", true);
    enforce_obstacle_clearance_ =
      param<bool>("enforce_obstacle_clearance", !require_traversable_support_);
    hard_collision_clearance_ = std::max(0.0, param<double>("hard_collision_clearance", 0.05));
    hard_collision_min_column_points_ = std::max(
      1, param<int>("hard_collision_min_column_points", 2));
    hard_collision_min_vertical_span_ = std::max(
      0.0, param<double>("hard_collision_min_vertical_span", 0.20));
    wall_clearance_cost_enabled_ = param<bool>("wall_clearance_cost_enabled", true);
    wall_preferred_clearance_ = std::max(
      hard_collision_clearance_, param<double>("wall_preferred_clearance", 0.60));
    wall_clearance_weight_ = std::max(0.0, param<double>("wall_clearance_weight", 10.0));
    wall_clearance_power_ = std::max(0.1, param<double>("wall_clearance_power", 2.0));
    tomogram_traversable_cost_threshold_ = std::max(
      1.0e-3, param<double>("tomogram_traversable_cost_threshold", 50.0));
    tomogram_z_aware_support_ = param<bool>("tomogram_z_aware_support", true);
    tomogram_support_z_tolerance_ = std::max(
      0.0, param<double>("tomogram_support_z_tolerance", 0.50));
    tomogram_cost_enabled_ = param<bool>("tomogram_cost_enabled", true);
    tomogram_cost_normalizer_ = std::max(
      1.0e-3, param<double>("tomogram_cost_normalizer", tomogram_traversable_cost_threshold_));
    tomogram_cost_weight_ = std::max(0.0, param<double>("tomogram_cost_weight", 8.0));
    tomogram_cost_power_ = std::max(0.1, param<double>("tomogram_cost_power", 1.5));
    traversable_support_radius_ = std::max(0.0, param<double>("traversable_support_radius", 0.25));
    traversable_neighbor_radius_ = std::max(0.0, param<double>("traversable_neighbor_radius", 0.30));
    traversable_min_neighbors_ = std::max(1, param<int>("traversable_min_neighbors", 3));
  }

  template<typename T>
  T param(const std::string & name, const T & fallback) const
  {
    try {
      rclcpp::Parameter parameter = node_->get_parameter(name);
      if constexpr (std::is_same<T, bool>::value) {
        return parameter.as_bool();
      } else if constexpr (std::is_same<T, int>::value) {
        return static_cast<int>(parameter.as_int());
      } else if constexpr (std::is_same<T, double>::value) {
        return parameter.as_double();
      } else if constexpr (std::is_same<T, std::string>::value) {
        return parameter.as_string();
      } else {
        return fallback;
      }
    } catch (const std::exception &) {
      return fallback;
    }
  }

  void clearMap()
  {
    occupied_.clear();
    occupied_by_xy_.clear();
    occupied_centers_.clear();
    terrain_min_z_.clear();
    map_min_ = {std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), std::numeric_limits<int>::max()};
    map_max_ = {std::numeric_limits<int>::min(), std::numeric_limits<int>::min(), std::numeric_limits<int>::min()};
    loaded_map_path_.clear();
  }

  bool loadOctomap(const std::string & path)
  {
    auto tree = std::make_unique<octomap::OcTree>(resolution_);
    if (!tree->readBinary(path)) {
      last_error_ = "Failed to read OctoMap binary file: " + path;
      RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
      return false;
    }
    tree->updateInnerOccupancy();

    std::size_t inserted = 0;
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
      if (!tree->isNodeOccupied(*it)) {
        continue;
      }
      addOccupiedPoint(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));
      ++inserted;
    }
    if (inserted == 0) {
      last_error_ = "OctoMap contains no occupied leaves: " + path;
      RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
      return false;
    }
    loaded_map_path_ = path;
    return true;
  }

  bool loadPcd(const std::string & path)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    const int result = pcl::io::loadPCDFile<pcl::PointXYZ>(path, cloud);
    if (result < 0) {
      last_error_ = "Failed to read PCD file: " + path;
      RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
      return false;
    }

    std::unordered_map<GridKey, int, GridKeyHash> counts;
    counts.reserve(cloud.points.size());
    for (const auto & p : cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      ++counts[worldToKey(Eigen::Vector3d(p.x, p.y, p.z))];
    }

    std::size_t inserted = 0;
    for (const auto & entry : counts) {
      if (entry.second < pcd_voxel_min_points_) {
        continue;
      }
      addOccupiedKey(entry.first);
      ++inserted;
      if (inserted >= static_cast<std::size_t>(map_distance_max_points_)) {
        break;
      }
    }

    if (inserted == 0) {
      last_error_ = "PCD contains no usable XYZ points: " + path;
      RCLCPP_ERROR(node_->get_logger(), "%s", last_error_.c_str());
      return false;
    }
    loaded_map_path_ = path;
    return true;
  }

  void addOccupiedPoint(const Eigen::Vector3d & point)
  {
    addOccupiedKey(worldToKey(point));
  }

  void addOccupiedKey(const GridKey & key)
  {
    if (!occupied_.insert(key).second) {
      return;
    }
    map_min_.x = std::min(map_min_.x, key.x);
    map_min_.y = std::min(map_min_.y, key.y);
    map_min_.z = std::min(map_min_.z, key.z);
    map_max_.x = std::max(map_max_.x, key.x);
    map_max_.y = std::max(map_max_.y, key.y);
    map_max_.z = std::max(map_max_.z, key.z);
  }

  void finalizeMap()
  {
    occupied_centers_.reserve(occupied_.size());
    for (const auto & key : occupied_) {
      const Eigen::Vector3d center = keyCenter(key);
      occupied_centers_.push_back(center);
      occupied_by_xy_[{key.x, key.y}].push_back(center.z());
      const XyKey xy{key.x, key.y};
      const auto terrain_it = terrain_min_z_.find(xy);
      if (terrain_it == terrain_min_z_.end()) {
        terrain_min_z_.emplace(xy, center.z());
      } else {
        terrain_it->second = std::min(terrain_it->second, center.z());
      }
    }
  }

  GridKey worldToKey(const Eigen::Vector3d & point) const
  {
    return {
      static_cast<int>(std::floor(point.x() / resolution_)),
      static_cast<int>(std::floor(point.y() / resolution_)),
      static_cast<int>(std::floor(point.z() / resolution_))};
  }

  Eigen::Vector3d keyCenter(const GridKey & key) const
  {
    return Eigen::Vector3d(
      (static_cast<double>(key.x) + 0.5) * resolution_,
      (static_cast<double>(key.y) + 0.5) * resolution_,
      (static_cast<double>(key.z) + 0.5) * resolution_);
  }

  Eigen::Vector3d keyToWorld(
    const GridKey & key,
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal,
    bool use_3d) const
  {
    if (use_3d) {
      return keyCenter(key);
    }

    const double x = (static_cast<double>(key.x) + 0.5) * resolution_;
    const double y = (static_cast<double>(key.y) + 0.5) * resolution_;
    double z = default_path_z_;
    if (terrain_following_enabled_) {
      z = terrainZAt({key.x, key.y}, start.z());
    } else if (std::isfinite(start.z())) {
      const double total = std::max(1.0e-6, (goal.head<2>() - start.head<2>()).norm());
      const double progress = std::clamp((Eigen::Vector2d(x, y) - start.head<2>()).norm() / total, 0.0, 1.0);
      z = start.z() + progress * (goal.z() - start.z());
    }
    if (tomogram_z_aware_support_ && traversable_support_loaded_) {
      z = traversableZAt({key.x, key.y}, z);
    }
    return Eigen::Vector3d(x, y, z);
  }

  double terrainZAt(const XyKey & xy, double fallback) const
  {
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(0.5 / resolution_)));
    double best_distance = kInf;
    double best_z = fallback;
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        const XyKey key{xy.x + dx, xy.y + dy};
        const auto it = terrain_min_z_.find(key);
        if (it == terrain_min_z_.end()) {
          continue;
        }
        const double d = std::hypot(static_cast<double>(dx), static_cast<double>(dy));
        if (d < best_distance) {
          best_distance = d;
          best_z = it->second;
        }
      }
    }
    if (!std::isfinite(best_z)) {
      return default_path_z_;
    }
    return best_z;
  }

  void updateSearchBounds(const GridKey & start, const GridKey & goal, bool use_3d)
  {
    const int pad = static_cast<int>(std::ceil(search_bounds_padding_ / resolution_));
    if (search_bounds_use_full_map_) {
      search_min_ = {std::min({map_min_.x, start.x, goal.x}) - pad,
        std::min({map_min_.y, start.y, goal.y}) - pad,
        std::min({map_min_.z, start.z, goal.z}) - pad};
      search_max_ = {std::max({map_max_.x, start.x, goal.x}) + pad,
        std::max({map_max_.y, start.y, goal.y}) + pad,
        std::max({map_max_.z, start.z, goal.z}) + pad};
    } else {
      search_min_ = {std::min(start.x, goal.x) - pad,
        std::min(start.y, goal.y) - pad,
        std::min(start.z, goal.z) - pad};
      search_max_ = {std::max(start.x, goal.x) + pad,
        std::max(start.y, goal.y) + pad,
        std::max(start.z, goal.z) + pad};
    }
    if (!use_3d) {
      search_min_.z = 0;
      search_max_.z = 0;
    }
  }

  bool withinSearchBounds(const GridKey & key, bool use_3d) const
  {
    if (key.x < search_min_.x || key.x > search_max_.x ||
      key.y < search_min_.y || key.y > search_max_.y)
    {
      return false;
    }
    if (use_3d && (key.z < search_min_.z || key.z > search_max_.z)) {
      return false;
    }
    if (unknown_as_occupied_) {
      const int pad = static_cast<int>(std::ceil(search_bounds_padding_ / resolution_));
      if (key.x < map_min_.x - pad || key.x > map_max_.x + pad ||
        key.y < map_min_.y - pad || key.y > map_max_.y + pad)
      {
        return false;
      }
      if (use_3d && (key.z < map_min_.z - pad || key.z > map_max_.z + pad)) {
        return false;
      }
    }
    return true;
  }

  bool isStateValid(const Eigen::Vector3d & point, const GridKey & key, bool use_3d) const
  {
    if (!withinSearchBounds(key, use_3d)) {
      return false;
    }
    if (!use_3d && require_traversable_support_) {
      if (!traversable_support_loaded_) {
        return false;
      }
      const XyKey xy{key.x, key.y};
      if (!hasTraversableSupportAt(xy)) {
        return false;
      }
    }
    if (enforce_obstacle_clearance_) {
      const double clearance = queryHardCollisionDistance(point, use_3d);
      const double required = hardCollisionClearance();
      if (clearance < required) {
        return false;
      }
    }
    return true;
  }

  std::string invalidStateReason(
    const std::string & label,
    const Eigen::Vector3d & point,
    const GridKey & key,
    bool use_3d) const
  {
    const double clearance = queryObstacleDistance(point, use_3d);
    const double hard_collision_distance = queryHardCollisionDistance(point, use_3d);
    const XyKey xy{key.x, key.y};
    std::ostringstream ss;
    ss << label
       << ": point=[" << point.x() << ", " << point.y() << ", " << point.z() << "]"
       << " key=[" << key.x << ", " << key.y << ", " << key.z << "]"
       << " within_search=" << (withinSearchBounds(key, use_3d) ? "true" : "false")
       << " traversable_support=" << (traversable_support_loaded_ ? "loaded" : "missing")
       << " in_traversable_domain="
       << ((!use_3d && hasTraversableSupportAt(xy)) ? "true" : "false")
       << " active_traversable_z_range=["
       << active_traversable_min_z_ << ", " << active_traversable_max_z_ << "]"
       << " obstacle_clearance_enforced=" << (enforce_obstacle_clearance_ ? "true" : "false")
       << " hard_collision_clearance=" << hardCollisionClearance()
       << " hard_collision_distance=" << hard_collision_distance
       << " clearance=" << clearance
       << " preferred_required=" << requiredClearance();
    if (!use_3d) {
      ss << " obstacle_z_window=["
         << point.z() + obstacle_min_relative_z_ << ", "
         << point.z() + obstacle_max_relative_z_ << "]";
    }
    ss << " map_bounds=["
       << map_min_.x << "," << map_max_.x << "]x["
       << map_min_.y << "," << map_max_.y << "]x["
       << map_min_.z << "," << map_max_.z << "]";
    return ss.str();
  }

  double requiredClearance() const
  {
    const double configured_inflation = inflation_radius_ > 0.0 ?
      inflation_radius_ : robot_radius_ + safety_margin_;
    return std::max(hard_min_clearance_, configured_inflation);
  }

  double hardCollisionClearance() const
  {
    return enforce_obstacle_clearance_ ? hard_collision_clearance_ : 0.0;
  }

  bool hasTraversableSupportAt(const XyKey & xy) const
  {
    if (!tomogram_z_aware_support_) {
      return traversable_xy_.find(xy) != traversable_xy_.end();
    }
    return traversableCostAt(xy).has_value();
  }

  std::optional<double> traversableCostAt(const XyKey & xy) const
  {
    if (!tomogram_z_aware_support_) {
      const auto it = traversable_cost_.find(xy);
      if (it == traversable_cost_.end()) {
        return std::nullopt;
      }
      return it->second;
    }

    const auto layers_it = traversable_layers_.find(xy);
    if (layers_it == traversable_layers_.end()) {
      return std::nullopt;
    }

    double best_cost = kInf;
    for (const auto & layer : layers_it->second) {
      if (active_traversable_z_range_enabled_ &&
        (layer.z < active_traversable_min_z_ || layer.z > active_traversable_max_z_))
      {
        continue;
      }
      best_cost = std::min(best_cost, layer.cost);
    }
    if (!std::isfinite(best_cost)) {
      return std::nullopt;
    }
    return best_cost;
  }

  double traversableZAt(const XyKey & xy, double fallback) const
  {
    if (!tomogram_z_aware_support_) {
      return fallback;
    }
    const auto layers_it = traversable_layers_.find(xy);
    if (layers_it == traversable_layers_.end()) {
      return fallback;
    }

    double best_z = fallback;
    double best_distance = kInf;
    for (const auto & layer : layers_it->second) {
      if (active_traversable_z_range_enabled_ &&
        (layer.z < active_traversable_min_z_ || layer.z > active_traversable_max_z_))
      {
        continue;
      }
      const double distance = std::abs(layer.z - fallback);
      if (distance < best_distance) {
        best_distance = distance;
        best_z = layer.z;
      }
    }
    return best_z;
  }

  double queryObstacleDistance(const Eigen::Vector3d & p, bool use_3d) const
  {
    if (occupied_.empty()) {
      return kInf;
    }
    const GridKey center = worldToKey(p);
    const double max_radius = std::max({preferred_clearance_, requiredClearance(), resolution_}) + resolution_;
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(max_radius / resolution_)));
    double best = kInf;

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        const XyKey xy{center.x + dx, center.y + dy};
        const auto it = occupied_by_xy_.find(xy);
        if (it == occupied_by_xy_.end()) {
          continue;
        }
        const double ox = (static_cast<double>(xy.x) + 0.5) * resolution_;
        const double oy = (static_cast<double>(xy.y) + 0.5) * resolution_;
        const double dxy = std::hypot(ox - p.x(), oy - p.y());
        if (dxy > max_radius + resolution_) {
          continue;
        }
        for (const double oz : it->second) {
          if (use_3d) {
            const double d = std::sqrt(dxy * dxy + (oz - p.z()) * (oz - p.z()));
            best = std::min(best, d);
          } else {
            if (oz < p.z() + obstacle_min_relative_z_ || oz > p.z() + obstacle_max_relative_z_) {
              continue;
            }
            best = std::min(best, dxy);
          }
        }
      }
    }
    if (!std::isfinite(best)) {
      return max_radius;
    }
    return best;
  }

  double queryHardCollisionDistance(const Eigen::Vector3d & p, bool use_3d) const
  {
    if (occupied_.empty()) {
      return kInf;
    }

    const GridKey center = worldToKey(p);
    if (!use_3d && isHardObstacleColumn({center.x, center.y}, p.z())) {
      return 0.0;
    }
    if (use_3d && occupied_.find(center) != occupied_.end()) {
      return 0.0;
    }

    const double max_radius = std::max({hardCollisionClearance(), wall_preferred_clearance_, resolution_}) +
      resolution_;
    const int radius_cells = std::max(1, static_cast<int>(std::ceil(max_radius / resolution_)));
    double best = kInf;

    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        const XyKey xy{center.x + dx, center.y + dy};
        const auto it = occupied_by_xy_.find(xy);
        if (it == occupied_by_xy_.end()) {
          continue;
        }
        const double ox = (static_cast<double>(xy.x) + 0.5) * resolution_;
        const double oy = (static_cast<double>(xy.y) + 0.5) * resolution_;
        const double dxy = std::hypot(ox - p.x(), oy - p.y());
        if (dxy > max_radius + resolution_) {
          continue;
        }
        if (use_3d) {
          for (const double oz : it->second) {
            const double d = std::sqrt(dxy * dxy + (oz - p.z()) * (oz - p.z()));
            best = std::min(best, d);
          }
        } else if (isHardObstacleColumn(xy, p.z())) {
          best = std::min(best, dxy);
        }
      }
    }

    if (!std::isfinite(best)) {
      return max_radius;
    }
    return best;
  }

  bool isHardObstacleColumn(const XyKey & xy, double path_z) const
  {
    const auto it = occupied_by_xy_.find(xy);
    if (it == occupied_by_xy_.end()) {
      return false;
    }

    int count = 0;
    double min_z = kInf;
    double max_z = -kInf;
    const double lower = path_z + obstacle_min_relative_z_;
    const double upper = path_z + obstacle_max_relative_z_;
    for (const double oz : it->second) {
      if (oz < lower || oz > upper) {
        continue;
      }
      ++count;
      min_z = std::min(min_z, oz);
      max_z = std::max(max_z, oz);
    }

    if (count >= hard_collision_min_column_points_) {
      return true;
    }
    return count > 0 && hard_collision_min_vertical_span_ > 0.0 &&
      (max_z - min_z) >= hard_collision_min_vertical_span_;
  }

  std::vector<GridKey> neighbors(const GridKey & key, bool use_3d) const
  {
    std::vector<GridKey> out;
    if (!use_3d) {
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) {
            continue;
          }
          if (!allow_diagonal_ && std::abs(dx) + std::abs(dy) > 1) {
            continue;
          }
          out.push_back({key.x + dx, key.y + dy, 0});
        }
      }
      return out;
    }

    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }
          const int axes = std::abs(dx) + std::abs(dy) + std::abs(dz);
          if (neighbor_mode_ == 6 && axes != 1) {
            continue;
          }
          if (neighbor_mode_ == 18 && axes > 2) {
            continue;
          }
          out.push_back({key.x + dx, key.y + dy, key.z + dz});
        }
      }
    }
    return out;
  }

  double heuristicCost(const GridKey & key, const GridKey & goal, bool use_3d) const
  {
    const double dx = static_cast<double>(key.x - goal.x);
    const double dy = static_cast<double>(key.y - goal.y);
    const double dz = use_3d ? static_cast<double>(key.z - goal.z) : 0.0;
    return heuristic_weight_ * path_length_weight_ * resolution_ * std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  template<typename Record>
  double transitionCost(
    const GridKey & current,
    const GridKey & neighbor,
    const Eigen::Vector3d & neighbor_point,
    const Record & current_record,
    const GridKey & goal,
    bool use_3d) const
  {
    (void)goal;
    const double dx = static_cast<double>(neighbor.x - current.x);
    const double dy = static_cast<double>(neighbor.y - current.y);
    const double dz = use_3d ? static_cast<double>(neighbor.z - current.z) : 0.0;
    const double step_length = resolution_ * std::sqrt(dx * dx + dy * dy + dz * dz);

    const double clearance_cost = clearanceCost(neighbor_point, use_3d);
    if (!std::isfinite(clearance_cost)) {
      return kInf;
    }
    const double wall_cost = wallClearanceCost(neighbor_point, use_3d);
    if (!std::isfinite(wall_cost)) {
      return kInf;
    }
    const double tomogram_cost = tomogramCost(neighbor, use_3d);
    if (!std::isfinite(tomogram_cost)) {
      return kInf;
    }

    double smoothness_cost = 0.0;
    if (current_record.has_parent) {
      const Eigen::Vector3d previous_dir(
        static_cast<double>(current.x - current_record.parent.x),
        static_cast<double>(current.y - current_record.parent.y),
        use_3d ? static_cast<double>(current.z - current_record.parent.z) : 0.0);
      const Eigen::Vector3d next_dir(dx, dy, dz);
      const double denom = previous_dir.norm() * next_dir.norm();
      if (denom > 1.0e-9) {
        const double cosine = std::clamp(previous_dir.dot(next_dir) / denom, -1.0, 1.0);
        smoothness_cost = smoothness_weight_ * (1.0 - cosine);
      }
    }

    return path_length_weight_ * step_length + clearance_cost + wall_cost + tomogram_cost +
      smoothness_cost;
  }

  double clearanceCost(const Eigen::Vector3d & point, bool use_3d) const
  {
    if (!clearance_cost_enabled_) {
      return 0.0;
    }
    const double clearance = queryObstacleDistance(point, use_3d);
    const double hard_collision_distance = queryHardCollisionDistance(point, use_3d);
    if (enforce_obstacle_clearance_ && hard_collision_distance < hardCollisionClearance()) {
      return kInf;
    }
    if (clearance >= preferred_clearance_) {
      return 0.0;
    }
    const double span = std::max(1.0e-3, preferred_clearance_ - hardCollisionClearance());
    const double ratio = std::clamp((preferred_clearance_ - clearance) / span, 0.0, 1.0);
    return clearance_weight_ * ratio * ratio;
  }

  double wallClearanceCost(const Eigen::Vector3d & point, bool use_3d) const
  {
    if (!wall_clearance_cost_enabled_) {
      return 0.0;
    }
    const double hard_distance = queryHardCollisionDistance(point, use_3d);
    if (enforce_obstacle_clearance_ && hard_distance < hardCollisionClearance()) {
      return kInf;
    }
    if (hard_distance >= wall_preferred_clearance_) {
      return 0.0;
    }
    const double span = std::max(1.0e-3, wall_preferred_clearance_ - hardCollisionClearance());
    const double ratio = std::clamp((wall_preferred_clearance_ - hard_distance) / span, 0.0, 1.0);
    return wall_clearance_weight_ * std::pow(ratio, wall_clearance_power_);
  }

  double tomogramCost(const GridKey & key, bool use_3d) const
  {
    if (use_3d || !tomogram_cost_enabled_) {
      return 0.0;
    }
    const auto cost = traversableCostAt({key.x, key.y});
    if (!cost.has_value()) {
      return require_traversable_support_ ? kInf : 0.0;
    }
    const double normalizer = std::max(1.0e-3, tomogram_cost_normalizer_);
    const double ratio = std::clamp(*cost / normalizer, 0.0, 1.0);
    return tomogram_cost_weight_ * std::pow(ratio, tomogram_cost_power_);
  }

  template<typename Records>
  std::vector<Eigen::Vector3d> reconstructPath(
    const Records & records,
    const GridKey & start_key,
    GridKey final_key,
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal,
    bool use_3d) const
  {
    std::vector<GridKey> keys;
    GridKey current = final_key;
    keys.push_back(current);
    while (current != start_key) {
      const auto it = records.find(current);
      if (it == records.end() || !it->second.has_parent) {
        return {};
      }
      current = it->second.parent;
      keys.push_back(current);
    }
    std::reverse(keys.begin(), keys.end());

    std::vector<Eigen::Vector3d> points;
    points.reserve(keys.size());
    for (const auto & key : keys) {
      points.push_back(keyToWorld(key, start, goal, use_3d));
    }
    if (!points.empty()) {
      points.front() = start;
      points.back() = goal;
    }
    return points;
  }

  void rememberDebugPoint(
    const GridKey & key,
    bool use_3d,
    std::vector<Eigen::Vector3d> & points)
  {
    if (max_debug_marker_points_ <= 0 ||
      points.size() >= static_cast<std::size_t>(max_debug_marker_points_))
    {
      return;
    }
    Eigen::Vector3d point = keyCenter(key);
    if (!use_3d) {
      point.z() = terrainZAt({key.x, key.y}, default_path_z_) + 0.05;
    }
    points.push_back(point);
  }

  rclcpp::Node::SharedPtr node_;
  bool initialized_{false};
  std::string last_error_;
  std::string loaded_map_path_;
  PathMetrics last_metrics_;
  std::size_t last_expanded_nodes_{0};
  double last_planning_time_sec_{0.0};

  std::string map_frame_{"map"};
  std::string map_source_{"octomap"};
  std::string octomap_file_{"maps/map_preprocessed.bt"};
  std::string pcd_file_{"maps/map_preprocessed.pcd"};
  double resolution_{0.2};
  std::string planning_mode_{"2.5d"};
  bool unknown_as_occupied_{true};
  double robot_radius_{0.35};
  double safety_margin_{0.15};
  double inflation_radius_{0.45};
  double robot_height_min_{0.05};
  double robot_height_max_{0.70};
  double obstacle_min_relative_z_{0.20};
  double obstacle_max_relative_z_{1.40};
  bool terrain_following_enabled_{true};
  double default_path_z_{0.0};
  double heuristic_weight_{1.0};
  std::size_t max_iterations_{500000};
  double search_timeout_sec_{5.0};
  int neighbor_mode_{26};
  bool allow_diagonal_{true};
  bool clearance_cost_enabled_{true};
  double hard_min_clearance_{0.35};
  double preferred_clearance_{0.75};
  double clearance_weight_{2.0};
  double path_length_weight_{1.0};
  double smoothness_weight_{1.0};
  double search_bounds_padding_{2.0};
  bool search_bounds_use_full_map_{false};
  int max_debug_marker_points_{20000};
  int pcd_voxel_min_points_{1};
  int map_distance_max_points_{250000};
  bool require_traversable_support_{true};
  bool enforce_obstacle_clearance_{false};
  double hard_collision_clearance_{0.05};
  int hard_collision_min_column_points_{2};
  double hard_collision_min_vertical_span_{0.20};
  bool wall_clearance_cost_enabled_{true};
  double wall_preferred_clearance_{0.60};
  double wall_clearance_weight_{10.0};
  double wall_clearance_power_{2.0};
  double tomogram_traversable_cost_threshold_{50.0};
  bool tomogram_z_aware_support_{true};
  double tomogram_support_z_tolerance_{0.50};
  bool tomogram_cost_enabled_{true};
  double tomogram_cost_normalizer_{20.0};
  double tomogram_cost_weight_{8.0};
  double tomogram_cost_power_{1.5};
  double traversable_support_radius_{0.25};
  double traversable_neighbor_radius_{0.30};
  int traversable_min_neighbors_{3};

  std::unordered_set<GridKey, GridKeyHash> occupied_;
  std::unordered_map<XyKey, std::vector<double>, XyKeyHash> occupied_by_xy_;
  std::vector<Eigen::Vector3d> occupied_centers_;
  std::unordered_map<XyKey, double, XyKeyHash> terrain_min_z_;
  GridKey map_min_;
  GridKey map_max_;
  GridKey search_min_;
  GridKey search_max_;
  std::vector<Eigen::Vector3d> expanded_debug_points_;
  std::vector<Eigen::Vector3d> closed_debug_points_;
  std::unordered_set<XyKey, XyKeyHash> traversable_xy_;
  std::unordered_map<XyKey, double, XyKeyHash> traversable_cost_;
  std::unordered_map<XyKey, std::vector<TraversableLayer>, XyKeyHash> traversable_layers_;
  bool traversable_support_loaded_{false};
  bool active_traversable_z_range_enabled_{false};
  double active_traversable_min_z_{-kInf};
  double active_traversable_max_z_{kInf};
};

class AStarGlobalPlannerNode : public rclcpp::Node
{
public:
  AStarGlobalPlannerNode()
  : Node("astar_global_planner_node"),
    tf_buffer_(std::make_unique<tf2_ros::Buffer>(get_clock())),
    tf_listener_(std::make_unique<tf2_ros::TransformListener>(*tf_buffer_))
  {
    declareParameters();
    readParameters();
    createRosInterfaces();
  }

  void start()
  {
    setStatus("WAITING_FOR_MAP");
    if (adapter_.initialize(shared_from_this())) {
      setStatus("WAITING_FOR_GOAL");
    } else {
      setStatus("MAP_ERROR");
    }
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("start_source", "tf");
    declare_parameter<std::string>("start_pose_topic", "/astar_global_planner/start_pose");
    declare_parameter<std::string>("odom_topic", "/odom");
    declare_parameter<double>("tf_lookup_timeout", 0.25);
    declare_parameter<bool>("start_z_override_enabled", false);
    declare_parameter<double>("start_z_override", 0.0);

    declare_parameter<std::string>("goal_pose_topic", "/goal_pose_3d");
    declare_parameter<std::string>("goal_point_topic", "/goal_point_3d");
    declare_parameter<std::string>("rviz_2d_goal_topic", "/goal_pose");
    declare_parameter<double>("default_goal_z", 0.0);

    declare_parameter<std::string>("map_source", "octomap");
    declare_parameter<std::string>("octomap_file", "maps/map_preprocessed.bt");
    declare_parameter<std::string>("pcd_file", "maps/map_preprocessed.pcd");
    declare_parameter<std::string>("tomogram_topic", "/tomogram");
    declare_parameter<double>("resolution", 0.20);
    declare_parameter<std::string>("planning_mode", "2.5d");
    declare_parameter<bool>("unknown_as_occupied", true);
    declare_parameter<double>("robot_radius", 0.35);
    declare_parameter<double>("safety_margin", 0.15);
    declare_parameter<double>("inflation_radius", 0.45);
    declare_parameter<double>("robot_height_min", 0.05);
    declare_parameter<double>("robot_height_max", 0.70);
    declare_parameter<double>("obstacle_min_relative_z", 0.20);
    declare_parameter<double>("obstacle_max_relative_z", 1.40);
    declare_parameter<bool>("terrain_following_enabled", true);
    declare_parameter<double>("default_path_z", 0.0);
    declare_parameter<double>("heuristic_weight", 1.0);
    declare_parameter<int>("max_iterations", 500000);
    declare_parameter<double>("search_timeout_sec", 5.0);
    declare_parameter<int>("neighbor_mode", 26);
    declare_parameter<bool>("allow_diagonal", true);
    declare_parameter<double>("search_bounds_padding", 2.0);
    declare_parameter<bool>("search_bounds_use_full_map", false);
    declare_parameter<int>("pcd_voxel_min_points", 1);
    declare_parameter<int>("map_distance_max_points", 250000);
    declare_parameter<bool>("require_traversable_support", true);
    declare_parameter<bool>("enforce_obstacle_clearance", true);
    declare_parameter<double>("hard_collision_clearance", 0.05);
    declare_parameter<int>("hard_collision_min_column_points", 2);
    declare_parameter<double>("hard_collision_min_vertical_span", 0.20);
    declare_parameter<double>("tomogram_traversable_cost_threshold", 50.0);
    declare_parameter<bool>("tomogram_z_aware_support", true);
    declare_parameter<bool>("wall_clearance_cost_enabled", true);
    declare_parameter<double>("wall_preferred_clearance", 0.60);
    declare_parameter<double>("wall_clearance_weight", 10.0);
    declare_parameter<double>("wall_clearance_power", 2.0);
    declare_parameter<bool>("tomogram_cost_enabled", true);
    declare_parameter<double>("tomogram_cost_normalizer", 20.0);
    declare_parameter<double>("tomogram_cost_weight", 8.0);
    declare_parameter<double>("tomogram_cost_power", 1.5);
    declare_parameter<bool>("tomogram_filter_by_z", true);
    declare_parameter<double>("tomogram_support_z_tolerance", 0.50);
    declare_parameter<double>("traversable_support_radius", 0.25);
    declare_parameter<double>("traversable_neighbor_radius", 0.30);
    declare_parameter<int>("traversable_min_neighbors", 3);

    declare_parameter<bool>("clearance_cost_enabled", true);
    declare_parameter<double>("hard_min_clearance", 0.35);
    declare_parameter<double>("preferred_clearance", 0.75);
    declare_parameter<double>("clearance_weight", 2.0);
    declare_parameter<double>("path_length_weight", 1.0);
    declare_parameter<double>("smoothness_weight", 1.0);

    declare_parameter<bool>("postprocess_enabled", true);
    declare_parameter<bool>("enable_path_resampling", true);
    declare_parameter<double>("path_resample_resolution", 0.2);
    declare_parameter<bool>("enable_z_smoothing", true);
    declare_parameter<int>("z_smoothing_iterations", 30);
    declare_parameter<double>("z_smoothing_alpha", 0.45);
    declare_parameter<double>("z_max_step", 0.12);
    declare_parameter<double>("z_max_slope", 0.80);
    declare_parameter<bool>("enable_path_smoothing", true);
    declare_parameter<int>("smoothing_iterations", 20);
    declare_parameter<bool>("remove_duplicate_points", true);
    declare_parameter<bool>("validate_after_smoothing", true);

    declare_parameter<std::string>("publish_path_topic", "/planned_path");
    declare_parameter<std::string>("publish_alias_path_topic", "/path");
    declare_parameter<std::string>("publish_marker_topic", "/planned_path_marker");
    declare_parameter<std::string>("status_topic", "/astar_global_planner/status");
    declare_parameter<std::string>("debug_topic", "/astar_global_planner/debug");
    declare_parameter<std::string>("expanded_nodes_marker_topic", "/astar_expanded_nodes_marker");
    declare_parameter<std::string>("closed_set_marker_topic", "/astar_closed_set_marker");
    declare_parameter<std::string>("clearance_marker_topic", "/astar_clearance_marker");
    declare_parameter<bool>("publish_debug_markers", true);
    declare_parameter<int>("max_debug_marker_points", 20000);
    declare_parameter<double>("marker_line_width", 0.08);
  }

  void readParameters()
  {
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    start_source_ = lower(trim(get_parameter("start_source").as_string()));
    if (start_source_.empty()) {
      start_source_ = "tf";
    }
    start_pose_topic_ = get_parameter("start_pose_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    tf_lookup_timeout_ = std::max(0.01, get_parameter("tf_lookup_timeout").as_double());
    start_z_override_enabled_ = get_parameter("start_z_override_enabled").as_bool();
    start_z_override_ = get_parameter("start_z_override").as_double();

    goal_pose_topic_ = get_parameter("goal_pose_topic").as_string();
    goal_point_topic_ = get_parameter("goal_point_topic").as_string();
    rviz_2d_goal_topic_ = get_parameter("rviz_2d_goal_topic").as_string();
    default_goal_z_ = get_parameter("default_goal_z").as_double();
    default_path_z_ = get_parameter("default_path_z").as_double();
    tomogram_topic_ = get_parameter("tomogram_topic").as_string();
    tomogram_traversable_cost_threshold_ =
      get_parameter("tomogram_traversable_cost_threshold").as_double();
    tomogram_z_aware_support_ = get_parameter("tomogram_z_aware_support").as_bool();
    tomogram_filter_by_z_ = get_parameter("tomogram_filter_by_z").as_bool();
    tomogram_support_z_tolerance_ =
      std::max(0.0, get_parameter("tomogram_support_z_tolerance").as_double());

    postprocess_enabled_ = get_parameter("postprocess_enabled").as_bool();
    enable_path_resampling_ = get_parameter("enable_path_resampling").as_bool();
    path_resample_resolution_ = std::max(0.02, get_parameter("path_resample_resolution").as_double());
    enable_z_smoothing_ = get_parameter("enable_z_smoothing").as_bool();
    z_smoothing_iterations_ = std::max(0, static_cast<int>(get_parameter("z_smoothing_iterations").as_int()));
    z_smoothing_alpha_ = std::clamp(get_parameter("z_smoothing_alpha").as_double(), 0.0, 1.0);
    z_max_step_ = std::max(0.0, get_parameter("z_max_step").as_double());
    z_max_slope_ = std::max(0.0, get_parameter("z_max_slope").as_double());
    enable_path_smoothing_ = get_parameter("enable_path_smoothing").as_bool();
    smoothing_iterations_ = std::max(0, static_cast<int>(get_parameter("smoothing_iterations").as_int()));
    remove_duplicate_points_ = get_parameter("remove_duplicate_points").as_bool();
    validate_after_smoothing_ = get_parameter("validate_after_smoothing").as_bool();

    publish_path_topic_ = get_parameter("publish_path_topic").as_string();
    publish_alias_path_topic_ = get_parameter("publish_alias_path_topic").as_string();
    publish_marker_topic_ = get_parameter("publish_marker_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    debug_topic_ = get_parameter("debug_topic").as_string();
    expanded_nodes_marker_topic_ = get_parameter("expanded_nodes_marker_topic").as_string();
    closed_set_marker_topic_ = get_parameter("closed_set_marker_topic").as_string();
    clearance_marker_topic_ = get_parameter("clearance_marker_topic").as_string();
    publish_debug_markers_ = get_parameter("publish_debug_markers").as_bool();
    marker_line_width_ = get_parameter("marker_line_width").as_double();
  }

  void createRosInterfaces()
  {
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, latchedQos());
    debug_pub_ = create_publisher<std_msgs::msg::String>(debug_topic_, latchedQos());
    path_pub_ = create_publisher<nav_msgs::msg::Path>(publish_path_topic_, latchedQos());
    if (!publish_alias_path_topic_.empty() && publish_alias_path_topic_ != publish_path_topic_) {
      alias_path_pub_ = create_publisher<nav_msgs::msg::Path>(publish_alias_path_topic_, latchedQos());
    }
    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(publish_marker_topic_, latchedQos());
    expanded_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      expanded_nodes_marker_topic_, latchedQos());
    closed_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      closed_set_marker_topic_, latchedQos());
    clearance_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      clearance_marker_topic_, latchedQos());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::QoS(20),
      std::bind(&AStarGlobalPlannerNode::onOdom, this, std::placeholders::_1));
    start_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      start_pose_topic_, latchedQos(),
      std::bind(&AStarGlobalPlannerNode::onStartPose, this, std::placeholders::_1));
    if (!tomogram_topic_.empty()) {
      tomogram_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        tomogram_topic_, latchedQos(),
        std::bind(&AStarGlobalPlannerNode::onTomogram, this, std::placeholders::_1));
    }

    if (!goal_pose_topic_.empty()) {
      goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_pose_topic_, latchedQos(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          onGoalPose(msg, false, "goal_pose_topic");
        });
    }
    if (!rviz_2d_goal_topic_.empty() && rviz_2d_goal_topic_ != goal_pose_topic_) {
      rviz_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        rviz_2d_goal_topic_, latchedQos(),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          onGoalPose(msg, true, "rviz_2d_goal_topic");
        });
    }
    if (!goal_point_topic_.empty()) {
      goal_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        goal_point_topic_, latchedQos(),
        std::bind(&AStarGlobalPlannerNode::onGoalPoint, this, std::placeholders::_1));
    }

    status_timer_ = create_wall_timer(1s, [this]() {publishStatus();});
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odom_ = *msg;
  }

  void onStartPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    last_start_pose_ = *msg;
  }

  void onTomogram(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::vector<TraversableSample> support_points;
    support_points.reserve(static_cast<std::size_t>(msg->width) * static_cast<std::size_t>(msg->height));
    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
      sensor_msgs::PointCloud2ConstIterator<float> iter_intensity(*msg, "intensity");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        const float cost = *iter_intensity;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !std::isfinite(cost)) {
          continue;
        }
        if (!tomogram_z_aware_support_ && tomogram_filter_by_z_ &&
          std::abs(static_cast<double>(z) - default_path_z_) > tomogram_support_z_tolerance_)
        {
          continue;
        }
        if (cost <= tomogram_traversable_cost_threshold_) {
          support_points.push_back(
            {
              Eigen::Vector3d(static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)),
              static_cast<double>(cost)});
        }
      }
    } catch (const std::exception & exc) {
      RCLCPP_WARN(get_logger(), "Cannot parse tomogram support cloud on %s: %s", tomogram_topic_.c_str(), exc.what());
      return;
    }

    if (support_points.empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Tomogram support cloud on %s has no usable points with intensity <= %.3f%s.",
        tomogram_topic_.c_str(), tomogram_traversable_cost_threshold_,
        tomogram_filter_by_z_ ? " near default_path_z" : "");
      return;
    }
    adapter_.setTraversableSupport(support_points);
    if (status_ == "WAITING_FOR_MAP") {
      setStatus("WAITING_FOR_GOAL");
    }
    if (pending_goal_.has_value()) {
      const Eigen::Vector3d goal = *pending_goal_;
      pending_goal_.reset();
      RCLCPP_INFO(
        get_logger(),
        "Planning previously received A* goal after tomogram support became available.");
      planToGoal(goal);
    }
  }

  void onGoalPose(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg,
    bool force_default_z,
    const std::string & source)
  {
    Eigen::Vector3d goal(
      msg->pose.position.x,
      msg->pose.position.y,
      force_default_z ? default_goal_z_ : msg->pose.position.z);
    if (!transformToMap(goal, msg->header.frame_id, goal)) {
      setStatus("TF_ERROR");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Received A* goal from %s: [%.3f, %.3f, %.3f]",
      source.c_str(), goal.x(), goal.y(), goal.z());
    planToGoal(goal);
  }

  void onGoalPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    Eigen::Vector3d goal(msg->point.x, msg->point.y, msg->point.z);
    if (!transformToMap(goal, msg->header.frame_id, goal)) {
      setStatus("TF_ERROR");
      return;
    }
    RCLCPP_INFO(
      get_logger(), "Received A* goal point: [%.3f, %.3f, %.3f]",
      goal.x(), goal.y(), goal.z());
    planToGoal(goal);
  }

  bool planToGoal(const Eigen::Vector3d & goal)
  {
    if (planning_in_progress_) {
      RCLCPP_WARN(get_logger(), "A* plan request ignored because a plan is already running.");
      return false;
    }
    planning_in_progress_ = true;
    const auto guard = std::shared_ptr<void>(nullptr, [this](void *) {planning_in_progress_ = false;});

    if (!adapter_.initialized()) {
      RCLCPP_ERROR(get_logger(), "%s", adapter_.lastError().c_str());
      setStatus("MAP_ERROR");
      return false;
    }
    if (adapter_.needsTraversableSupport() && !adapter_.hasTraversableSupport()) {
      RCLCPP_WARN(
        get_logger(),
        "A* is waiting for tomogram support on %s before planning. This prevents paths through unmapped free space.",
        tomogram_topic_.c_str());
      pending_goal_ = goal;
      setStatus("WAITING_FOR_MAP");
      return false;
    }

    Eigen::Vector3d start;
    if (!getCurrentStart(start)) {
      setStatus(start_source_ == "tf" ? "TF_ERROR" : "START_INVALID");
      return false;
    }

    setStatus("PLANNING");
    RCLCPP_INFO(
      get_logger(),
      "Planning A* global path: start=[%.3f, %.3f, %.3f] goal=[%.3f, %.3f, %.3f]",
      start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());

    std::vector<Eigen::Vector3d> raw_path;
    if (!adapter_.plan(start, goal, raw_path)) {
      RCLCPP_ERROR(get_logger(), "A* planning failed: %s", adapter_.lastError().c_str());
      publishDebug(start, goal, {}, adapter_.lastMetrics(), adapter_.lastError());
      publishDebugMarkers({});
      setStatus(statusFromFailure(adapter_.lastError()));
      return false;
    }

    std::vector<Eigen::Vector3d> path = postprocessPath(raw_path);
    if (path.empty()) {
      RCLCPP_ERROR(get_logger(), "A* postprocessing produced an empty path.");
      setStatus("FAILED");
      return false;
    }

    const PathMetrics metrics = adapter_.computeMetrics(path);
    publishPath(path);
    publishDebug(start, goal, path, metrics, "");
    publishDebugMarkers(path);
    pending_goal_.reset();
    RCLCPP_INFO(
      get_logger(),
      "A* global path published: points=%zu length=%.3fm min_clearance=%.3fm avg_clearance=%.3fm expanded=%zu planning_time=%.1fms",
      path.size(), metrics.length, metrics.min_clearance, metrics.avg_clearance,
      adapter_.lastExpandedNodes(), adapter_.lastPlanningTimeSec() * 1000.0);
    setStatus("SUCCESS");
    return true;
  }

  bool getCurrentStart(Eigen::Vector3d & start)
  {
    if (start_source_ == "tf") {
      try {
        const auto transform = tf_buffer_->lookupTransform(
          map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(tf_lookup_timeout_));
        start = Eigen::Vector3d(
          transform.transform.translation.x,
          transform.transform.translation.y,
          transform.transform.translation.z);
        applyStartZOverride(start);
        return true;
      } catch (const std::exception & exc) {
        RCLCPP_WARN(
          get_logger(), "Cannot query TF %s -> %s for A* start: %s",
          map_frame_.c_str(), base_frame_.c_str(), exc.what());
        return false;
      }
    }

    if (start_source_ == "odom") {
      if (!last_odom_.has_value()) {
        RCLCPP_WARN(get_logger(), "No odometry received on %s; cannot plan.", odom_topic_.c_str());
        return false;
      }
      Eigen::Vector3d odom_point(
        last_odom_->pose.pose.position.x,
        last_odom_->pose.pose.position.y,
        last_odom_->pose.pose.position.z);
      if (!transformToMap(odom_point, last_odom_->header.frame_id, start)) {
        return false;
      }
      applyStartZOverride(start);
      return true;
    }

    if (start_source_ == "topic" || start_source_ == "manual") {
      if (!last_start_pose_.has_value()) {
        RCLCPP_WARN(
          get_logger(), "No start pose received on %s; cannot plan.", start_pose_topic_.c_str());
        return false;
      }
      Eigen::Vector3d pose_point(
        last_start_pose_->pose.position.x,
        last_start_pose_->pose.position.y,
        last_start_pose_->pose.position.z);
      if (!transformToMap(pose_point, last_start_pose_->header.frame_id, start)) {
        return false;
      }
      applyStartZOverride(start);
      return true;
    }

    RCLCPP_ERROR(get_logger(), "Unsupported start_source='%s'. Use tf, odom, topic, or manual.", start_source_.c_str());
    return false;
  }

  void applyStartZOverride(Eigen::Vector3d & start) const
  {
    if (start_z_override_enabled_) {
      start.z() = start_z_override_;
    }
  }

  bool transformToMap(
    const Eigen::Vector3d & input,
    const std::string & frame_id,
    Eigen::Vector3d & output)
  {
    const std::string source_frame = frame_id.empty() ? map_frame_ : frame_id;
    if (source_frame == map_frame_) {
      output = input;
      return true;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        map_frame_, source_frame, tf2::TimePointZero, tf2::durationFromSec(tf_lookup_timeout_));
      output = transformPoint(transform, input);
      return true;
    } catch (const std::exception & exc) {
      RCLCPP_WARN(
        get_logger(), "Cannot transform point from '%s' to '%s': %s",
        source_frame.c_str(), map_frame_.c_str(), exc.what());
      return false;
    }
  }

  std::vector<Eigen::Vector3d> postprocessPath(const std::vector<Eigen::Vector3d> & raw_path)
  {
    if (!postprocess_enabled_) {
      return raw_path;
    }

    std::vector<Eigen::Vector3d> basic = raw_path;
    if (remove_duplicate_points_) {
      basic = removeDuplicates(basic);
    }
    if (enable_path_resampling_) {
      basic = resamplePath(basic, path_resample_resolution_);
    }
    if (enable_z_smoothing_) {
      const std::vector<Eigen::Vector3d> z_smoothed = smoothZProfile(basic);
      if (!validate_after_smoothing_ || adapter_.validatePath(z_smoothed)) {
        const double before_step = maxZStep(basic);
        const double after_step = maxZStep(z_smoothed);
        if (after_step + 1.0e-6 < before_step) {
          RCLCPP_INFO(
            get_logger(),
            "A* z profile smoothed: max_dz %.3fm -> %.3fm",
            before_step, after_step);
        }
        basic = z_smoothed;
      } else {
        RCLCPP_WARN(
          get_logger(),
          "A* z-smoothed path failed validation; keeping unsmoothed z profile.");
      }
    }
    if (validate_after_smoothing_ && !adapter_.validatePath(basic)) {
      RCLCPP_WARN(
        get_logger(),
        "A* resampled path failed collision/domain validation; using raw path only if it is valid.");
      return adapter_.validatePath(raw_path) ? raw_path : std::vector<Eigen::Vector3d>{};
    }

    std::vector<Eigen::Vector3d> candidate = basic;
    if (enable_path_smoothing_ && smoothing_iterations_ > 0) {
      candidate = smoothPath(candidate, smoothing_iterations_);
      if (enable_path_resampling_) {
        candidate = resamplePath(candidate, path_resample_resolution_);
      }
    }

    if (!validate_after_smoothing_) {
      return candidate;
    }

    const PathMetrics before = adapter_.computeMetrics(basic);
    const PathMetrics after = adapter_.computeMetrics(candidate);
    if (!adapter_.validatePath(candidate) || after.min_clearance + 1.0e-6 < before.min_clearance) {
      RCLCPP_WARN(
        get_logger(),
        "A* smoothed path failed validation or reduced clearance (before %.3f, after %.3f); using unsmoothed path.",
        before.min_clearance, after.min_clearance);
      return basic;
    }
    return candidate;
  }

  static std::vector<Eigen::Vector3d> removeDuplicates(const std::vector<Eigen::Vector3d> & points)
  {
    if (points.empty()) {
      return {};
    }
    std::vector<Eigen::Vector3d> out;
    out.reserve(points.size());
    out.push_back(points.front());
    for (std::size_t i = 1; i < points.size(); ++i) {
      if (distance3d(out.back(), points[i]) > 1.0e-4) {
        out.push_back(points[i]);
      }
    }
    return out;
  }

  static std::vector<Eigen::Vector3d> resamplePath(
    const std::vector<Eigen::Vector3d> & points,
    double resolution)
  {
    if (points.size() < 2 || resolution <= 0.0) {
      return points;
    }
    std::vector<Eigen::Vector3d> out;
    out.push_back(points.front());
    Eigen::Vector3d current = points.front();
    double remaining = resolution;

    for (std::size_t i = 1; i < points.size(); ++i) {
      Eigen::Vector3d target = points[i];
      double segment = distance3d(current, target);
      if (segment <= 1.0e-9) {
        current = target;
        continue;
      }
      while (segment >= remaining) {
        const double ratio = remaining / segment;
        const Eigen::Vector3d next = current + ratio * (target - current);
        out.push_back(next);
        current = next;
        segment = distance3d(current, target);
        remaining = resolution;
      }
      remaining -= segment;
      current = target;
    }

    if (distance3d(out.back(), points.back()) > 1.0e-6) {
      out.push_back(points.back());
    }
    return out;
  }

  std::vector<Eigen::Vector3d> smoothZProfile(const std::vector<Eigen::Vector3d> & points) const
  {
    if (points.size() < 3 || z_smoothing_iterations_ <= 0) {
      return points;
    }

    std::vector<Eigen::Vector3d> smoothed = points;
    for (int iter = 0; iter < z_smoothing_iterations_; ++iter) {
      std::vector<Eigen::Vector3d> next = smoothed;
      for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
        const double target = 0.5 * (smoothed[i - 1].z() + smoothed[i + 1].z());
        next[i].z() = (1.0 - z_smoothing_alpha_) * smoothed[i].z() +
          z_smoothing_alpha_ * target;
      }
      next.front().z() = points.front().z();
      next.back().z() = points.back().z();
      enforceZRateLimits(next, points);
      smoothed = std::move(next);
    }
    enforceZRateLimits(smoothed, points);
    return smoothed;
  }

  void enforceZRateLimits(
    std::vector<Eigen::Vector3d> & points,
    const std::vector<Eigen::Vector3d> & reference) const
  {
    if (points.size() < 2) {
      return;
    }

    points.front().z() = reference.front().z();
    points.back().z() = reference.back().z();

    for (std::size_t i = 1; i + 1 < points.size(); ++i) {
      const double allowed = allowedZStep(points[i - 1], points[i]);
      points[i].z() = std::clamp(points[i].z(), points[i - 1].z() - allowed, points[i - 1].z() + allowed);
    }
    for (std::size_t reverse_i = points.size() - 2; reverse_i > 0; --reverse_i) {
      const double allowed = allowedZStep(points[reverse_i], points[reverse_i + 1]);
      points[reverse_i].z() = std::clamp(
        points[reverse_i].z(),
        points[reverse_i + 1].z() - allowed,
        points[reverse_i + 1].z() + allowed);
    }
  }

  double allowedZStep(const Eigen::Vector3d & a, const Eigen::Vector3d & b) const
  {
    double allowed = z_max_step_ > 0.0 ? z_max_step_ : kInf;
    if (z_max_slope_ > 0.0) {
      const double dxy = (b.head<2>() - a.head<2>()).norm();
      if (dxy > 1.0e-6) {
        allowed = std::min(allowed, std::max(0.02, z_max_slope_ * dxy));
      }
    }
    if (!std::isfinite(allowed)) {
      return kInf;
    }
    return std::max(0.02, allowed);
  }

  static double maxZStep(const std::vector<Eigen::Vector3d> & points)
  {
    double max_step = 0.0;
    for (std::size_t i = 1; i < points.size(); ++i) {
      max_step = std::max(max_step, std::abs(points[i].z() - points[i - 1].z()));
    }
    return max_step;
  }

  static std::vector<Eigen::Vector3d> smoothPath(
    const std::vector<Eigen::Vector3d> & points,
    int iterations)
  {
    if (points.size() < 3 || iterations <= 0) {
      return points;
    }
    std::vector<Eigen::Vector3d> smoothed = points;
    for (int iter = 0; iter < iterations; ++iter) {
      std::vector<Eigen::Vector3d> next;
      next.reserve(smoothed.size());
      next.push_back(smoothed.front());
      for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
        next.push_back(0.25 * smoothed[i - 1] + 0.50 * smoothed[i] + 0.25 * smoothed[i + 1]);
      }
      next.push_back(smoothed.back());
      smoothed = std::move(next);
    }
    return smoothed;
  }

  void publishPath(const std::vector<Eigen::Vector3d> & points)
  {
    const builtin_interfaces::msg::Time stamp = now();
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = map_frame_;
    path_msg.poses.reserve(points.size());
    for (std::size_t i = 0; i < points.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position = toPointMsg(points[i]);
      double yaw = 0.0;
      if (i + 1 < points.size()) {
        yaw = std::atan2(points[i + 1].y() - points[i].y(), points[i + 1].x() - points[i].x());
      } else if (i > 0) {
        yaw = std::atan2(points[i].y() - points[i - 1].y(), points[i].x() - points[i - 1].x());
      }
      pose.pose.orientation = yawToQuaternion(yaw);
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    if (alias_path_pub_) {
      alias_path_pub_->publish(path_msg);
    }

    visualization_msgs::msg::Marker marker;
    marker.header = path_msg.header;
    marker.ns = "astar_global_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = marker_line_width_;
    marker.color.r = 0.0;
    marker.color.g = 0.9;
    marker.color.b = 0.25;
    marker.color.a = 1.0;
    for (const auto & point : points) {
      marker.points.push_back(toPointMsg(point));
    }
    marker_pub_->publish(marker);
  }

  void publishDebug(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal,
    const std::vector<Eigen::Vector3d> & path,
    const PathMetrics & metrics,
    const std::string & failure_reason)
  {
    std_msgs::msg::String msg;
    std::ostringstream ss;
    ss << "start=[" << start.x() << "," << start.y() << "," << start.z() << "]";
    ss << " goal=[" << goal.x() << "," << goal.y() << "," << goal.z() << "]";
    ss << " planning_mode=" << get_parameter("planning_mode").as_string();
    ss << " expanded_nodes=" << adapter_.lastExpandedNodes();
    ss << " path_points=" << path.size();
    ss << " path_length=" << metrics.length;
    ss << " min_clearance=" << metrics.min_clearance;
    ss << " avg_clearance=" << metrics.avg_clearance;
    ss << " planning_time=" << adapter_.lastPlanningTimeSec();
    if (!failure_reason.empty()) {
      ss << " failure_reason=\"" << failure_reason << "\"";
    }
    msg.data = ss.str();
    debug_pub_->publish(msg);
  }

  void publishDebugMarkers(const std::vector<Eigen::Vector3d> & path)
  {
    if (!publish_debug_markers_) {
      return;
    }
    const builtin_interfaces::msg::Time stamp = now();
    expanded_marker_pub_->publish(makePointMarker(
      adapter_.expandedDebugPoints(), stamp, "astar_expanded_nodes", 0, 0.03,
      makeColor(0.1F, 0.55F, 1.0F, 0.35F)));
    closed_marker_pub_->publish(makePointMarker(
      adapter_.closedDebugPoints(), stamp, "astar_closed_set", 0, 0.035,
      makeColor(1.0F, 0.7F, 0.1F, 0.35F)));
    clearance_marker_pub_->publish(makeClearanceMarker(path, stamp));
  }

  visualization_msgs::msg::Marker makePointMarker(
    const std::vector<Eigen::Vector3d> & points,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & ns,
    int id,
    double scale,
    const std_msgs::msg::ColorRGBA & color) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.color = color;
    marker.points.reserve(points.size());
    for (const auto & point : points) {
      marker.points.push_back(toPointMsg(point));
    }
    return marker;
  }

  visualization_msgs::msg::MarkerArray makeClearanceMarker(
    const std::vector<Eigen::Vector3d> & path,
    const builtin_interfaces::msg::Time & stamp) const
  {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = "astar_clearance";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.16;
    marker.scale.y = 0.16;
    marker.scale.z = 0.16;
    for (const auto & point : path) {
      const double clearance = adapter_.queryObstacleDistance(point);
      std_msgs::msg::ColorRGBA color;
      color.a = 0.8F;
      if (clearance < adapter_.hardMinClearance()) {
        color.r = 1.0F;
        color.g = 0.05F;
        color.b = 0.05F;
      } else if (clearance < adapter_.preferredClearance()) {
        color.r = 1.0F;
        color.g = 0.75F;
        color.b = 0.05F;
      } else {
        color.r = 0.05F;
        color.g = 0.9F;
        color.b = 0.25F;
      }
      marker.points.push_back(toPointMsg(point));
      marker.colors.push_back(color);
    }
    array.markers.push_back(marker);
    return array;
  }

  static std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
  {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  }

  std::string statusFromFailure(const std::string & error) const
  {
    if (error.find("START_INVALID") != std::string::npos) {
      return "START_INVALID";
    }
    if (error.find("GOAL_INVALID") != std::string::npos) {
      return "GOAL_INVALID";
    }
    if (error.find("map") != std::string::npos || error.find("Map") != std::string::npos) {
      return "MAP_ERROR";
    }
    return "FAILED";
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
  AStarPlannerAdapter adapter_;

  std::string map_frame_{"map"};
  std::string odom_frame_{"odom"};
  std::string base_frame_{"base_link"};
  std::string start_source_{"tf"};
  std::string start_pose_topic_{"/astar_global_planner/start_pose"};
  std::string odom_topic_{"/odom"};
  double tf_lookup_timeout_{0.25};
  bool start_z_override_enabled_{false};
  double start_z_override_{0.0};
  std::string goal_pose_topic_{"/goal_pose_3d"};
  std::string goal_point_topic_{"/goal_point_3d"};
  std::string rviz_2d_goal_topic_{"/goal_pose"};
  double default_goal_z_{0.0};
  double default_path_z_{0.0};
  std::string tomogram_topic_{"/tomogram"};
  double tomogram_traversable_cost_threshold_{50.0};
  bool tomogram_z_aware_support_{true};
  bool tomogram_filter_by_z_{true};
  double tomogram_support_z_tolerance_{0.50};
  bool postprocess_enabled_{true};
  bool enable_path_resampling_{true};
  double path_resample_resolution_{0.2};
  bool enable_z_smoothing_{true};
  int z_smoothing_iterations_{30};
  double z_smoothing_alpha_{0.45};
  double z_max_step_{0.12};
  double z_max_slope_{0.80};
  bool enable_path_smoothing_{true};
  int smoothing_iterations_{20};
  bool remove_duplicate_points_{true};
  bool validate_after_smoothing_{true};
  std::string publish_path_topic_{"/planned_path"};
  std::string publish_alias_path_topic_{"/path"};
  std::string publish_marker_topic_{"/planned_path_marker"};
  std::string status_topic_{"/astar_global_planner/status"};
  std::string debug_topic_{"/astar_global_planner/debug"};
  std::string expanded_nodes_marker_topic_{"/astar_expanded_nodes_marker"};
  std::string closed_set_marker_topic_{"/astar_closed_set_marker"};
  std::string clearance_marker_topic_{"/astar_clearance_marker"};
  bool publish_debug_markers_{true};
  double marker_line_width_{0.08};
  bool planning_in_progress_{false};
  std::string status_{"IDLE"};
  std::optional<nav_msgs::msg::Odometry> last_odom_;
  std::optional<geometry_msgs::msg::PoseStamped> last_start_pose_;
  std::optional<Eigen::Vector3d> pending_goal_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr debug_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr alias_path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr expanded_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr closed_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr clearance_marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr start_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr tomogram_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_sub_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AStarGlobalPlannerNode>();
  node->start();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
