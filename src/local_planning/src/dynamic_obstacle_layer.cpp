#include "local_planning/dynamic_obstacle_layer.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <tuple>
#include <unordered_map>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace local_planning
{
namespace
{

std::string toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

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
    std::size_t seed = static_cast<std::size_t>(key.x) * 73856093U;
    seed ^= static_cast<std::size_t>(key.y) * 19349663U;
    seed ^= static_cast<std::size_t>(key.z) * 83492791U;
    return seed;
  }
};

struct VoxelAccum
{
  Eigen::Vector3d sum{0.0, 0.0, 0.0};
  int count{0};
};

}  // namespace

void DynamicObstacleLayer::setConfig(const Config & config)
{
  config_ = config;
}

const DynamicObstacleLayer::Config & DynamicObstacleLayer::getConfig() const
{
  return config_;
}

void DynamicObstacleLayer::setCurrentPose(const Pose3D & pose)
{
  current_pose_ = pose;
}

void DynamicObstacleLayer::updateFromPointCloud(const sensor_msgs::msg::PointCloud2 & msg)
{
  obstacle_points_.clear();
  const std::string frame_mode = toLower(config_.pointcloud_frame_mode);
  const bool input_is_base = frame_mode == "base_link" || frame_mode == "base";

  std::unordered_map<VoxelKey, VoxelAccum, VoxelKeyHash> voxel_map;
  const double leaf_size = std::max(1.0e-4, config_.voxel_leaf_size);

  try {
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }

      Eigen::Vector3d point_base;
      Eigen::Vector3d point_map;
      if (input_is_base) {
        point_base = Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
        point_map = pointBaseToMap(point_base);
      } else {
        point_map = Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
        point_base = pointMapToBase(point_map);
      }

      if (!shouldKeepPoint(point_base, point_map)) {
        continue;
      }

      if (config_.enable_voxel_filter) {
        const VoxelKey key{
          static_cast<int>(std::floor(point_map.x() / leaf_size)),
          static_cast<int>(std::floor(point_map.y() / leaf_size)),
          static_cast<int>(std::floor(point_map.z() / leaf_size))};
        auto & accum = voxel_map[key];
        accum.sum += point_map;
        accum.count += 1;
      } else {
        obstacle_points_.push_back(point_map);
      }
    }
  } catch (const std::exception &) {
    obstacle_points_.clear();
  }

  if (config_.enable_voxel_filter) {
    obstacle_points_.reserve(voxel_map.size());
    for (const auto & item : voxel_map) {
      if (item.second.count > 0) {
        obstacle_points_.push_back(item.second.sum / static_cast<double>(item.second.count));
      }
    }
  }

  last_update_time_ = std::chrono::steady_clock::now();
  has_cloud_ = true;
}

void DynamicObstacleLayer::clear()
{
  obstacle_points_.clear();
  has_cloud_ = false;
}

bool DynamicObstacleLayer::hasCloud() const
{
  return has_cloud_;
}

bool DynamicObstacleLayer::hasObstacles() const
{
  return !obstacle_points_.empty();
}

bool DynamicObstacleLayer::isFresh() const
{
  return has_cloud_ && ageSeconds() <= std::max(0.0, config_.pointcloud_timeout);
}

bool DynamicObstacleLayer::isExpired() const
{
  return has_cloud_ && ageSeconds() > std::max(0.0, config_.dynamic_obstacle_decay_time);
}

double DynamicObstacleLayer::ageSeconds() const
{
  if (!has_cloud_) {
    return std::numeric_limits<double>::infinity();
  }
  const auto now = std::chrono::steady_clock::now();
  return std::chrono::duration<double>(now - last_update_time_).count();
}

bool DynamicObstacleLayer::isPointCollisionFree(
  const Eigen::Vector3d & p,
  double radius,
  double height) const
{
  const double checked_radius = std::max(0.0, radius);
  const double checked_height = std::max(0.0, height);
  for (const auto & obstacle : obstacle_points_) {
    const bool z_hit =
      obstacle.z() >= p.z() + config_.body_z_offset &&
      obstacle.z() <= p.z() + checked_height;
    if (!z_hit) {
      continue;
    }
    if (std::hypot(obstacle.x() - p.x(), obstacle.y() - p.y()) <= checked_radius) {
      return false;
    }
  }
  return true;
}

bool DynamicObstacleLayer::isTrajectoryCollisionFree(const std::vector<Pose3D> & traj) const
{
  const double radius = config_.robot_radius + config_.safety_margin;
  for (const auto & pose : traj) {
    if (!isPointCollisionFree(
        Eigen::Vector3d(pose.x, pose.y, pose.z), radius, config_.robot_height))
    {
      return false;
    }
  }
  return true;
}

double DynamicObstacleLayer::getMinDistanceToTrajectory(const std::vector<Pose3D> & traj) const
{
  double min_distance = std::numeric_limits<double>::infinity();
  for (const auto & pose : traj) {
    const Eigen::Vector3d p(pose.x, pose.y, pose.z);
    for (const auto & obstacle : obstacle_points_) {
      min_distance = std::min(
        min_distance,
        pointDistanceToBody(obstacle, p, config_.robot_radius + config_.safety_margin, config_.robot_height));
    }
  }
  return min_distance;
}

const std::vector<Eigen::Vector3d> & DynamicObstacleLayer::obstaclePoints() const
{
  return obstacle_points_;
}

Eigen::Vector3d DynamicObstacleLayer::pointMapToBase(const Eigen::Vector3d & point_map) const
{
  const double dx = point_map.x() - current_pose_.x;
  const double dy = point_map.y() - current_pose_.y;
  const double cos_yaw = std::cos(current_pose_.yaw);
  const double sin_yaw = std::sin(current_pose_.yaw);
  return Eigen::Vector3d(
    cos_yaw * dx + sin_yaw * dy,
    -sin_yaw * dx + cos_yaw * dy,
    point_map.z() - current_pose_.z);
}

Eigen::Vector3d DynamicObstacleLayer::pointBaseToMap(const Eigen::Vector3d & point_base) const
{
  const double cos_yaw = std::cos(current_pose_.yaw);
  const double sin_yaw = std::sin(current_pose_.yaw);
  return Eigen::Vector3d(
    current_pose_.x + cos_yaw * point_base.x() - sin_yaw * point_base.y(),
    current_pose_.y + sin_yaw * point_base.x() + cos_yaw * point_base.y(),
    current_pose_.z + point_base.z());
}

bool DynamicObstacleLayer::shouldKeepPoint(
  const Eigen::Vector3d & point_base,
  const Eigen::Vector3d & point_map) const
{
  if (point_base.x() < -config_.local_cloud_range_x ||
    point_base.x() > config_.local_cloud_range_x ||
    point_base.y() < -config_.local_cloud_range_y ||
    point_base.y() > config_.local_cloud_range_y ||
    point_base.z() < config_.local_cloud_range_z_min ||
    point_base.z() > config_.local_cloud_range_z_max)
  {
    return false;
  }

  if (config_.ignore_robot_self_points &&
    std::hypot(point_base.x(), point_base.y()) <= std::max(0.0, config_.self_filter_radius) &&
    point_base.z() >= config_.self_filter_z_min &&
    point_base.z() <= config_.self_filter_z_max)
  {
    return false;
  }

  if (config_.remove_ground_points) {
    const double ground_z =
      config_.ground_relative_to_base ? point_base.z() : point_map.z();
    if (ground_z <= config_.ground_z_threshold) {
      return false;
    }
  }

  return true;
}

double DynamicObstacleLayer::pointDistanceToBody(
  const Eigen::Vector3d & obstacle,
  const Eigen::Vector3d & body_reference,
  double radius,
  double height) const
{
  const double xy = std::hypot(obstacle.x() - body_reference.x(), obstacle.y() - body_reference.y());
  const double bottom_z = body_reference.z() + config_.body_z_offset;
  const double top_z = body_reference.z() + std::max(0.0, height);
  double z_distance = 0.0;
  if (obstacle.z() < bottom_z) {
    z_distance = bottom_z - obstacle.z();
  } else if (obstacle.z() > top_z) {
    z_distance = obstacle.z() - top_z;
  }
  return std::hypot(std::max(0.0, xy - std::max(0.0, radius)), z_distance);
}

}  // namespace local_planning
