#include "local_planning/dwa_3d_local_planner.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <sstream>
#include <utility>

namespace local_planning
{
namespace
{

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Eigen::Vector3d posePosition(const Pose3D & pose)
{
  return Eigen::Vector3d(pose.x, pose.y, pose.z);
}

double distance3D(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return (a - b).norm();
}

double distance2D(const Eigen::Vector3d & a, const Eigen::Vector3d & b)
{
  return std::hypot(a.x() - b.x(), a.y() - b.y());
}

double footprintObstacleDistanceToBounds(
  const Pose3D & pose,
  const Eigen::Vector3d & obstacle,
  double bottom_z,
  double top_z)
{
  const double xy_distance = std::hypot(obstacle.x() - pose.x, obstacle.y() - pose.y);
  double z_distance = 0.0;
  if (obstacle.z() < bottom_z) {
    z_distance = bottom_z - obstacle.z();
  } else if (obstacle.z() > top_z) {
    z_distance = obstacle.z() - top_z;
  }
  return std::hypot(xy_distance, z_distance);
}

bool insideMetricBounds(const octomap::OcTree & tree, double x, double y, double z)
{
  double min_x = 0.0;
  double min_y = 0.0;
  double min_z = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  double max_z = 0.0;
  tree.getMetricMin(min_x, min_y, min_z);
  tree.getMetricMax(max_x, max_y, max_z);
  return x >= min_x && x <= max_x &&
         y >= min_y && y <= max_y &&
         z >= min_z && z <= max_z;
}

std::string toLower(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

}  // namespace

DWA3DLocalPlanner::DWA3DLocalPlanner()
: DWA3DLocalPlanner(Config())
{
}

DWA3DLocalPlanner::DWA3DLocalPlanner(const Config & config)
: config_(config)
{
}

void DWA3DLocalPlanner::setConfig(const Config & config)
{
  config_ = config;
}

const DWA3DLocalPlanner::Config & DWA3DLocalPlanner::getConfig() const
{
  return config_;
}

void DWA3DLocalPlanner::setOctomap(std::shared_ptr<const octomap::OcTree> octree)
{
  octree_ = std::move(octree);
}

bool DWA3DLocalPlanner::hasOctomap() const
{
  return static_cast<bool>(octree_);
}

void DWA3DLocalPlanner::setObstacleCloud(const std::vector<Eigen::Vector3d> & points)
{
  obstacle_cloud_ = points;
}

void DWA3DLocalPlanner::clearObstacleCloud()
{
  obstacle_cloud_.clear();
}

bool DWA3DLocalPlanner::hasObstacleCloud() const
{
  return !obstacle_cloud_.empty();
}

bool DWA3DLocalPlanner::computeVelocityCommand(
  const Pose3D & current_pose,
  const Velocity3D & current_vel,
  const std::vector<Eigen::Vector3d> & global_path,
  Velocity3D & cmd_vel,
  Trajectory3D & best_traj)
{
  last_candidates_.clear();
  last_recovery_traj_ = Trajectory3D();
  last_debug_info_ = DwaDebugInfo();
  last_debug_info_.recovery_state = "idle";
  active_current_pose_ = current_pose;
  active_global_path_ = global_path;
  has_active_context_ = true;
  best_traj = Trajectory3D();

  if (global_path.empty()) {
    return false;
  }

  const Eigen::Vector3d local_goal = selectLocalGoal(current_pose, global_path);
  last_local_goal_ = local_goal;

  const double control_dt = 1.0 / std::max(1.0, config_.control_frequency);
  double vx_min = std::max(config_.min_vx, current_vel.vx - config_.max_acc_vx * control_dt);
  double vx_max = std::min(config_.max_vx, current_vel.vx + config_.max_acc_vx * control_dt);
  double vy_min = std::max(config_.min_vy, current_vel.vy - config_.max_acc_vy * control_dt);
  double vy_max = std::min(config_.max_vy, current_vel.vy + config_.max_acc_vy * control_dt);
  const double wz_min = std::max(config_.min_wz, current_vel.wz - config_.max_acc_wz * control_dt);
  const double wz_max = std::min(config_.max_wz, current_vel.wz + config_.max_acc_wz * control_dt);

  const double dynamic_speed_scale = computeDynamicObstacleSpeedScale(current_pose);
  last_debug_info_.dynamic_obstacle_speed_scale = dynamic_speed_scale;
  last_debug_info_.nearest_dynamic_obstacle_distance = minForwardPointCloudDistance(current_pose);
  if (config_.enable_dynamic_speed_scaling && usePointCloud() && dynamic_speed_scale < 1.0) {
    const double escape_scale =
      dynamic_speed_scale <= 1.0e-6 ?
      clamp(config_.min_speed_scale, 0.0, 1.0) : dynamic_speed_scale;

    if (dynamic_speed_scale <= 1.0e-6) {
      vx_min = std::min(vx_min, 0.0);
      vx_max = 0.0;
    } else {
      vx_min *= dynamic_speed_scale;
      vx_max *= dynamic_speed_scale;
    }

    if (config_.robot_model != "ground_diff") {
      vy_min *= escape_scale;
      vy_max *= escape_scale;
    }
  }

  int vy_samples = config_.vy_samples;
  if (config_.robot_model == "ground_diff") {
    vy_min = 0.0;
    vy_max = 0.0;
    vy_samples = 1;
  }

  const auto vx_values = sampleRange(vx_min, vx_max, config_.vx_samples);
  const auto vy_values = sampleRange(vy_min, vy_max, vy_samples);
  const auto wz_values = sampleRange(wz_min, wz_max, config_.wz_samples);

  bool found_valid = false;
  double best_score = -std::numeric_limits<double>::infinity();

  for (const double vx : vx_values) {
    for (const double vy : vy_values) {
      for (const double wz : wz_values) {
        Velocity3D sampled_cmd;
        sampled_cmd.vx = vx;
        sampled_cmd.vy = vy;
        sampled_cmd.wz = wz;

        Trajectory3D traj = simulateTrajectory(current_pose, sampled_cmd);
        double min_obstacle_distance = std::numeric_limits<double>::infinity();
        double min_dynamic_obstacle_distance = std::numeric_limits<double>::infinity();
        traj.collision_free = evaluateCollisionAndDistance(
          traj.poses, min_obstacle_distance, &last_debug_info_,
          &min_dynamic_obstacle_distance);
        traj.min_obstacle_distance = min_obstacle_distance;
        traj.min_dynamic_obstacle_distance = min_dynamic_obstacle_distance;
        if (traj.collision_free) {
          ++last_debug_info_.valid_trajectories;
          traj.score = scoreTrajectory(traj, current_pose, global_path, local_goal);
          if (!found_valid || traj.score > best_score) {
            found_valid = true;
            best_score = traj.score;
            best_traj = traj;
          }
        } else {
          ++last_debug_info_.collision_trajectories;
          traj.score = -1.0;
        }
        last_candidates_.push_back(std::move(traj));
      }
    }
  }

  if (!found_valid) {
    last_debug_info_.best_score = -std::numeric_limits<double>::infinity();
    return false;
  }

  last_debug_info_.best_score = best_score;
  cmd_vel = best_traj.cmd;
  last_cmd_ = cmd_vel;
  return true;
}

bool DWA3DLocalPlanner::isTrajectoryCollisionFree(const std::vector<Pose3D> & traj) const
{
  double min_obstacle_distance = std::numeric_limits<double>::infinity();
  return evaluateCollisionAndDistance(traj, min_obstacle_distance);
}

double DWA3DLocalPlanner::getMinObstacleDistance(const std::vector<Pose3D> & traj) const
{
  double min_obstacle_distance = std::numeric_limits<double>::infinity();
  evaluateCollisionAndDistance(traj, min_obstacle_distance);
  return min_obstacle_distance;
}

double DWA3DLocalPlanner::getMinDynamicObstacleDistance(const std::vector<Pose3D> & traj) const
{
  double min_obstacle_distance = std::numeric_limits<double>::infinity();
  double min_dynamic_obstacle_distance = std::numeric_limits<double>::infinity();
  evaluateCollisionAndDistance(
    traj, min_obstacle_distance, nullptr, &min_dynamic_obstacle_distance);
  return min_dynamic_obstacle_distance;
}

double DWA3DLocalPlanner::computeDynamicObstacleSpeedScale(const Pose3D & current_pose) const
{
  if (!config_.enable_dynamic_speed_scaling || !usePointCloud()) {
    return 1.0;
  }

  const double dynamic_distance = minForwardPointCloudDistance(current_pose);
  if (!std::isfinite(dynamic_distance)) {
    return 1.0;
  }

  const double stop_distance = std::max(
    dynamicCollisionDistanceThreshold(),
    config_.dynamic_obstacle_stop_distance);
  const double slow_distance = std::max(stop_distance + 1.0e-3, config_.dynamic_obstacle_slow_distance);
  if (dynamic_distance <= stop_distance) {
    return 0.0;
  }
  if (dynamic_distance >= slow_distance) {
    return 1.0;
  }

  const double raw_scale = (dynamic_distance - stop_distance) / (slow_distance - stop_distance);
  return clamp(raw_scale, clamp(config_.min_speed_scale, 0.0, 1.0), 1.0);
}

const std::vector<Trajectory3D> & DWA3DLocalPlanner::getLastCandidateTrajectories() const
{
  return last_candidates_;
}

const Eigen::Vector3d & DWA3DLocalPlanner::getLastLocalGoal() const
{
  return last_local_goal_;
}

const Trajectory3D & DWA3DLocalPlanner::getLastRecoveryTrajectory() const
{
  return last_recovery_traj_;
}

const DwaDebugInfo & DWA3DLocalPlanner::getLastDebugInfo() const
{
  return last_debug_info_;
}

std::vector<double> DWA3DLocalPlanner::sampleRange(
  double min_value,
  double max_value,
  int samples) const
{
  if (max_value < min_value) {
    std::swap(min_value, max_value);
  }

  if (samples <= 1 || std::abs(max_value - min_value) < 1.0e-9) {
    return {0.5 * (min_value + max_value)};
  }

  std::vector<double> values;
  values.reserve(static_cast<std::size_t>(samples));
  const double step = (max_value - min_value) / static_cast<double>(samples - 1);
  for (int i = 0; i < samples; ++i) {
    values.push_back(min_value + static_cast<double>(i) * step);
  }
  return values;
}

Trajectory3D DWA3DLocalPlanner::simulateTrajectory(const Pose3D & start, const Velocity3D & cmd) const
{
  return simulateTrajectoryForDuration(start, cmd, config_.sim_time);
}

Trajectory3D DWA3DLocalPlanner::simulateTrajectoryForDuration(
  const Pose3D & start,
  const Velocity3D & cmd,
  double duration) const
{
  Trajectory3D traj;
  traj.cmd = cmd;

  Pose3D pose = start;
  traj.poses.push_back(pose);

  const double sim_dt = std::max(0.01, config_.sim_dt);
  const int steps = std::max(1, static_cast<int>(std::ceil(std::max(0.0, duration) / sim_dt)));
  for (int i = 0; i < steps; ++i) {
    const double cos_yaw = std::cos(pose.yaw);
    const double sin_yaw = std::sin(pose.yaw);
    pose.x += (cmd.vx * cos_yaw - cmd.vy * sin_yaw) * sim_dt;
    pose.y += (cmd.vx * sin_yaw + cmd.vy * cos_yaw) * sim_dt;
    if (config_.robot_model == "aerial_3d") {
      pose.z += cmd.vz * sim_dt;
    } else if (config_.terrain_following_enabled && config_.use_path_z_for_collision) {
      const double ref_z = getReferenceZFromPath(pose.x, pose.y);
      const double max_jump = std::max(0.0, config_.max_allowed_z_jump);
      if (max_jump > 0.0) {
        pose.z = clamp(ref_z, pose.z - max_jump, pose.z + max_jump);
      } else {
        pose.z = ref_z;
      }
    }
    pose.yaw = normalizeAngle(pose.yaw + cmd.wz * sim_dt);
    traj.poses.push_back(pose);
  }

  return traj;
}

double DWA3DLocalPlanner::getReferenceZFromPath(double x, double y) const
{
  if (!has_active_context_ || active_global_path_.empty()) {
    return active_current_pose_.z;
  }

  double nearest_dist = std::numeric_limits<double>::infinity();
  double nearest_z = active_current_pose_.z;
  for (const auto & path_point : active_global_path_) {
    const double dist = std::hypot(x - path_point.x(), y - path_point.y());
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_z = path_point.z();
    }
  }

  const double search_radius = std::max(0.0, config_.z_search_radius);
  if (nearest_dist <= 0.5 * search_radius) {
    return nearest_z;
  }
  if (nearest_dist <= search_radius) {
    return nearest_z;
  }
  return getInterpolatedPathZ(x, y);
}

double DWA3DLocalPlanner::getInterpolatedPathZ(double x, double y) const
{
  if (!has_active_context_ || active_global_path_.empty()) {
    return active_current_pose_.z;
  }
  if (active_global_path_.size() == 1) {
    const auto & p = active_global_path_.front();
    const double dist = std::hypot(x - p.x(), y - p.y());
    return dist <= std::max(0.0, config_.z_search_radius) ? p.z() : active_current_pose_.z;
  }

  double best_dist = std::numeric_limits<double>::infinity();
  double best_z = active_current_pose_.z;
  for (std::size_t i = 1; i < active_global_path_.size(); ++i) {
    const auto & a = active_global_path_[i - 1];
    const auto & b = active_global_path_[i];
    const double vx = b.x() - a.x();
    const double vy = b.y() - a.y();
    const double length_sq = vx * vx + vy * vy;
    double t = 0.0;
    if (length_sq > 1.0e-9) {
      t = clamp(((x - a.x()) * vx + (y - a.y()) * vy) / length_sq, 0.0, 1.0);
    }
    const double px = a.x() + t * vx;
    const double py = a.y() + t * vy;
    const double dist = std::hypot(x - px, y - py);
    if (dist < best_dist) {
      best_dist = dist;
      best_z = a.z() + t * (b.z() - a.z());
    }
  }

  return best_dist <= std::max(0.0, config_.z_search_radius) ? best_z : active_current_pose_.z;
}

Eigen::Vector3d DWA3DLocalPlanner::selectLocalGoal(
  const Pose3D & current_pose,
  const std::vector<Eigen::Vector3d> & global_path) const
{
  if (global_path.empty()) {
    return posePosition(current_pose);
  }

  const Eigen::Vector3d current = posePosition(current_pose);
  std::size_t nearest_index = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < global_path.size(); ++i) {
    const double dist =
      config_.robot_model == "aerial_3d" ? distance3D(current, global_path[i]) :
      distance2D(current, global_path[i]);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_index = i;
    }
  }

  double lookahead = std::max(0.0, config_.local_goal_lookahead);
  if (config_.adaptive_lookahead_enabled) {
    const double min_lookahead = std::max(0.0, config_.min_local_goal_lookahead);
    const double max_lookahead = std::max(min_lookahead, config_.max_local_goal_lookahead);
    lookahead = clamp(lookahead, min_lookahead, max_lookahead);

    if (nearest_dist > std::max(config_.path_corridor_radius, config_.near_path_bonus_radius)) {
      return global_path[nearest_index];
    }

    double z_accumulated = 0.0;
    const double start_z = global_path[nearest_index].z();
    for (std::size_t i = nearest_index + 1; i < global_path.size(); ++i) {
      z_accumulated += distance2D(global_path[i - 1], global_path[i]);
      if (std::abs(global_path[i].z() - start_z) >= config_.z_change_slowdown_threshold) {
        lookahead = std::min(lookahead, std::max(min_lookahead, z_accumulated));
        break;
      }
      if (z_accumulated >= max_lookahead) {
        break;
      }
    }
  }

  double accumulated = 0.0;
  for (std::size_t i = nearest_index + 1; i < global_path.size(); ++i) {
    accumulated +=
      config_.robot_model == "aerial_3d" ? distance3D(global_path[i - 1], global_path[i]) :
      distance2D(global_path[i - 1], global_path[i]);
    if (accumulated >= lookahead) {
      return global_path[i];
    }
  }
  return global_path.back();
}

double DWA3DLocalPlanner::distanceToPath(
  const Eigen::Vector3d & point,
  const std::vector<Eigen::Vector3d> & global_path) const
{
  if (global_path.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double nearest_dist = std::numeric_limits<double>::infinity();
  for (const auto & path_point : global_path) {
    const double distance =
      config_.robot_model == "aerial_3d" ? distance3D(point, path_point) :
      distance2D(point, path_point);
    nearest_dist = std::min(nearest_dist, distance);
  }
  return nearest_dist;
}

double DWA3DLocalPlanner::planarDistanceToPath(
  const Eigen::Vector3d & point,
  const std::vector<Eigen::Vector3d> & global_path) const
{
  if (global_path.empty()) {
    return std::numeric_limits<double>::infinity();
  }

  double nearest_dist = std::numeric_limits<double>::infinity();
  for (const auto & path_point : global_path) {
    nearest_dist = std::min(nearest_dist, distance2D(point, path_point));
  }
  return nearest_dist;
}

std::size_t DWA3DLocalPlanner::nearestPathIndex2D(
  const Eigen::Vector3d & point,
  const std::vector<Eigen::Vector3d> & global_path) const
{
  if (global_path.empty()) {
    return 0;
  }

  std::size_t nearest_index = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < global_path.size(); ++i) {
    const double dist = distance2D(point, global_path[i]);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_index = i;
    }
  }
  return nearest_index;
}

double DWA3DLocalPlanner::scoreTrajectory(
  const Trajectory3D & traj,
  const Pose3D & current_pose,
  const std::vector<Eigen::Vector3d> & global_path,
  const Eigen::Vector3d & local_goal) const
{
  if (traj.poses.empty()) {
    return -std::numeric_limits<double>::infinity();
  }

  const Pose3D & end_pose = traj.poses.back();
  const Eigen::Vector3d end = posePosition(end_pose);

  const double path_distance = distanceToPath(end, global_path);
  const double path_score = 1.0 / (1.0 + path_distance);

  const double goal_distance = distance3D(end, local_goal);
  const double goal_score = 1.0 / (1.0 + goal_distance);

  const double min_obstacle_distance = traj.min_obstacle_distance;
  const double collision_threshold = collisionDistanceThreshold();
  const double dynamic_collision_threshold = dynamicCollisionDistanceThreshold();
  double obstacle_score = 1.0;
  if (std::isfinite(min_obstacle_distance)) {
    const double score_span =
      std::max(1.0e-3, config_.obstacle_score_distance - collision_threshold);
    obstacle_score = clamp(
      (min_obstacle_distance - collision_threshold) / score_span, 0.0, 1.0);
  }

  double dynamic_obstacle_score = 1.0;
  if (usePointCloud() && std::isfinite(traj.min_dynamic_obstacle_distance)) {
    const double stop_distance = std::max(
      dynamic_collision_threshold,
      config_.dynamic_obstacle_stop_distance);
    const double slow_distance =
      std::max(stop_distance + 1.0e-3, config_.dynamic_obstacle_slow_distance);
    dynamic_obstacle_score =
      clamp(
        (traj.min_dynamic_obstacle_distance - stop_distance) / (slow_distance - stop_distance),
        0.0,
        1.0);
  }

  const double target_yaw = std::atan2(local_goal.y() - end_pose.y, local_goal.x() - end_pose.x);
  const double heading_error = normalizeAngle(target_yaw - end_pose.yaw);
  const double heading_score = 0.5 * (std::cos(heading_error) + 1.0);

  const double speed = std::hypot(traj.cmd.vx, traj.cmd.vy);
  const double max_speed = std::max(1.0e-3, maxPlanarSpeed());
  double speed_scale = 1.0;
  if (!global_path.empty()) {
    const double final_goal_distance = distance3D(posePosition(current_pose), global_path.back());
    speed_scale = clamp(
      final_goal_distance / std::max(1.0e-3, config_.local_goal_lookahead), 0.15, 1.0);
  }
  const double desired_speed = max_speed * speed_scale;
  const double velocity_score = 1.0 - clamp(std::abs(speed - desired_speed) / max_speed, 0.0, 1.0);

  const double cmd_delta =
    std::hypot(traj.cmd.vx - last_cmd_.vx, traj.cmd.vy - last_cmd_.vy) +
    std::abs(traj.cmd.wz - last_cmd_.wz);
  const double smoothness_norm = max_speed + std::max(std::abs(config_.min_wz), std::abs(config_.max_wz));
  const double smoothness_score =
    1.0 - clamp(cmd_delta / std::max(1.0e-3, smoothness_norm), 0.0, 1.0);

  double path_corridor_score = 0.0;
  double z_consistency_score = 0.0;
  double progress_score = 0.0;
  if (!global_path.empty()) {
    const double corridor_radius = std::max(1.0e-3, config_.near_path_bonus_radius);
    path_corridor_score =
      1.0 - clamp(planarDistanceToPath(end, global_path) / corridor_radius, 0.0, 1.0);

    const double ref_z = getReferenceZFromPath(end_pose.x, end_pose.y);
    const double z_scale = std::max(
      1.0e-3,
      std::max(config_.slope_edge_z_tolerance, config_.max_allowed_z_jump));
    z_consistency_score = 1.0 - clamp(std::abs(end_pose.z - ref_z) / z_scale, 0.0, 1.0);

    const std::size_t start_index = nearestPathIndex2D(posePosition(current_pose), global_path);
    const std::size_t end_index = nearestPathIndex2D(end, global_path);
    if (end_index >= start_index) {
      const std::size_t remaining = std::max<std::size_t>(1, global_path.size() - 1 - start_index);
      progress_score = clamp(
        static_cast<double>(end_index - start_index) / static_cast<double>(remaining),
        0.0,
        1.0);
    }
  }

  return config_.weight_path_distance * path_score +
         config_.weight_goal_distance * goal_score +
         config_.weight_obstacle_distance * obstacle_score +
         config_.weight_heading * heading_score +
         config_.weight_velocity * velocity_score +
         config_.weight_smoothness * smoothness_score +
         config_.weight_dynamic_obstacle_distance * dynamic_obstacle_score +
         (config_.near_path_bonus_enabled ? config_.weight_path_corridor * path_corridor_score : 0.0) +
         config_.weight_z_consistency * z_consistency_score +
         config_.weight_progress * progress_score;
}

bool DWA3DLocalPlanner::evaluateCollisionAndDistance(
  const std::vector<Pose3D> & traj,
  double & min_obstacle_distance,
  DwaDebugInfo * debug_info,
  double * min_dynamic_obstacle_distance) const
{
  min_obstacle_distance = std::numeric_limits<double>::infinity();
  double dynamic_min = std::numeric_limits<double>::infinity();
  const double collision_threshold = collisionDistanceThreshold();
  const double dynamic_collision_threshold = dynamicCollisionDistanceThreshold();

  for (std::size_t pose_index = 0; pose_index < traj.size(); ++pose_index) {
    if (pose_index == 0 && traj.size() > 1) {
      continue;
    }
    const auto & pose = traj[pose_index];
    if (usePointCloud()) {
      const double cloud_distance = minPointCloudDistance(pose);
      dynamic_min = std::min(dynamic_min, cloud_distance);
      min_obstacle_distance = std::min(min_obstacle_distance, cloud_distance);
      if (cloud_distance <= dynamic_collision_threshold) {
        if (min_dynamic_obstacle_distance) {
          *min_dynamic_obstacle_distance = dynamic_min;
        }
        return false;
      }
    }

    if (useOctomap() && octree_) {
      min_obstacle_distance = std::min(min_obstacle_distance, minOctomapDistance(pose));
      CollisionReason reason = CollisionReason::kNone;
      if (!isPoseCollisionFreeTerrainAdaptive(pose, &reason)) {
        if (debug_info) {
          if (reason == CollisionReason::kUnknown) {
            ++debug_info->unknown_blocked_count;
          } else if (reason == CollisionReason::kGround) {
            ++debug_info->ground_blocked_count;
          }
        }
        if (min_dynamic_obstacle_distance) {
          *min_dynamic_obstacle_distance = dynamic_min;
        }
        return false;
      }
      if (min_obstacle_distance <= collision_threshold &&
        toLower(config_.collision_model) != "terrain_adaptive_cylinder")
      {
        if (min_dynamic_obstacle_distance) {
          *min_dynamic_obstacle_distance = dynamic_min;
        }
        return false;
      }
    }
  }

  if (min_dynamic_obstacle_distance) {
    *min_dynamic_obstacle_distance = dynamic_min;
  }
  return true;
}

double DWA3DLocalPlanner::minPointCloudDistance(const Pose3D & pose) const
{
  double min_distance = std::numeric_limits<double>::infinity();
  if (config_.dynamic_obstacle_use_2d_footprint) {
    for (const auto & point : obstacle_cloud_) {
      min_distance = std::min(
        min_distance,
        std::hypot(point.x() - pose.x, point.y() - pose.y));
    }
    return min_distance;
  }

  const double bottom_z = collisionBodyMinZ(pose);
  const double top_z = collisionBodyMaxZ(pose);

  for (const auto & point : obstacle_cloud_) {
    if (point.z() < bottom_z || point.z() > top_z + config_.obstacle_score_distance)
    {
      continue;
    }
    min_distance = std::min(
      min_distance,
      footprintObstacleDistanceToBounds(pose, point, bottom_z, top_z));
  }

  return min_distance;
}

double DWA3DLocalPlanner::minForwardPointCloudDistance(const Pose3D & pose) const
{
  if (!usePointCloud()) {
    return std::numeric_limits<double>::infinity();
  }

  double min_distance = std::numeric_limits<double>::infinity();
  const double bottom_z = collisionBodyMinZ(pose);
  const double top_z = collisionBodyMaxZ(pose);
  const double collision_radius = dynamicCollisionDistanceThreshold();
  const double lateral_limit = collision_radius + std::max(0.15, config_.robot_radius);
  const double forward_limit =
    collision_radius + std::max(config_.dynamic_obstacle_slow_distance, config_.obstacle_score_distance);
  const double cos_yaw = std::cos(pose.yaw);
  const double sin_yaw = std::sin(pose.yaw);

  for (const auto & point : obstacle_cloud_) {
    if (!config_.dynamic_obstacle_use_2d_footprint &&
      (point.z() < bottom_z || point.z() > top_z + config_.obstacle_score_distance))
    {
      continue;
    }

    const double dx = point.x() - pose.x;
    const double dy = point.y() - pose.y;
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;
    if (local_x < -collision_radius || local_x > forward_limit ||
      std::abs(local_y) > lateral_limit)
    {
      continue;
    }

    min_distance = std::min(
      min_distance,
      config_.dynamic_obstacle_use_2d_footprint ?
      std::hypot(dx, dy) :
      footprintObstacleDistanceToBounds(pose, point, bottom_z, top_z));
  }

  return min_distance;
}

double DWA3DLocalPlanner::minOctomapDistance(const Pose3D & pose) const
{
  if (!octree_) {
    return std::numeric_limits<double>::infinity();
  }

  const double query_radius = std::max(config_.obstacle_score_distance, collisionDistanceThreshold());
  const double bottom_z = collisionBodyMinZ(pose);
  const double top_z = collisionBodyMaxZ(pose);
  const octomap::point3d min_point(
    static_cast<float>(pose.x - query_radius),
    static_cast<float>(pose.y - query_radius),
    static_cast<float>(bottom_z));
  const octomap::point3d max_point(
    static_cast<float>(pose.x + query_radius),
    static_cast<float>(pose.y + query_radius),
    static_cast<float>(top_z + query_radius));

  const Eigen::Vector3d pose_point(pose.x, pose.y, pose.z);
  if (!isUnknownAllowed(pose_point) &&
    (!insideMetricBounds(*octree_, min_point.x(), min_point.y(), min_point.z()) ||
    !insideMetricBounds(*octree_, max_point.x(), max_point.y(), max_point.z())))
  {
    return 0.0;
  }

  double min_distance = std::numeric_limits<double>::infinity();
  for (auto it = octree_->begin_leafs_bbx(min_point, max_point); it != octree_->end_leafs_bbx(); ++it) {
    if (!octree_->isNodeOccupied(*it)) {
      continue;
    }
    min_distance = std::min(
      min_distance,
      footprintObstacleDistanceToBounds(
        pose,
        Eigen::Vector3d(it.getX(), it.getY(), it.getZ()),
        bottom_z,
        top_z));
  }

  return min_distance;
}

bool DWA3DLocalPlanner::octomapFootprintCollision(const Pose3D & pose) const
{
  return !isPoseCollisionFreeTerrainAdaptive(pose);
}

bool DWA3DLocalPlanner::isPoseCollisionFreeTerrainAdaptive(const Pose3D & pose) const
{
  CollisionReason reason = CollisionReason::kNone;
  return isPoseCollisionFreeTerrainAdaptive(pose, &reason);
}

bool DWA3DLocalPlanner::isUnknownAllowed(const Eigen::Vector3d & point) const
{
  const std::string policy = toLower(config_.unknown_policy);
  if (policy == "free") {
    return true;
  }
  if (policy == "occupied") {
    return false;
  }
  if (policy == "path_corridor_free") {
    return planarDistanceToPath(point, active_global_path_) <= std::max(0.0, config_.path_corridor_radius);
  }
  return !config_.unknown_as_occupied;
}

bool DWA3DLocalPlanner::isPoseCollisionFreeTerrainAdaptive(
  const Pose3D & pose,
  CollisionReason * reason) const
{
  if (reason) {
    *reason = CollisionReason::kNone;
  }
  if (!octree_) {
    return true;
  }

  const double radius = collisionDistanceThreshold();
  const double resolution = std::max(
    0.02,
    config_.obstacle_check_resolution > 0.0 ?
    config_.obstacle_check_resolution : octree_->getResolution());
  const double bottom_z = collisionBodyMinZ(pose);
  const double top_z = collisionBodyMaxZ(pose);
  const bool terrain_adaptive = toLower(config_.collision_model) == "terrain_adaptive_cylinder";

  int occupied_hits = 0;

  for (double z = bottom_z; z <= top_z + 1.0e-9; z += resolution) {
    for (double dx = -radius; dx <= radius + 1.0e-9; dx += resolution) {
      for (double dy = -radius; dy <= radius + 1.0e-9; dy += resolution) {
        if (dx * dx + dy * dy > radius * radius) {
          continue;
        }

        const double x = pose.x + dx;
        const double y = pose.y + dy;
        const auto * node = octree_->search(x, y, z);
        if (node && octree_->isNodeOccupied(node)) {
          ++occupied_hits;
          continue;
        }
        if (!node && !isUnknownAllowed(Eigen::Vector3d(x, y, z))) {
          if (reason) {
            *reason = CollisionReason::kUnknown;
          }
          return false;
        }
      }
    }
  }

  if (occupied_hits == 0) {
    return true;
  }

  if (!terrain_adaptive || !config_.slope_edge_relaxation_enabled ||
    !isNearActivePath2D(pose, std::max(0.0, config_.slope_edge_relaxation_radius)))
  {
    if (reason) {
      *reason = terrain_adaptive ? CollisionReason::kGround : CollisionReason::kOccupied;
    }
    return false;
  }

  const double upper_z = top_z + std::max(0.0, config_.slope_edge_z_tolerance);
  int upper_hits = 0;
  for (double z = top_z + resolution; z <= upper_z + 1.0e-9; z += resolution) {
    for (double dx = -radius; dx <= radius + 1.0e-9; dx += resolution) {
      for (double dy = -radius; dy <= radius + 1.0e-9; dy += resolution) {
        if (dx * dx + dy * dy > radius * radius) {
          continue;
        }
        const double x = pose.x + dx;
        const double y = pose.y + dy;
        const auto * node = octree_->search(x, y, z);
        if (node && octree_->isNodeOccupied(node)) {
          ++upper_hits;
        }
      }
    }
  }

  if (upper_hits == 0) {
    return true;
  }

  if (reason) {
    *reason = CollisionReason::kOccupied;
  }
  return false;
}

double DWA3DLocalPlanner::collisionBodyMinZ(const Pose3D & pose) const
{
  if (toLower(config_.collision_model) != "terrain_adaptive_cylinder") {
    return pose.z + config_.collision_z_offset;
  }

  double offset = std::max(config_.body_z_offset, config_.ground_clearance);
  if (config_.ignore_ground_below_base) {
    offset = std::max(offset, config_.ground_ignore_depth);
  }
  return pose.z + std::max(0.0, offset);
}

double DWA3DLocalPlanner::collisionBodyMaxZ(const Pose3D & pose) const
{
  if (toLower(config_.collision_model) != "terrain_adaptive_cylinder") {
    return pose.z + config_.collision_z_offset + std::max(0.0, config_.robot_height);
  }

  const double top_z = pose.z + std::max(0.0, config_.robot_height);
  return std::max(collisionBodyMinZ(pose), top_z);
}

bool DWA3DLocalPlanner::isNearActivePath2D(const Pose3D & pose, double radius) const
{
  if (active_global_path_.empty()) {
    return false;
  }
  return planarDistanceToPath(Eigen::Vector3d(pose.x, pose.y, pose.z), active_global_path_) <= radius;
}

bool DWA3DLocalPlanner::computeRecoveryCommand(Velocity3D & cmd_vel)
{
  Trajectory3D recovery_traj;
  return computeRecoveryCommand(cmd_vel, recovery_traj);
}

bool DWA3DLocalPlanner::computeRecoveryCommand(
  Velocity3D & cmd_vel,
  Trajectory3D & recovery_traj)
{
  last_recovery_traj_ = Trajectory3D();
  if (!config_.recovery_enabled || !has_active_context_) {
    last_debug_info_.recovery_state = "disabled";
    return false;
  }

  std::vector<std::pair<std::string, Velocity3D>> candidates;
  if (config_.enable_reverse_escape) {
    Velocity3D reverse;
    reverse.vx = std::max(config_.min_vx, -0.15);
    candidates.emplace_back("reverse", reverse);
  }
  if (config_.enable_lateral_escape && config_.robot_model != "ground_diff") {
    Velocity3D left;
    left.vy = std::min(config_.max_vy, 0.15);
    candidates.emplace_back("lateral_left", left);

    Velocity3D right;
    right.vy = std::max(config_.min_vy, -0.15);
    candidates.emplace_back("lateral_right", right);
  }
  if (config_.enable_rotate_escape) {
    Velocity3D rotate_left;
    rotate_left.wz = std::min(config_.max_wz, 0.4);
    candidates.emplace_back("rotate_left", rotate_left);

    Velocity3D rotate_right;
    rotate_right.wz = std::max(config_.min_wz, -0.4);
    candidates.emplace_back("rotate_right", rotate_right);
  }

  for (const auto & named_cmd : candidates) {
    Trajectory3D candidate = simulateTrajectoryForDuration(
      active_current_pose_, named_cmd.second, std::max(0.1, config_.recovery_duration));
    double min_obstacle_distance = std::numeric_limits<double>::infinity();
    candidate.collision_free = evaluateCollisionAndDistance(candidate.poses, min_obstacle_distance);
    candidate.min_obstacle_distance = min_obstacle_distance;
    if (!candidate.collision_free) {
      continue;
    }

    cmd_vel = named_cmd.second;
    recovery_traj = candidate;
    last_recovery_traj_ = candidate;
    last_cmd_ = cmd_vel;
    last_debug_info_.recovery_state = named_cmd.first;
    return true;
  }

  last_debug_info_.recovery_state = "blocked";
  return false;
}

bool DWA3DLocalPlanner::useOctomap() const
{
  const std::string source = toLower(config_.obstacle_source);
  return source == "octomap" || source == "both";
}

bool DWA3DLocalPlanner::usePointCloud() const
{
  const std::string source = toLower(config_.obstacle_source);
  return (source == "pointcloud" || source == "both") && !obstacle_cloud_.empty();
}

double DWA3DLocalPlanner::collisionDistanceThreshold() const
{
  return std::max(
    config_.robot_radius + config_.safety_margin,
    config_.min_obstacle_distance);
}

double DWA3DLocalPlanner::dynamicCollisionDistanceThreshold() const
{
  return std::max(
    config_.dynamic_obstacle_radius + config_.dynamic_obstacle_safety_margin,
    config_.min_obstacle_distance);
}

double DWA3DLocalPlanner::maxPlanarSpeed() const
{
  const double max_abs_vx = std::max(std::abs(config_.min_vx), std::abs(config_.max_vx));
  const double max_abs_vy =
    config_.robot_model == "ground_diff" ? 0.0 :
    std::max(std::abs(config_.min_vy), std::abs(config_.max_vy));
  return std::hypot(max_abs_vx, max_abs_vy);
}

}  // namespace local_planning
