#include "global_planner/global_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>
#include <utility>

namespace global_planner
{

namespace
{
std::vector<GridIndex> make26Directions()
{
  std::vector<GridIndex> directions;
  directions.reserve(26);
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dz = -1; dz <= 1; ++dz) {
        if (dx == 0 && dy == 0 && dz == 0) {
          continue;
        }
        directions.push_back(GridIndex{dx, dy, dz});
      }
    }
  }
  return directions;
}

bool samePoint(const Eigen::Vector3d & lhs, const Eigen::Vector3d & rhs)
{
  return (lhs - rhs).norm() <= 1.0e-9;
}
}  // namespace

GlobalPlannerCore::GlobalPlannerCore(const OctomapVoxelMap & map)
: map_(map)
{
}

GlobalPlannerCore::GlobalPlannerCore(const OctomapVoxelMap & map, Options options)
: map_(map), options_(std::move(options))
{
}

void GlobalPlannerCore::setOptions(const Options & options)
{
  options_ = options;
}

bool GlobalPlannerCore::plan(
  const Eigen::Vector3d & start,
  const Eigen::Vector3d & goal,
  std::vector<Eigen::Vector3d> & path)
{
  path.clear();
  last_iterations_ = 0;
  last_failure_reason_.clear();

  if (!map_.isReady()) {
    setFailure("map not ready");
    return false;
  }
  if (!map_.isInBounds(start)) {
    setFailure("start out of bounds");
    return false;
  }
  if (!map_.isInBounds(goal)) {
    setFailure("goal out of bounds");
    return false;
  }
  if (!map_.isFree(start)) {
    setFailure("start occupied");
    return false;
  }
  if (!map_.isFree(goal)) {
    setFailure("goal occupied");
    return false;
  }

  const GridIndex start_index = map_.worldToGrid(start);
  const GridIndex goal_index = map_.worldToGrid(goal);

  if (!isTraversable(start_index)) {
    setFailure("start occupied");
    return false;
  }
  if (!isTraversable(goal_index)) {
    setFailure("goal occupied");
    return false;
  }

  if (start_index == goal_index) {
    path.push_back(map_.gridToWorld(start_index));
    return true;
  }

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
  std::unordered_map<GridIndex, double, GridIndexHash> g_score;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
  std::unordered_set<GridIndex, GridIndexHash> closed_set;

  g_score[start_index] = 0.0;
  open_set.push(QueueNode{start_index, gridDistance(start_index, goal_index), 0.0});

  const std::vector<GridIndex> directions = make26Directions();
  const int max_iterations = std::max(1, options_.max_iterations);

  while (!open_set.empty() && last_iterations_ < max_iterations) {
    const QueueNode current = open_set.top();
    open_set.pop();
    ++last_iterations_;

    if (closed_set.find(current.index) != closed_set.end()) {
      continue;
    }
    closed_set.insert(current.index);

    if (current.index == goal_index) {
      const auto cells = reconstructPath(came_from, current.index);
      path = toWorldPath(cells);
      smoothPath(path);
      resamplePath(path);
      return true;
    }

    for (const auto & direction : directions) {
      const GridIndex neighbor{
        current.index.x + direction.x,
        current.index.y + direction.y,
        current.index.z + direction.z};

      if (closed_set.find(neighbor) != closed_set.end()) {
        continue;
      }
      if (!isTraversable(neighbor)) {
        continue;
      }

      const double step_cost = gridDistance(current.index, neighbor);
      const double tentative_g = current.g + step_cost;
      auto g_score_it = g_score.find(neighbor);
      if (g_score_it == g_score.end() || tentative_g < g_score_it->second) {
        came_from[neighbor] = current.index;
        g_score[neighbor] = tentative_g;
        open_set.push(
          QueueNode{neighbor, tentative_g + gridDistance(neighbor, goal_index), tentative_g});
      }
    }
  }

  if (last_iterations_ >= max_iterations) {
    setFailure("search timeout / max_iterations reached");
  } else {
    setFailure("no path");
  }
  return false;
}

const std::string & GlobalPlannerCore::lastFailureReason() const
{
  return last_failure_reason_;
}

int GlobalPlannerCore::lastIterations() const
{
  return last_iterations_;
}

bool GlobalPlannerCore::isTraversable(const GridIndex & index) const
{
  return map_.isGridInBounds(index) && map_.isFree(map_.gridToWorld(index));
}

double GlobalPlannerCore::gridDistance(const GridIndex & lhs, const GridIndex & rhs) const
{
  const double dx = static_cast<double>(lhs.x - rhs.x);
  const double dy = static_cast<double>(lhs.y - rhs.y);
  const double dz = static_cast<double>(lhs.z - rhs.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz) * map_.getResolution();
}

std::vector<GridIndex> GlobalPlannerCore::reconstructPath(
  const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
  GridIndex current) const
{
  std::vector<GridIndex> path;
  path.push_back(current);
  while (came_from.find(current) != came_from.end()) {
    current = came_from.at(current);
    path.push_back(current);
  }
  std::reverse(path.begin(), path.end());
  return path;
}

std::vector<Eigen::Vector3d> GlobalPlannerCore::toWorldPath(
  const std::vector<GridIndex> & cells) const
{
  std::vector<Eigen::Vector3d> path;
  path.reserve(cells.size());
  for (const auto & cell : cells) {
    path.push_back(map_.gridToWorld(cell));
  }
  return path;
}

bool GlobalPlannerCore::hasLineOfSight(
  const Eigen::Vector3d & from,
  const Eigen::Vector3d & to) const
{
  const double distance = (to - from).norm();
  if (distance <= 1.0e-9) {
    return map_.isFree(from);
  }

  const double step = std::max(0.02, 0.5 * map_.getResolution());
  const int samples = std::max(1, static_cast<int>(std::ceil(distance / step)));
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::Vector3d point = from + t * (to - from);
    if (!map_.isFree(point)) {
      return false;
    }
  }
  return true;
}

void GlobalPlannerCore::smoothPath(std::vector<Eigen::Vector3d> & path) const
{
  if (!options_.enable_path_smoothing || options_.smoothing_method != "line_of_sight" ||
    path.size() < 3)
  {
    return;
  }

  std::vector<Eigen::Vector3d> smoothed;
  smoothed.reserve(path.size());
  smoothed.push_back(path.front());

  std::size_t anchor = 0;
  while (anchor + 1 < path.size()) {
    std::size_t next = path.size() - 1;
    while (next > anchor + 1 && !hasLineOfSight(path[anchor], path[next])) {
      --next;
    }
    smoothed.push_back(path[next]);
    anchor = next;
  }

  path = std::move(smoothed);
}

void GlobalPlannerCore::resamplePath(std::vector<Eigen::Vector3d> & path) const
{
  const double spacing = options_.path_resample_resolution;
  if (spacing <= 1.0e-6 || path.size() < 2) {
    return;
  }

  std::vector<Eigen::Vector3d> resampled;
  resampled.reserve(path.size());
  resampled.push_back(path.front());

  double remaining = spacing;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3d from = path[i - 1];
    const Eigen::Vector3d to = path[i];
    const Eigen::Vector3d delta = to - from;
    const double distance = delta.norm();
    if (distance <= 1.0e-9) {
      continue;
    }

    const Eigen::Vector3d direction = delta / distance;
    double traveled = 0.0;
    while (traveled + remaining <= distance) {
      traveled += remaining;
      const Eigen::Vector3d point = from + direction * traveled;
      if (!samePoint(point, resampled.back())) {
        resampled.push_back(point);
      }
      remaining = spacing;
    }
    remaining -= distance - traveled;
    if (remaining <= 1.0e-6) {
      remaining = spacing;
    }
  }

  if (!samePoint(path.back(), resampled.back())) {
    resampled.push_back(path.back());
  }
  path = std::move(resampled);
}

void GlobalPlannerCore::setFailure(const std::string & reason)
{
  last_failure_reason_ = reason;
}

}  // namespace global_planner
