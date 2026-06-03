#include "global_planner/global_planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace global_planner
{

std::size_t GridIndexHash::operator()(const GridIndex & key) const
{
  const std::size_t h1 = std::hash<int>{}(key.x);
  const std::size_t h2 = std::hash<int>{}(key.y);
  const std::size_t h3 = std::hash<int>{}(key.z);
  return h1 ^ (h2 << 1U) ^ (h3 << 2U);
}

bool GlobalPlannerCore::QueueNodeCompare::operator()(
  const QueueNode & lhs,
  const QueueNode & rhs) const
{
  return lhs.f > rhs.f;
}

GlobalPlannerCore::GlobalPlannerCore(const PlannerOptions & options)
: options_(options)
{
}

void GlobalPlannerCore::setOptions(const PlannerOptions & options)
{
  options_ = options;
  if (octree_) {
    rebuildLayers();
  }
}

const PlannerOptions & GlobalPlannerCore::options() const
{
  return options_;
}

void GlobalPlannerCore::setOctomap(std::shared_ptr<octomap::OcTree> map)
{
  octree_ = std::move(map);
  rebuildLayers();
}

bool GlobalPlannerCore::hasMap() const
{
  return static_cast<bool>(octree_);
}

PlanResult GlobalPlannerCore::makePlan(const Point3D & start_point, const Point3D & goal_point)
{
  PlanResult result;
  result.start = start_point;
  result.goal = goal_point;

  if (!octree_) {
    result.message = "OctoMap is not available";
    return result;
  }

  const GridIndex start_raw = worldToGrid(start_point.x, start_point.y, start_point.z);
  const GridIndex goal_raw = worldToGrid(goal_point.x, goal_point.y, goal_point.z);

  GridIndex start = start_raw;
  GridIndex goal = goal_raw;
  if (!findNearestFreeCell(start_raw, start)) {
    result.message = "Start is occupied/outside the map and no nearby traversable cell was found";
    return result;
  }
  if (!findNearestFreeCell(goal_raw, goal)) {
    result.message = "Goal is occupied/outside the map and no nearby traversable cell was found";
    return result;
  }

  result.snapped_start = !(start == start_raw);
  result.snapped_goal = !(goal == goal_raw);
  result.start = gridToPoint(start);
  result.goal = gridToPoint(goal);

  std::priority_queue<QueueNode, std::vector<QueueNode>, QueueNodeCompare> open_set;
  std::unordered_map<GridIndex, double, GridIndexHash> g_score;
  std::unordered_map<GridIndex, GridIndex, GridIndexHash> came_from;
  std::unordered_set<GridIndex, GridIndexHash> closed_set;

  g_score[start] = 0.0;
  open_set.push(QueueNode{start, euclidean(start, goal), 0.0});

  const std::vector<GridIndex> directions = make26Directions();
  while (!open_set.empty() && result.iterations < options_.max_iterations) {
    const QueueNode current = open_set.top();
    open_set.pop();
    ++result.iterations;

    if (closed_set.find(current.idx) != closed_set.end()) {
      continue;
    }
    closed_set.insert(current.idx);

    if (current.idx == goal) {
      const auto cells = reconstructPath(came_from, current.idx);
      result.path.reserve(cells.size());
      for (const auto & cell : cells) {
        result.path.push_back(gridToPoint(cell));
      }
      result.success = true;
      result.message = "A* path found";
      return result;
    }

    for (const auto & direction : directions) {
      const GridIndex neighbor{
        current.idx.x + direction.x,
        current.idx.y + direction.y,
        current.idx.z + direction.z};

      if (closed_set.find(neighbor) != closed_set.end() || !isCellTraversable(neighbor)) {
        continue;
      }

      double tentative_g = current.g + euclidean(current.idx, neighbor);
      if (options_.enable_preblocked_costmap) {
        tentative_g += options_.preblocked_costmap_weight * getPreblockedCost(neighbor);
      }

      const auto score_it = g_score.find(neighbor);
      if (score_it == g_score.end() || tentative_g < score_it->second) {
        came_from[neighbor] = current.idx;
        g_score[neighbor] = tentative_g;
        open_set.push(QueueNode{neighbor, tentative_g + euclidean(neighbor, goal), tentative_g});
      }
    }
  }

  result.message = result.iterations >= options_.max_iterations ?
    "A* reached max_iterations without finding a path" :
    "A* open set is empty; no path exists";
  return result;
}

const std::unordered_set<GridIndex, GridIndexHash> & GlobalPlannerCore::traversableCells() const
{
  return traversable_cells_;
}

const std::unordered_set<GridIndex, GridIndexHash> & GlobalPlannerCore::preblockedCells() const
{
  return preblocked_cells_;
}

void GlobalPlannerCore::rebuildLayers()
{
  traversable_cells_.clear();
  preblocked_cells_.clear();
  preblocked_costmap_.clear();

  if (!octree_) {
    return;
  }

  rebuildPreblockedCells();
  rebuildTraversableCells();
  rebuildPreblockedCostmap();
}

void GlobalPlannerCore::rebuildPreblockedCells()
{
  std::unordered_set<GridIndex, GridIndexHash> candidates;
  for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
    if (!octree_->isNodeOccupied(*it)) {
      continue;
    }

    const GridIndex occupied = worldToGrid(it.getX(), it.getY(), it.getZ());
    for (int dx = -1; dx <= 1; ++dx) {
      for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) {
          continue;
        }
        candidates.insert(GridIndex{occupied.x + dx, occupied.y + dy, occupied.z});
      }
    }
  }

  for (const auto & candidate : candidates) {
    if (!isInsideMetricBounds(candidate) || isOccupiedCell(candidate)) {
      continue;
    }

    const GridIndex below{candidate.x, candidate.y, candidate.z - 1};
    const bool has_occupied_below = isInsideMetricBounds(below) && isOccupiedCell(below);
    if (has_occupied_below && hasSameLevelNeighborWithOccupiedAbove(candidate)) {
      preblocked_cells_.insert(candidate);
      continue;
    }

    const GridIndex above{candidate.x, candidate.y, candidate.z + 1};
    if (!hasNonOccupiedNeighborSameLevel(candidate) ||
      (isInsideMetricBounds(above) && isOccupiedCell(above)))
    {
      continue;
    }

    if (!isInsideMetricBounds(below) || !isOccupiedCell(below)) {
      preblocked_cells_.insert(candidate);
    }
  }
}

void GlobalPlannerCore::rebuildTraversableCells()
{
  double min_x{};
  double min_y{};
  double min_z{};
  double max_x{};
  double max_y{};
  double max_z{};
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
  const GridIndex max_idx = worldToGrid(max_x, max_y, max_z);

  for (int x = min_idx.x; x <= max_idx.x; ++x) {
    for (int y = min_idx.y; y <= max_idx.y; ++y) {
      for (int z = min_idx.z; z <= max_idx.z; ++z) {
        const GridIndex idx{x, y, z};
        if (!isInsideMetricBounds(idx) || isOccupiedCell(idx)) {
          continue;
        }
        if (isCellTraversable(idx)) {
          traversable_cells_.insert(idx);
          if (options_.lowest_traversable_only) {
            break;
          }
        }
      }
    }
  }
}

void GlobalPlannerCore::rebuildPreblockedCostmap()
{
  if (!options_.enable_preblocked_costmap) {
    return;
  }

  const int radius_cells = std::max(1, options_.preblocked_costmap_radius_cells);
  const double denominator = static_cast<double>(radius_cells) + 1.0;

  for (const auto & cell : preblocked_cells_) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
          if (dx == 0 && dy == 0 && dz == 0) {
            continue;
          }

          const GridIndex neighbor{cell.x + dx, cell.y + dy, cell.z + dz};
          if (!isInsideMetricBounds(neighbor) ||
            traversable_cells_.find(neighbor) == traversable_cells_.end() ||
            preblocked_cells_.find(neighbor) != preblocked_cells_.end())
          {
            continue;
          }

          const double distance = std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz));
          if (distance > static_cast<double>(radius_cells)) {
            continue;
          }

          const double cost = std::max(0.0, (denominator - distance) / denominator);
          auto cost_it = preblocked_costmap_.find(neighbor);
          if (cost_it == preblocked_costmap_.end() || cost > cost_it->second) {
            preblocked_costmap_[neighbor] = cost;
          }
        }
      }
    }
  }
}

GridIndex GlobalPlannerCore::worldToGrid(double x, double y, double z) const
{
  const double resolution = octree_->getResolution();
  return GridIndex{
    static_cast<int>(std::floor(x / resolution)),
    static_cast<int>(std::floor(y / resolution)),
    static_cast<int>(std::floor(z / resolution))};
}

octomap::point3d GlobalPlannerCore::gridToWorld(const GridIndex & idx) const
{
  const double resolution = octree_->getResolution();
  return octomap::point3d(
    static_cast<float>((static_cast<double>(idx.x) + 0.5) * resolution),
    static_cast<float>((static_cast<double>(idx.y) + 0.5) * resolution),
    static_cast<float>((static_cast<double>(idx.z) + 0.5) * resolution));
}

Point3D GlobalPlannerCore::gridToPoint(const GridIndex & idx) const
{
  const auto point = gridToWorld(idx);
  return Point3D{point.x(), point.y(), point.z()};
}

bool GlobalPlannerCore::isInsideMetricBounds(const GridIndex & idx) const
{
  if (!octree_) {
    return false;
  }

  double min_x{};
  double min_y{};
  double min_z{};
  double max_x{};
  double max_y{};
  double max_z{};
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);
  const auto point = gridToWorld(idx);
  return point.x() >= static_cast<float>(min_x) && point.x() <= static_cast<float>(max_x) &&
         point.y() >= static_cast<float>(min_y) && point.y() <= static_cast<float>(max_y) &&
         point.z() >= static_cast<float>(min_z) && point.z() <= static_cast<float>(max_z);
}

bool GlobalPlannerCore::isOccupiedCell(const GridIndex & idx) const
{
  if (!isInsideMetricBounds(idx)) {
    return false;
  }

  const auto point = gridToWorld(idx);
  const octomap::OcTreeNode * node = octree_->search(point);
  return node && octree_->isNodeOccupied(node);
}

bool GlobalPlannerCore::hasGroundSupport(const GridIndex & idx) const
{
  if (options_.strict_direct_ground_support) {
    const GridIndex below{idx.x, idx.y, idx.z - 1};
    if (!isInsideMetricBounds(below)) {
      return false;
    }
    return isOccupiedCell(below);
  }

  const int depth_cells = std::max(1, options_.ground_support_depth_cells);
  for (int dz = 1; dz <= depth_cells; ++dz) {
    for (int dx = -options_.ground_support_xy_radius_cells;
      dx <= options_.ground_support_xy_radius_cells;
      ++dx)
    {
      for (int dy = -options_.ground_support_xy_radius_cells;
        dy <= options_.ground_support_xy_radius_cells;
        ++dy)
      {
        const GridIndex below{idx.x + dx, idx.y + dy, idx.z - dz};
        if (isInsideMetricBounds(below) && isOccupiedCell(below)) {
          return true;
        }
      }
    }
  }
  return false;
}

bool GlobalPlannerCore::hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const
{
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const GridIndex neighbor{idx.x + dx, idx.y + dy, idx.z};
      if (isInsideMetricBounds(neighbor) && !isOccupiedCell(neighbor)) {
        return true;
      }
    }
  }
  return false;
}

bool GlobalPlannerCore::hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const
{
  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      if (dx == 0 && dy == 0) {
        continue;
      }
      const GridIndex above_neighbor{idx.x + dx, idx.y + dy, idx.z + 1};
      if (isInsideMetricBounds(above_neighbor) && isOccupiedCell(above_neighbor)) {
        return true;
      }
    }
  }
  return false;
}

bool GlobalPlannerCore::isCellTraversable(const GridIndex & idx) const
{
  if (!isInsideMetricBounds(idx) || isOccupiedCell(idx)) {
    return false;
  }

  if (options_.require_ground_support && !hasGroundSupport(idx)) {
    return false;
  }

  double min_x{};
  double min_y{};
  double min_z{};
  octree_->getMetricMin(min_x, min_y, min_z);
  const GridIndex min_idx = worldToGrid(min_x, min_y, min_z);
  for (int z = idx.z - 1; z >= min_idx.z; --z) {
    const GridIndex below{idx.x, idx.y, z};
    if (isOccupiedCell(below)) {
      break;
    }
    if (preblocked_cells_.find(below) != preblocked_cells_.end()) {
      return false;
    }
  }

  const octomap::point3d center = gridToWorld(idx);
  const double resolution = octree_->getResolution();
  const int radius_cells = std::max(1, static_cast<int>(std::ceil(options_.robot_radius / resolution)));
  const double radius_sq = options_.robot_radius * options_.robot_radius;

  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dz = 0; dz <= radius_cells; ++dz) {
        const double dist_x = static_cast<double>(dx) * resolution;
        const double dist_y = static_cast<double>(dy) * resolution;
        const double dist_z = static_cast<double>(dz) * resolution;
        if (dist_x * dist_x + dist_y * dist_y + dist_z * dist_z > radius_sq) {
          continue;
        }

        const octomap::point3d point(
          center.x() + static_cast<float>(dx * resolution),
          center.y() + static_cast<float>(dy * resolution),
          center.z() + static_cast<float>(dz * resolution));
        const GridIndex nearby = worldToGrid(point.x(), point.y(), point.z());
        if (preblocked_cells_.find(nearby) != preblocked_cells_.end()) {
          return false;
        }

        const octomap::OcTreeNode * node = octree_->search(point);
        if (node && octree_->isNodeOccupied(node)) {
          return false;
        }
      }
    }
  }
  return true;
}

bool GlobalPlannerCore::findNearestFreeCell(const GridIndex & seed, GridIndex & out) const
{
  if (isCellTraversable(seed)) {
    out = seed;
    return true;
  }

  for (int radius = 1; radius <= options_.snap_search_radius_cells; ++radius) {
    for (int dz = 0; dz <= radius; ++dz) {
      for (int dx = -radius; dx <= radius; ++dx) {
        for (int dy = -radius; dy <= radius; ++dy) {
          if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != radius) {
            continue;
          }

          const GridIndex upper{seed.x + dx, seed.y + dy, seed.z + dz};
          if (isCellTraversable(upper)) {
            out = upper;
            return true;
          }

          if (dz > 0) {
            const GridIndex lower{seed.x + dx, seed.y + dy, seed.z - dz};
            if (isCellTraversable(lower)) {
              out = lower;
              return true;
            }
          }
        }
      }
    }
  }
  return false;
}

std::vector<GridIndex> GlobalPlannerCore::make26Directions() const
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

double GlobalPlannerCore::euclidean(const GridIndex & lhs, const GridIndex & rhs) const
{
  const double dx = static_cast<double>(lhs.x - rhs.x);
  const double dy = static_cast<double>(lhs.y - rhs.y);
  const double dz = static_cast<double>(lhs.z - rhs.z);
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double GlobalPlannerCore::getPreblockedCost(const GridIndex & idx) const
{
  const auto cost_it = preblocked_costmap_.find(idx);
  if (cost_it == preblocked_costmap_.end()) {
    return 0.0;
  }
  return cost_it->second;
}

}  // namespace global_planner
