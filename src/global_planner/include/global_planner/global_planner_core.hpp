#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <octomap/OcTree.h>

namespace global_planner
{

struct GridIndex
{
  int x{};
  int y{};
  int z{};

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & key) const;
};

struct Point3D
{
  double x{};
  double y{};
  double z{};
};

struct PlannerOptions
{
  double robot_radius{0.20};
  int max_iterations{250000};
  int snap_search_radius_cells{8};
  bool require_ground_support{true};
  bool strict_direct_ground_support{true};
  int ground_support_xy_radius_cells{1};
  int ground_support_depth_cells{2};
  bool enable_preblocked_costmap{true};
  int preblocked_costmap_radius_cells{3};
  double preblocked_costmap_weight{1.5};
  bool lowest_traversable_only{false};
};

struct PlanResult
{
  bool success{false};
  std::string message;
  std::vector<Point3D> path;
  int iterations{0};
  bool snapped_start{false};
  bool snapped_goal{false};
  Point3D start;
  Point3D goal;
};

class GlobalPlannerCore
{
public:
  explicit GlobalPlannerCore(const PlannerOptions & options = PlannerOptions{});

  void setOptions(const PlannerOptions & options);
  const PlannerOptions & options() const;

  void setOctomap(std::shared_ptr<octomap::OcTree> map);
  bool hasMap() const;

  PlanResult makePlan(const Point3D & start, const Point3D & goal);

  const std::unordered_set<GridIndex, GridIndexHash> & traversableCells() const;
  const std::unordered_set<GridIndex, GridIndexHash> & preblockedCells() const;

private:
  struct QueueNode
  {
    GridIndex idx;
    double f{};
    double g{};
  };

  struct QueueNodeCompare
  {
    bool operator()(const QueueNode & lhs, const QueueNode & rhs) const;
  };

  void rebuildLayers();
  void rebuildPreblockedCells();
  void rebuildTraversableCells();
  void rebuildPreblockedCostmap();

  GridIndex worldToGrid(double x, double y, double z) const;
  octomap::point3d gridToWorld(const GridIndex & idx) const;
  Point3D gridToPoint(const GridIndex & idx) const;

  bool isInsideMetricBounds(const GridIndex & idx) const;
  bool isOccupiedCell(const GridIndex & idx) const;
  bool hasGroundSupport(const GridIndex & idx) const;
  bool hasNonOccupiedNeighborSameLevel(const GridIndex & idx) const;
  bool hasSameLevelNeighborWithOccupiedAbove(const GridIndex & idx) const;
  bool isCellTraversable(const GridIndex & idx) const;

  bool findNearestFreeCell(const GridIndex & seed, GridIndex & out) const;
  std::vector<GridIndex> make26Directions() const;
  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const;

  double euclidean(const GridIndex & lhs, const GridIndex & rhs) const;
  double getPreblockedCost(const GridIndex & idx) const;

  PlannerOptions options_;
  std::shared_ptr<octomap::OcTree> octree_;
  std::unordered_set<GridIndex, GridIndexHash> traversable_cells_;
  std::unordered_set<GridIndex, GridIndexHash> preblocked_cells_;
  std::unordered_map<GridIndex, double, GridIndexHash> preblocked_costmap_;
};

}  // namespace global_planner
