#ifndef GLOBAL_PLANNER__GLOBAL_PLANNER_CORE_HPP_
#define GLOBAL_PLANNER__GLOBAL_PLANNER_CORE_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "global_planner/octomap_voxel_map.hpp"

namespace global_planner
{

class GlobalPlannerCore
{
public:
  struct Options
  {
    int max_iterations{500000};
    bool enable_path_smoothing{true};
    std::string smoothing_method{"line_of_sight"};
    double path_resample_resolution{0.2};
  };

  explicit GlobalPlannerCore(const OctomapVoxelMap & map);
  GlobalPlannerCore(const OctomapVoxelMap & map, Options options);

  void setOptions(const Options & options);

  bool plan(
    const Eigen::Vector3d & start,
    const Eigen::Vector3d & goal,
    std::vector<Eigen::Vector3d> & path);

  const std::string & lastFailureReason() const;
  int lastIterations() const;

private:
  struct QueueNode
  {
    GridIndex index;
    double f;
    double g;
  };

  struct QueueNodeCompare
  {
    bool operator()(const QueueNode & lhs, const QueueNode & rhs) const
    {
      return lhs.f > rhs.f;
    }
  };

  bool isTraversable(const GridIndex & index) const;
  double gridDistance(const GridIndex & lhs, const GridIndex & rhs) const;
  std::vector<GridIndex> reconstructPath(
    const std::unordered_map<GridIndex, GridIndex, GridIndexHash> & came_from,
    GridIndex current) const;
  std::vector<Eigen::Vector3d> toWorldPath(const std::vector<GridIndex> & cells) const;
  bool hasLineOfSight(const Eigen::Vector3d & from, const Eigen::Vector3d & to) const;
  void smoothPath(std::vector<Eigen::Vector3d> & path) const;
  void resamplePath(std::vector<Eigen::Vector3d> & path) const;
  void setFailure(const std::string & reason);

  const OctomapVoxelMap & map_;
  Options options_;
  std::string last_failure_reason_;
  int last_iterations_{0};
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__GLOBAL_PLANNER_CORE_HPP_
