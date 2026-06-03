#ifndef JIE_PLANNING__GLOBAL_PLANNER_3D_HPP_
#define JIE_PLANNING__GLOBAL_PLANNER_3D_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

#include "jie_planning/octomap_voxel_map.hpp"

namespace jie_planning
{

class GlobalPlanner3D
{
public:
  struct Options
  {
    int max_iterations{500000};
    bool enable_path_smoothing{true};
    std::string smoothing_method{"line_of_sight"};
    double path_resample_resolution{0.2};
  };

  explicit GlobalPlanner3D(const OctomapVoxelMap & map);
  GlobalPlanner3D(const OctomapVoxelMap & map, Options options);

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

}  // namespace jie_planning

#endif  // JIE_PLANNING__GLOBAL_PLANNER_3D_HPP_
