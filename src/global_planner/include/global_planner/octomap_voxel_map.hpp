#ifndef GLOBAL_PLANNER__OCTOMAP_VOXEL_MAP_HPP_
#define GLOBAL_PLANNER__OCTOMAP_VOXEL_MAP_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>

#include "octomap/OcTree.h"

namespace global_planner
{

struct GridIndex
{
  int x;
  int y;
  int z;

  bool operator==(const GridIndex & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct GridIndexHash
{
  std::size_t operator()(const GridIndex & key) const;
};

class OctomapVoxelMap
{
public:
  bool load(
    const std::string & map_file,
    double fallback_resolution,
    double inflation_radius,
    bool unknown_as_occupied,
    std::string * error_message = nullptr);

  bool setTree(
    std::shared_ptr<octomap::OcTree> tree,
    double inflation_radius,
    bool unknown_as_occupied,
    std::string * error_message = nullptr);

  bool isReady() const;
  bool isFree(const Eigen::Vector3d & point) const;
  bool isOccupied(const Eigen::Vector3d & point) const;
  bool isInBounds(const Eigen::Vector3d & point) const;
  bool isGridInBounds(const GridIndex & index) const;

  double getResolution() const;
  double getInflationRadius() const;
  bool unknownAsOccupied() const;
  const Eigen::Vector3d & mapMin() const;
  const Eigen::Vector3d & mapMax() const;
  GridIndex minIndex() const;
  GridIndex maxIndex() const;
  std::size_t inflatedCellCount() const;

  GridIndex worldToGrid(const Eigen::Vector3d & point) const;
  Eigen::Vector3d gridToWorld(const GridIndex & index) const;
  std::vector<Eigen::Vector3d> occupiedVoxelCenters() const;

  std::shared_ptr<octomap::OcTree> tree();
  std::shared_ptr<const octomap::OcTree> tree() const;

private:
  void clear();
  bool updateBounds(std::string * error_message);
  void rebuildInflation();
  bool isInflatedOccupied(const GridIndex & index) const;
  bool isRawOccupied(const Eigen::Vector3d & point) const;

  std::shared_ptr<octomap::OcTree> tree_;
  Eigen::Vector3d map_min_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d map_max_{Eigen::Vector3d::Zero()};
  GridIndex min_index_{0, 0, 0};
  GridIndex max_index_{0, 0, 0};
  double inflation_radius_{0.0};
  bool unknown_as_occupied_{true};
  std::unordered_set<GridIndex, GridIndexHash> inflated_occupied_;
};

}  // namespace global_planner

#endif  // GLOBAL_PLANNER__OCTOMAP_VOXEL_MAP_HPP_
