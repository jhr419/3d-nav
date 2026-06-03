#include "global_planner/octomap_voxel_map.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "octomap/AbstractOcTree.h"

namespace global_planner
{

namespace
{
template<typename T>
void hashCombine(std::size_t & seed, const T & value)
{
  seed ^= std::hash<T>{}(value) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

void setError(std::string * error_message, const std::string & text)
{
  if (error_message) {
    *error_message = text;
  }
}
}  // namespace

std::size_t GridIndexHash::operator()(const GridIndex & key) const
{
  std::size_t seed = 0;
  hashCombine(seed, key.x);
  hashCombine(seed, key.y);
  hashCombine(seed, key.z);
  return seed;
}

bool OctomapVoxelMap::load(
  const std::string & map_file,
  double fallback_resolution,
  double inflation_radius,
  bool unknown_as_occupied,
  std::string * error_message)
{
  if (map_file.empty()) {
    setError(error_message, "map_file is empty");
    clear();
    return false;
  }

  const double resolution = fallback_resolution > 0.0 ? fallback_resolution : 0.2;
  auto binary_tree = std::make_shared<octomap::OcTree>(resolution);
  if (binary_tree->readBinary(map_file)) {
    return setTree(binary_tree, inflation_radius, unknown_as_occupied, error_message);
  }

  std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap::AbstractOcTree::read(map_file));
  if (abstract_tree) {
    auto * octree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (octree) {
      abstract_tree.release();
      return setTree(
        std::shared_ptr<octomap::OcTree>(octree), inflation_radius, unknown_as_occupied,
        error_message);
    }
  }

  std::ostringstream oss;
  oss << "failed to load OctoMap file: " << map_file;
  setError(error_message, oss.str());
  clear();
  return false;
}

bool OctomapVoxelMap::setTree(
  std::shared_ptr<octomap::OcTree> tree,
  double inflation_radius,
  bool unknown_as_occupied,
  std::string * error_message)
{
  if (!tree) {
    setError(error_message, "OctoMap tree is null");
    clear();
    return false;
  }

  tree_ = std::move(tree);
  inflation_radius_ = std::max(0.0, inflation_radius);
  unknown_as_occupied_ = unknown_as_occupied;

  if (!updateBounds(error_message)) {
    clear();
    return false;
  }

  rebuildInflation();
  return true;
}

bool OctomapVoxelMap::isReady() const
{
  return static_cast<bool>(tree_);
}

bool OctomapVoxelMap::isFree(const Eigen::Vector3d & point) const
{
  return isInBounds(point) && !isOccupied(point);
}

bool OctomapVoxelMap::isOccupied(const Eigen::Vector3d & point) const
{
  if (!tree_) {
    return true;
  }
  if (!isInBounds(point)) {
    return true;
  }
  const GridIndex index = worldToGrid(point);
  if (isInflatedOccupied(index)) {
    return true;
  }
  return isRawOccupied(point);
}

bool OctomapVoxelMap::isInBounds(const Eigen::Vector3d & point) const
{
  if (!tree_) {
    return false;
  }

  return point.x() >= map_min_.x() && point.x() <= map_max_.x() &&
         point.y() >= map_min_.y() && point.y() <= map_max_.y() &&
         point.z() >= map_min_.z() && point.z() <= map_max_.z();
}

bool OctomapVoxelMap::isGridInBounds(const GridIndex & index) const
{
  if (!tree_) {
    return false;
  }

  return index.x >= min_index_.x && index.x <= max_index_.x &&
         index.y >= min_index_.y && index.y <= max_index_.y &&
         index.z >= min_index_.z && index.z <= max_index_.z;
}

double OctomapVoxelMap::getResolution() const
{
  return tree_ ? tree_->getResolution() : 0.0;
}

double OctomapVoxelMap::getInflationRadius() const
{
  return inflation_radius_;
}

bool OctomapVoxelMap::unknownAsOccupied() const
{
  return unknown_as_occupied_;
}

const Eigen::Vector3d & OctomapVoxelMap::mapMin() const
{
  return map_min_;
}

const Eigen::Vector3d & OctomapVoxelMap::mapMax() const
{
  return map_max_;
}

GridIndex OctomapVoxelMap::minIndex() const
{
  return min_index_;
}

GridIndex OctomapVoxelMap::maxIndex() const
{
  return max_index_;
}

std::size_t OctomapVoxelMap::inflatedCellCount() const
{
  return inflated_occupied_.size();
}

GridIndex OctomapVoxelMap::worldToGrid(const Eigen::Vector3d & point) const
{
  const double resolution = getResolution();
  return GridIndex{
    static_cast<int>(std::floor(point.x() / resolution)),
    static_cast<int>(std::floor(point.y() / resolution)),
    static_cast<int>(std::floor(point.z() / resolution))};
}

Eigen::Vector3d OctomapVoxelMap::gridToWorld(const GridIndex & index) const
{
  const double resolution = getResolution();
  return Eigen::Vector3d(
    (static_cast<double>(index.x) + 0.5) * resolution,
    (static_cast<double>(index.y) + 0.5) * resolution,
    (static_cast<double>(index.z) + 0.5) * resolution);
}

std::vector<Eigen::Vector3d> OctomapVoxelMap::occupiedVoxelCenters() const
{
  std::vector<Eigen::Vector3d> points;
  if (!tree_) {
    return points;
  }

  points.reserve(tree_->size());
  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
    if (!tree_->isNodeOccupied(*it)) {
      continue;
    }
    points.emplace_back(it.getX(), it.getY(), it.getZ());
  }
  return points;
}

std::shared_ptr<octomap::OcTree> OctomapVoxelMap::tree()
{
  return tree_;
}

std::shared_ptr<const octomap::OcTree> OctomapVoxelMap::tree() const
{
  return tree_;
}

void OctomapVoxelMap::clear()
{
  tree_.reset();
  inflated_occupied_.clear();
  map_min_ = Eigen::Vector3d::Zero();
  map_max_ = Eigen::Vector3d::Zero();
  min_index_ = GridIndex{0, 0, 0};
  max_index_ = GridIndex{0, 0, 0};
}

bool OctomapVoxelMap::updateBounds(std::string * error_message)
{
  if (!tree_) {
    setError(error_message, "OctoMap tree is null");
    return false;
  }

  double min_x = 0.0;
  double min_y = 0.0;
  double min_z = 0.0;
  double max_x = 0.0;
  double max_y = 0.0;
  double max_z = 0.0;
  tree_->getMetricMin(min_x, min_y, min_z);
  tree_->getMetricMax(max_x, max_y, max_z);

  if (max_x <= min_x || max_y <= min_y || max_z <= min_z) {
    setError(error_message, "OctoMap bounds are empty");
    return false;
  }

  map_min_ = Eigen::Vector3d(min_x, min_y, min_z);
  map_max_ = Eigen::Vector3d(max_x, max_y, max_z);
  min_index_ = worldToGrid(map_min_);
  max_index_ = worldToGrid(map_max_);
  return true;
}

void OctomapVoxelMap::rebuildInflation()
{
  inflated_occupied_.clear();
  if (!tree_) {
    return;
  }

  const double resolution = getResolution();
  const int radius_cells = std::max(0, static_cast<int>(std::ceil(inflation_radius_ / resolution)));
  const double radius_sq = inflation_radius_ * inflation_radius_;

  for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
    if (!tree_->isNodeOccupied(*it)) {
      continue;
    }

    const GridIndex occupied_index = worldToGrid(Eigen::Vector3d(it.getX(), it.getY(), it.getZ()));
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
          const double x = static_cast<double>(dx) * resolution;
          const double y = static_cast<double>(dy) * resolution;
          const double z = static_cast<double>(dz) * resolution;
          const double distance_sq = x * x + y * y + z * z;
          if (distance_sq > radius_sq + 1.0e-9) {
            continue;
          }

          const GridIndex inflated_index{
            occupied_index.x + dx, occupied_index.y + dy, occupied_index.z + dz};
          if (isGridInBounds(inflated_index)) {
            inflated_occupied_.insert(inflated_index);
          }
        }
      }
    }
  }
}

bool OctomapVoxelMap::isInflatedOccupied(const GridIndex & index) const
{
  return inflated_occupied_.find(index) != inflated_occupied_.end();
}

bool OctomapVoxelMap::isRawOccupied(const Eigen::Vector3d & point) const
{
  const auto * node = tree_->search(
    static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z()));
  if (!node) {
    return unknown_as_occupied_;
  }
  return tree_->isNodeOccupied(node);
}

}  // namespace global_planner
