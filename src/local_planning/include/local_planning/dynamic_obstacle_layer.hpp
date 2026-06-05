#ifndef LOCAL_PLANNING_DYNAMIC_OBSTACLE_LAYER_HPP_
#define LOCAL_PLANNING_DYNAMIC_OBSTACLE_LAYER_HPP_

#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "local_planning/dwa_3d_local_planner.hpp"

namespace local_planning
{

class DynamicObstacleLayer
{
public:
  struct Config
  {
    std::string pointcloud_frame_mode{"map"};
    double pointcloud_timeout{0.5};
    double local_cloud_range_x{4.0};
    double local_cloud_range_y{3.0};
    double local_cloud_range_z_min{-0.2};
    double local_cloud_range_z_max{1.2};
    double voxel_leaf_size{0.08};
    bool enable_voxel_filter{true};
    bool remove_ground_points{true};
    double ground_z_threshold{0.08};
    bool ground_relative_to_base{true};
    bool ignore_robot_self_points{true};
    double self_filter_radius{0.45};
    double self_filter_z_min{-0.3};
    double self_filter_z_max{1.2};
    double dynamic_obstacle_decay_time{0.5};
    double robot_radius{0.35};
    double robot_height{0.6};
    double safety_margin{0.15};
    double body_z_offset{0.10};
  };

  void setConfig(const Config & config);
  const Config & getConfig() const;
  void setCurrentPose(const Pose3D & pose);

  void updateFromPointCloud(const sensor_msgs::msg::PointCloud2 & msg);
  void clear();

  bool hasCloud() const;
  bool hasObstacles() const;
  bool isFresh() const;
  bool isExpired() const;
  double ageSeconds() const;

  bool isPointCollisionFree(const Eigen::Vector3d & p, double radius, double height) const;
  bool isTrajectoryCollisionFree(const std::vector<Pose3D> & traj) const;
  double getMinDistanceToTrajectory(const std::vector<Pose3D> & traj) const;

  const std::vector<Eigen::Vector3d> & obstaclePoints() const;

private:
  Eigen::Vector3d pointMapToBase(const Eigen::Vector3d & point_map) const;
  Eigen::Vector3d pointBaseToMap(const Eigen::Vector3d & point_base) const;
  bool shouldKeepPoint(
    const Eigen::Vector3d & point_base,
    const Eigen::Vector3d & point_map) const;
  double pointDistanceToBody(
    const Eigen::Vector3d & obstacle,
    const Eigen::Vector3d & body_reference,
    double radius,
    double height) const;

  Config config_;
  Pose3D current_pose_;
  std::vector<Eigen::Vector3d> obstacle_points_;
  std::chrono::steady_clock::time_point last_update_time_;
  bool has_cloud_{false};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_DYNAMIC_OBSTACLE_LAYER_HPP_
