#ifndef LOCAL_PLANNING_DWA_3D_LOCAL_PLANNER_HPP_
#define LOCAL_PLANNING_DWA_3D_LOCAL_PLANNER_HPP_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <octomap/OcTree.h>

namespace local_planning
{

struct Pose3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw{0.0};
};

struct Velocity3D
{
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double wx{0.0};
  double wy{0.0};
  double wz{0.0};
};

struct Trajectory3D
{
  std::vector<Pose3D> poses;
  Velocity3D cmd;
  double score{0.0};
  double min_obstacle_distance{std::numeric_limits<double>::infinity()};
  bool collision_free{false};
};

class DWA3DLocalPlanner
{
public:
  struct Config
  {
    std::string robot_model{"ground_omni"};
    std::string obstacle_source{"octomap"};
    bool unknown_as_occupied{true};
    double robot_radius{0.35};
    double robot_height{0.6};
    double collision_z_offset{0.10};
    double safety_margin{0.15};
    double control_frequency{10.0};
    double sim_time{2.0};
    double sim_dt{0.1};
    double max_vx{0.6};
    double min_vx{-0.2};
    double max_vy{0.4};
    double min_vy{-0.4};
    double max_wz{1.0};
    double min_wz{-1.0};
    double max_acc_vx{0.5};
    double max_acc_vy{0.5};
    double max_acc_wz{1.5};
    int vx_samples{9};
    int vy_samples{9};
    int wz_samples{15};
    double local_goal_lookahead{1.5};
    double goal_reached_tolerance{0.3};
    double path_prune_distance{0.5};
    double weight_path_distance{1.0};
    double weight_goal_distance{1.2};
    double weight_obstacle_distance{1.5};
    double weight_heading{0.5};
    double weight_velocity{0.2};
    double weight_smoothness{0.2};
    double obstacle_check_resolution{0.1};
    double min_obstacle_distance{0.25};
    double obstacle_score_distance{2.0};
    bool stop_on_no_valid_trajectory{true};
  };

  DWA3DLocalPlanner();
  explicit DWA3DLocalPlanner(const Config & config);

  void setConfig(const Config & config);
  const Config & getConfig() const;

  void setOctomap(std::shared_ptr<const octomap::OcTree> octree);
  bool hasOctomap() const;

  void setObstacleCloud(const std::vector<Eigen::Vector3d> & points);
  void clearObstacleCloud();
  bool hasObstacleCloud() const;

  bool computeVelocityCommand(
    const Pose3D & current_pose,
    const Velocity3D & current_vel,
    const std::vector<Eigen::Vector3d> & global_path,
    Velocity3D & cmd_vel,
    Trajectory3D & best_traj);

  bool isTrajectoryCollisionFree(const std::vector<Pose3D> & traj) const;
  double getMinObstacleDistance(const std::vector<Pose3D> & traj) const;

  const std::vector<Trajectory3D> & getLastCandidateTrajectories() const;
  const Eigen::Vector3d & getLastLocalGoal() const;

private:
  std::vector<double> sampleRange(double min_value, double max_value, int samples) const;
  Trajectory3D simulateTrajectory(const Pose3D & start, const Velocity3D & cmd) const;
  Eigen::Vector3d selectLocalGoal(
    const Pose3D & current_pose,
    const std::vector<Eigen::Vector3d> & global_path) const;
  double distanceToPath(
    const Eigen::Vector3d & point,
    const std::vector<Eigen::Vector3d> & global_path) const;
  double scoreTrajectory(
    const Trajectory3D & traj,
    const Pose3D & current_pose,
    const std::vector<Eigen::Vector3d> & global_path,
    const Eigen::Vector3d & local_goal) const;
  bool evaluateCollisionAndDistance(
    const std::vector<Pose3D> & traj,
    double & min_obstacle_distance) const;
  double minPointCloudDistance(const Pose3D & pose) const;
  double minOctomapDistance(const Pose3D & pose) const;
  bool octomapFootprintCollision(const Pose3D & pose) const;
  bool useOctomap() const;
  bool usePointCloud() const;
  double collisionDistanceThreshold() const;
  double maxPlanarSpeed() const;

  Config config_;
  std::shared_ptr<const octomap::OcTree> octree_;
  std::vector<Eigen::Vector3d> obstacle_cloud_;
  Velocity3D last_cmd_;
  std::vector<Trajectory3D> last_candidates_;
  Eigen::Vector3d last_local_goal_{0.0, 0.0, 0.0};
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_DWA_3D_LOCAL_PLANNER_HPP_
