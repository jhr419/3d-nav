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
  double min_dynamic_obstacle_distance{std::numeric_limits<double>::infinity()};
  bool collision_free{false};
};

struct DwaDebugInfo
{
  int valid_trajectories{0};
  int collision_trajectories{0};
  int unknown_blocked_count{0};
  int ground_blocked_count{0};
  double best_score{-std::numeric_limits<double>::infinity()};
  double dynamic_obstacle_speed_scale{1.0};
  double nearest_dynamic_obstacle_distance{std::numeric_limits<double>::infinity()};
  std::string recovery_state{"idle"};
};

class DWA3DLocalPlanner
{
public:
  struct Config
  {
    std::string robot_model{"ground_omni"};
    std::string obstacle_source{"both"};
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
    double weight_dynamic_obstacle_distance{2.0};
    double obstacle_check_resolution{0.1};
    double min_obstacle_distance{0.25};
    double obstacle_score_distance{2.0};
    bool stop_on_no_valid_trajectory{true};

    bool enable_dynamic_speed_scaling{true};
    bool dynamic_obstacle_use_2d_footprint{true};
    double dynamic_obstacle_radius{0.35};
    double dynamic_obstacle_safety_margin{0.15};
    double dynamic_obstacle_stop_distance{0.35};
    double dynamic_obstacle_slow_distance{0.8};
    double min_speed_scale{0.2};

    bool use_path_z_for_collision{true};
    bool terrain_following_enabled{true};
    double z_search_radius{0.6};
    double max_allowed_z_jump{0.35};
    double slope_edge_z_tolerance{0.45};

    std::string collision_model{"terrain_adaptive_cylinder"};
    double ground_clearance{0.05};
    double body_z_offset{0.10};
    bool ignore_ground_below_base{true};
    double ground_ignore_depth{0.15};
    bool slope_edge_relaxation_enabled{true};
    double slope_edge_relaxation_radius{0.25};

    std::string unknown_policy{"path_corridor_free"};
    double path_corridor_radius{0.5};

    bool adaptive_lookahead_enabled{true};
    double min_local_goal_lookahead{0.4};
    double max_local_goal_lookahead{1.8};
    double z_change_slowdown_threshold{0.25};

    bool recovery_enabled{true};
    bool stuck_detection_enabled{true};
    double stuck_time_threshold{2.0};
    double min_progress_distance{0.05};
    double recovery_duration{1.0};
    bool enable_reverse_escape{true};
    bool enable_lateral_escape{true};
    bool enable_rotate_escape{true};

    bool near_path_bonus_enabled{true};
    double near_path_bonus_radius{0.4};
    double weight_path_corridor{1.0};
    double weight_z_consistency{0.8};
    double weight_progress{1.0};

    bool debug_dwa{true};
    bool debug_collision{true};
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
  double getMinDynamicObstacleDistance(const std::vector<Pose3D> & traj) const;
  double computeDynamicObstacleSpeedScale(const Pose3D & current_pose) const;
  bool isPoseCollisionFreeTerrainAdaptive(const Pose3D & pose) const;
  bool isUnknownAllowed(const Eigen::Vector3d & point) const;
  double getReferenceZFromPath(double x, double y) const;
  double getInterpolatedPathZ(double x, double y) const;
  bool computeRecoveryCommand(Velocity3D & cmd_vel);
  bool computeRecoveryCommand(Velocity3D & cmd_vel, Trajectory3D & recovery_traj);

  const std::vector<Trajectory3D> & getLastCandidateTrajectories() const;
  const Eigen::Vector3d & getLastLocalGoal() const;
  const Trajectory3D & getLastRecoveryTrajectory() const;
  const DwaDebugInfo & getLastDebugInfo() const;

private:
  enum class CollisionReason
  {
    kNone,
    kOccupied,
    kUnknown,
    kGround
  };

  std::vector<double> sampleRange(double min_value, double max_value, int samples) const;
  Trajectory3D simulateTrajectory(const Pose3D & start, const Velocity3D & cmd) const;
  Trajectory3D simulateTrajectoryForDuration(
    const Pose3D & start,
    const Velocity3D & cmd,
    double duration) const;
  Eigen::Vector3d selectLocalGoal(
    const Pose3D & current_pose,
    const std::vector<Eigen::Vector3d> & global_path) const;
  double distanceToPath(
    const Eigen::Vector3d & point,
    const std::vector<Eigen::Vector3d> & global_path) const;
  double planarDistanceToPath(
    const Eigen::Vector3d & point,
    const std::vector<Eigen::Vector3d> & global_path) const;
  std::size_t nearestPathIndex2D(
    const Eigen::Vector3d & point,
    const std::vector<Eigen::Vector3d> & global_path) const;
  double scoreTrajectory(
    const Trajectory3D & traj,
    const Pose3D & current_pose,
    const std::vector<Eigen::Vector3d> & global_path,
    const Eigen::Vector3d & local_goal) const;
  bool evaluateCollisionAndDistance(
    const std::vector<Pose3D> & traj,
    double & min_obstacle_distance,
    DwaDebugInfo * debug_info = nullptr,
    double * min_dynamic_obstacle_distance = nullptr) const;
  double minPointCloudDistance(const Pose3D & pose) const;
  double minForwardPointCloudDistance(const Pose3D & pose) const;
  double minOctomapDistance(const Pose3D & pose) const;
  bool octomapFootprintCollision(const Pose3D & pose) const;
  bool isPoseCollisionFreeTerrainAdaptive(
    const Pose3D & pose,
    CollisionReason * reason) const;
  bool useOctomap() const;
  bool usePointCloud() const;
  double collisionDistanceThreshold() const;
  double dynamicCollisionDistanceThreshold() const;
  double maxPlanarSpeed() const;
  double collisionBodyMinZ(const Pose3D & pose) const;
  double collisionBodyMaxZ(const Pose3D & pose) const;
  bool isNearActivePath2D(const Pose3D & pose, double radius) const;

  Config config_;
  std::shared_ptr<const octomap::OcTree> octree_;
  std::vector<Eigen::Vector3d> obstacle_cloud_;
  Velocity3D last_cmd_;
  std::vector<Trajectory3D> last_candidates_;
  Eigen::Vector3d last_local_goal_{0.0, 0.0, 0.0};
  std::vector<Eigen::Vector3d> active_global_path_;
  Pose3D active_current_pose_;
  bool has_active_context_{false};
  Trajectory3D last_recovery_traj_;
  DwaDebugInfo last_debug_info_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING_DWA_3D_LOCAL_PLANNER_HPP_
