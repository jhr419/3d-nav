#pragma once

#include "sc_lio_sam_backend/backend_types.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <array>

namespace sc_lio_sam_backend
{

struct PoseGraphConfig
{
  std::array<double, 6> prior_noise{{1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2}};
  std::array<double, 6> odom_noise{{1e-2, 1e-2, 1e-2, 5e-2, 5e-2, 5e-2}};
  std::array<double, 6> loop_noise{{5e-2, 5e-2, 5e-2, 2e-1, 2e-1, 2e-1}};
  bool use_robust_loop_noise = true;
  double robust_loop_kernel_size = 1.0;
};

class PoseGraphOptimizer
{
public:
  explicit PoseGraphOptimizer(const PoseGraphConfig & config = PoseGraphConfig());

  void configure(const PoseGraphConfig & config);
  void reset();
  void addPriorFactor(int id, const Eigen::Isometry3d & pose);
  void addOdometryFactor(
    int from_id,
    int to_id,
    const Eigen::Isometry3d & relative_pose,
    const Eigen::Isometry3d & initial_pose);
  void addLoopFactor(int from_id, int to_id, const Eigen::Isometry3d & relative_pose);
  bool update();
  bool hasPose(int id) const;
  Eigen::Isometry3d pose(int id) const;

  static gtsam::Pose3 toGtsamPose(const Eigen::Isometry3d & pose);
  static Eigen::Isometry3d fromGtsamPose(const gtsam::Pose3 & pose);

private:
  gtsam::noiseModel::Diagonal::shared_ptr diagonalNoise(const std::array<double, 6> & sigmas) const;
  gtsam::SharedNoiseModel loopNoise() const;

  PoseGraphConfig config_;
  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  gtsam::Values current_estimate_;
};

}  // namespace sc_lio_sam_backend
