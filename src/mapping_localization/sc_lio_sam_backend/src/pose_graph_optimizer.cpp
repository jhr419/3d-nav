#include "sc_lio_sam_backend/pose_graph_optimizer.hpp"

#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <stdexcept>

namespace sc_lio_sam_backend
{

namespace
{
gtsam::ISAM2 makeIsam()
{
  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 1;
  return gtsam::ISAM2(parameters);
}
}  // namespace

PoseGraphOptimizer::PoseGraphOptimizer(const PoseGraphConfig & config)
: config_(config),
  isam_(makeIsam())
{
}

void PoseGraphOptimizer::configure(const PoseGraphConfig & config)
{
  config_ = config;
  reset();
}

void PoseGraphOptimizer::reset()
{
  isam_ = makeIsam();
  graph_.resize(0);
  initial_estimate_ = gtsam::Values();
  current_estimate_ = gtsam::Values();
}

void PoseGraphOptimizer::addPriorFactor(int id, const Eigen::Isometry3d & pose)
{
  graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
    static_cast<gtsam::Key>(id),
    toGtsamPose(pose),
    diagonalNoise(config_.prior_noise)));
  initial_estimate_.insert(static_cast<gtsam::Key>(id), toGtsamPose(pose));
}

void PoseGraphOptimizer::addOdometryFactor(
  int from_id,
  int to_id,
  const Eigen::Isometry3d & relative_pose,
  const Eigen::Isometry3d & initial_pose)
{
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
    static_cast<gtsam::Key>(from_id),
    static_cast<gtsam::Key>(to_id),
    toGtsamPose(relative_pose),
    diagonalNoise(config_.odom_noise)));
  initial_estimate_.insert(static_cast<gtsam::Key>(to_id), toGtsamPose(initial_pose));
}

void PoseGraphOptimizer::addLoopFactor(
  int from_id,
  int to_id,
  const Eigen::Isometry3d & relative_pose)
{
  graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
    static_cast<gtsam::Key>(from_id),
    static_cast<gtsam::Key>(to_id),
    toGtsamPose(relative_pose),
    loopNoise()));
}

bool PoseGraphOptimizer::update()
{
  if (graph_.empty() && initial_estimate_.empty()) {
    return false;
  }

  gtsam::ISAM2UpdateParams update_params;
  isam_.update(graph_, initial_estimate_, update_params);
  current_estimate_ = isam_.calculateEstimate();
  graph_.resize(0);
  initial_estimate_ = gtsam::Values();
  return true;
}

bool PoseGraphOptimizer::hasPose(int id) const
{
  return current_estimate_.exists(static_cast<gtsam::Key>(id));
}

Eigen::Isometry3d PoseGraphOptimizer::pose(int id) const
{
  const auto key = static_cast<gtsam::Key>(id);
  if (!current_estimate_.exists(key)) {
    throw std::out_of_range("pose graph does not contain requested key");
  }
  return fromGtsamPose(current_estimate_.at<gtsam::Pose3>(key));
}

gtsam::Pose3 PoseGraphOptimizer::toGtsamPose(const Eigen::Isometry3d & pose)
{
  return gtsam::Pose3(
    gtsam::Rot3(pose.rotation()),
    gtsam::Point3(pose.translation().x(), pose.translation().y(), pose.translation().z()));
}

Eigen::Isometry3d PoseGraphOptimizer::fromGtsamPose(const gtsam::Pose3 & pose)
{
  Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();
  eigen_pose.linear() = pose.rotation().matrix();
  eigen_pose.translation() =
    Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
  return eigen_pose;
}

gtsam::noiseModel::Diagonal::shared_ptr PoseGraphOptimizer::diagonalNoise(
  const std::array<double, 6> & sigmas) const
{
  gtsam::Vector vector(6);
  for (std::size_t i = 0; i < sigmas.size(); ++i) {
    vector(static_cast<int>(i)) = sigmas[i];
  }
  return gtsam::noiseModel::Diagonal::Sigmas(vector);
}

gtsam::SharedNoiseModel PoseGraphOptimizer::loopNoise() const
{
  auto base_noise = diagonalNoise(config_.loop_noise);
  if (!config_.use_robust_loop_noise) {
    return base_noise;
  }

  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(config_.robust_loop_kernel_size),
    base_noise);
}

}  // namespace sc_lio_sam_backend
