#include "sc_lio_sam_backend/keyframe_manager.hpp"

namespace sc_lio_sam_backend
{

void KeyframeManager::configure(double distance_threshold, double angle_threshold_deg)
{
  distance_threshold_ = distance_threshold;
  angle_threshold_deg_ = angle_threshold_deg;
}

bool KeyframeManager::shouldCreateKeyframe(const Eigen::Isometry3d & pose) const
{
  if (keyframes_.empty()) {
    return true;
  }

  const auto & last_pose = keyframes_.back().raw_pose;
  return translationDistance(last_pose, pose) >= distance_threshold_ ||
         rotationAngleDeg(last_pose, pose) >= angle_threshold_deg_;
}

int KeyframeManager::addKeyframe(
  const builtin_interfaces::msg::Time & stamp,
  double stamp_sec,
  const Eigen::Isometry3d & raw_pose,
  const PointCloudPtr & cloud,
  const ScanContextDescriptor & descriptor)
{
  Keyframe keyframe;
  keyframe.id = static_cast<int>(keyframes_.size());
  keyframe.stamp = stamp;
  keyframe.stamp_sec = stamp_sec;
  keyframe.raw_pose = raw_pose;
  keyframe.optimized_pose = raw_pose;
  keyframe.cloud = cloud;
  keyframe.scan_context = descriptor;
  keyframes_.push_back(keyframe);
  return keyframe.id;
}

std::vector<Keyframe> & KeyframeManager::keyframes()
{
  return keyframes_;
}

const std::vector<Keyframe> & KeyframeManager::keyframes() const
{
  return keyframes_;
}

bool KeyframeManager::empty() const
{
  return keyframes_.empty();
}

std::size_t KeyframeManager::size() const
{
  return keyframes_.size();
}

}  // namespace sc_lio_sam_backend
