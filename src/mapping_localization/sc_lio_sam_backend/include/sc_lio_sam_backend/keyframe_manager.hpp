#pragma once

#include "sc_lio_sam_backend/backend_types.hpp"

#include <vector>

namespace sc_lio_sam_backend
{

class KeyframeManager
{
public:
  KeyframeManager() = default;

  void configure(double distance_threshold, double angle_threshold_deg);
  bool shouldCreateKeyframe(const Eigen::Isometry3d & pose) const;
  int addKeyframe(
    const builtin_interfaces::msg::Time & stamp,
    double stamp_sec,
    const Eigen::Isometry3d & raw_pose,
    const PointCloudPtr & cloud,
    const ScanContextDescriptor & descriptor);

  std::vector<Keyframe> & keyframes();
  const std::vector<Keyframe> & keyframes() const;
  bool empty() const;
  std::size_t size() const;

private:
  double distance_threshold_ = 1.0;
  double angle_threshold_deg_ = 10.0;
  std::vector<Keyframe> keyframes_;
};

}  // namespace sc_lio_sam_backend
