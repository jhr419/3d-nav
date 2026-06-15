#pragma once

#include "sc_lio_sam_backend/backend_types.hpp"

#include <utility>

namespace sc_lio_sam_backend
{

struct ScanContextConfig
{
  int ring_num = 20;
  int sector_num = 60;
  double max_radius = 80.0;
  double lidar_height = 2.0;
  int exclude_recent_num = 30;
  double distance_threshold = 0.3;
  double search_ratio = 0.1;
};

struct LoopCandidate
{
  bool valid = false;
  int current_id = -1;
  int candidate_id = -1;
  double scan_context_distance = 0.0;
  double yaw_diff_rad = 0.0;
};

class ScanContextManager
{
public:
  explicit ScanContextManager(const ScanContextConfig & config = ScanContextConfig());

  void configure(const ScanContextConfig & config);
  ScanContextDescriptor makeDescriptor(const PointCloud & cloud) const;
  Eigen::MatrixXd makeRingKey(const ScanContextDescriptor & descriptor) const;
  Eigen::MatrixXd makeSectorKey(const ScanContextDescriptor & descriptor) const;
  std::pair<double, int> distance(const ScanContextDescriptor & query, const ScanContextDescriptor & candidate) const;

  LoopCandidate detectLoopCandidate(
    const std::vector<Keyframe> & keyframes,
    int current_id,
    double loop_search_radius,
    double min_time_diff) const;

private:
  static double xy2thetaDeg(double x, double y);
  static Eigen::MatrixXd circshift(const Eigen::MatrixXd & mat, int num_shift);
  double directDistance(const ScanContextDescriptor & query, const ScanContextDescriptor & candidate) const;
  int fastAlignUsingSectorKey(const Eigen::MatrixXd & query_key, const Eigen::MatrixXd & candidate_key) const;

  ScanContextConfig config_;
};

}  // namespace sc_lio_sam_backend
