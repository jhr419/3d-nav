#include "sc_lio_sam_backend/scan_context_manager.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sc_lio_sam_backend
{

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;
constexpr double kNoPoint = -1000.0;
}

ScanContextManager::ScanContextManager(const ScanContextConfig & config)
{
  configure(config);
}

void ScanContextManager::configure(const ScanContextConfig & config)
{
  config_ = config;
  config_.ring_num = std::max(1, config_.ring_num);
  config_.sector_num = std::max(1, config_.sector_num);
  config_.max_radius = std::max(1.0, config_.max_radius);
  config_.exclude_recent_num = std::max(0, config_.exclude_recent_num);
  config_.search_ratio = std::clamp(config_.search_ratio, 0.0, 1.0);
}

ScanContextDescriptor ScanContextManager::makeDescriptor(const PointCloud & cloud) const
{
  ScanContextDescriptor descriptor =
    kNoPoint * ScanContextDescriptor::Ones(config_.ring_num, config_.sector_num);

  for (const auto & point : cloud.points) {
    const double range = std::hypot(point.x, point.y);
    if (range > config_.max_radius || range <= 0.0) {
      continue;
    }

    const double azimuth_deg = xy2thetaDeg(point.x, point.y);
    const int ring_idx = std::clamp(
      static_cast<int>(std::ceil((range / config_.max_radius) * config_.ring_num)),
      1,
      config_.ring_num);
    const int sector_idx = std::clamp(
      static_cast<int>(std::ceil((azimuth_deg / 360.0) * config_.sector_num)),
      1,
      config_.sector_num);

    const double encoded_height = static_cast<double>(point.z) + config_.lidar_height;
    double & cell = descriptor(ring_idx - 1, sector_idx - 1);
    if (cell < encoded_height) {
      cell = encoded_height;
    }
  }

  for (int r = 0; r < descriptor.rows(); ++r) {
    for (int c = 0; c < descriptor.cols(); ++c) {
      if (descriptor(r, c) == kNoPoint) {
        descriptor(r, c) = 0.0;
      }
    }
  }

  return descriptor;
}

Eigen::MatrixXd ScanContextManager::makeRingKey(const ScanContextDescriptor & descriptor) const
{
  Eigen::MatrixXd key(descriptor.rows(), 1);
  for (int r = 0; r < descriptor.rows(); ++r) {
    key(r, 0) = descriptor.row(r).mean();
  }
  return key;
}

Eigen::MatrixXd ScanContextManager::makeSectorKey(const ScanContextDescriptor & descriptor) const
{
  Eigen::MatrixXd key(1, descriptor.cols());
  for (int c = 0; c < descriptor.cols(); ++c) {
    key(0, c) = descriptor.col(c).mean();
  }
  return key;
}

std::pair<double, int> ScanContextManager::distance(
  const ScanContextDescriptor & query,
  const ScanContextDescriptor & candidate) const
{
  if (query.rows() != candidate.rows() || query.cols() != candidate.cols()) {
    return {std::numeric_limits<double>::max(), 0};
  }

  const Eigen::MatrixXd query_key = makeSectorKey(query);
  const Eigen::MatrixXd candidate_key = makeSectorKey(candidate);
  const int best_vkey_shift = fastAlignUsingSectorKey(query_key, candidate_key);
  const int search_radius =
    static_cast<int>(std::round(0.5 * config_.search_ratio * static_cast<double>(query.cols())));

  std::vector<int> shifts{best_vkey_shift};
  for (int i = 1; i <= search_radius; ++i) {
    shifts.push_back((best_vkey_shift + i + query.cols()) % query.cols());
    shifts.push_back((best_vkey_shift - i + query.cols()) % query.cols());
  }
  std::sort(shifts.begin(), shifts.end());
  shifts.erase(std::unique(shifts.begin(), shifts.end()), shifts.end());

  double best_distance = std::numeric_limits<double>::max();
  int best_shift = 0;
  for (const int shift : shifts) {
    const Eigen::MatrixXd shifted = circshift(candidate, shift);
    const double current_distance = directDistance(query, shifted);
    if (current_distance < best_distance) {
      best_distance = current_distance;
      best_shift = shift;
    }
  }

  return {best_distance, best_shift};
}

LoopCandidate ScanContextManager::detectLoopCandidate(
  const std::vector<Keyframe> & keyframes,
  int current_id,
  double loop_search_radius,
  double min_time_diff) const
{
  LoopCandidate result;
  result.current_id = current_id;

  if (current_id < 0 || current_id >= static_cast<int>(keyframes.size())) {
    return result;
  }
  if (current_id <= config_.exclude_recent_num) {
    return result;
  }

  const auto & current = keyframes.at(current_id);
  double best_distance = std::numeric_limits<double>::max();
  int best_id = -1;
  int best_shift = 0;

  const int last_candidate = current_id - config_.exclude_recent_num;
  for (int candidate_id = 0; candidate_id < last_candidate; ++candidate_id) {
    const auto & candidate = keyframes.at(candidate_id);
    if (std::abs(current.stamp_sec - candidate.stamp_sec) < min_time_diff) {
      continue;
    }
    if (loop_search_radius > 0.0 &&
      translationDistance(current.optimized_pose, candidate.optimized_pose) > loop_search_radius)
    {
      continue;
    }

    const auto scan_context_distance =
      distance(current.scan_context, candidate.scan_context);
    if (scan_context_distance.first < best_distance) {
      best_distance = scan_context_distance.first;
      best_shift = scan_context_distance.second;
      best_id = candidate_id;
    }
  }

  if (best_id >= 0 && best_distance < config_.distance_threshold) {
    result.valid = true;
    result.candidate_id = best_id;
    result.scan_context_distance = best_distance;
    result.yaw_diff_rad =
      static_cast<double>(best_shift) * kTwoPi / static_cast<double>(config_.sector_num);
  }

  return result;
}

double ScanContextManager::xy2thetaDeg(double x, double y)
{
  double theta = std::atan2(y, x) * 180.0 / M_PI;
  if (theta < 0.0) {
    theta += 360.0;
  }
  return theta;
}

Eigen::MatrixXd ScanContextManager::circshift(const Eigen::MatrixXd & mat, int num_shift)
{
  if (mat.cols() == 0) {
    return mat;
  }

  num_shift = ((num_shift % mat.cols()) + mat.cols()) % mat.cols();
  if (num_shift == 0) {
    return mat;
  }

  Eigen::MatrixXd shifted = Eigen::MatrixXd::Zero(mat.rows(), mat.cols());
  for (int col = 0; col < mat.cols(); ++col) {
    const int new_location = (col + num_shift) % mat.cols();
    shifted.col(new_location) = mat.col(col);
  }
  return shifted;
}

double ScanContextManager::directDistance(
  const ScanContextDescriptor & query,
  const ScanContextDescriptor & candidate) const
{
  int effective_cols = 0;
  double similarity_sum = 0.0;

  for (int col = 0; col < query.cols(); ++col) {
    const Eigen::VectorXd query_col = query.col(col);
    const Eigen::VectorXd candidate_col = candidate.col(col);
    const double query_norm = query_col.norm();
    const double candidate_norm = candidate_col.norm();
    if (query_norm == 0.0 || candidate_norm == 0.0) {
      continue;
    }

    similarity_sum += query_col.dot(candidate_col) / (query_norm * candidate_norm);
    ++effective_cols;
  }

  if (effective_cols == 0) {
    return 1.0;
  }

  return 1.0 - similarity_sum / static_cast<double>(effective_cols);
}

int ScanContextManager::fastAlignUsingSectorKey(
  const Eigen::MatrixXd & query_key,
  const Eigen::MatrixXd & candidate_key) const
{
  int best_shift = 0;
  double best_norm = std::numeric_limits<double>::max();

  for (int shift = 0; shift < query_key.cols(); ++shift) {
    const Eigen::MatrixXd shifted = circshift(candidate_key, shift);
    const double current_norm = (query_key - shifted).norm();
    if (current_norm < best_norm) {
      best_norm = current_norm;
      best_shift = shift;
    }
  }

  return best_shift;
}

}  // namespace sc_lio_sam_backend
