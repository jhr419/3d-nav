#include "sc_pgo/scan_context/scan_context.hpp"

#include <cassert>
#include <limits>

namespace sc_pgo
{

float xy2theta(const float x, const float y)
{
  float theta = std::atan2(y, x) * 180.0f / static_cast<float>(M_PI);
  if (theta < 0.0f) {
    theta += 360.0f;
  }
  return theta;
}

Eigen::MatrixXd circshift(const Eigen::MatrixXd & mat, const int num_shift)
{
  assert(num_shift >= 0);

  if (num_shift == 0) {
    return mat;
  }

  Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(mat.rows(), mat.cols());
  for (int col_idx = 0; col_idx < mat.cols(); ++col_idx) {
    const int new_location = (col_idx + num_shift) % mat.cols();
    shifted_mat.col(new_location) = mat.col(col_idx);
  }

  return shifted_mat;
}

std::vector<float> eig2stdvec(const Eigen::MatrixXd & eigmat)
{
  return std::vector<float>(eigmat.data(), eigmat.data() + eigmat.size());
}

double ScanContextManager::distDirectSC(
  const Eigen::MatrixXd & sc1,
  const Eigen::MatrixXd & sc2) const
{
  int num_eff_cols = 0;
  double sum_sector_similarity = 0.0;

  for (int col_idx = 0; col_idx < sc1.cols(); ++col_idx) {
    const Eigen::VectorXd col_sc1 = sc1.col(col_idx);
    const Eigen::VectorXd col_sc2 = sc2.col(col_idx);

    if (col_sc1.norm() == 0.0 || col_sc2.norm() == 0.0) {
      continue;
    }

    const double sector_similarity =
      col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

    sum_sector_similarity += sector_similarity;
    ++num_eff_cols;
  }

  if (num_eff_cols == 0) {
    return 1.0;
  }

  const double sc_sim = sum_sector_similarity / static_cast<double>(num_eff_cols);
  return 1.0 - sc_sim;
}

int ScanContextManager::fastAlignUsingVkey(
  const Eigen::MatrixXd & vkey1,
  const Eigen::MatrixXd & vkey2) const
{
  int argmin_vkey_shift = 0;
  double min_vkey_diff_norm = std::numeric_limits<double>::max();

  for (int shift_idx = 0; shift_idx < vkey1.cols(); ++shift_idx) {
    const Eigen::MatrixXd vkey2_shifted = circshift(vkey2, shift_idx);
    const Eigen::MatrixXd vkey_diff = vkey1 - vkey2_shifted;
    const double cur_diff_norm = vkey_diff.norm();

    if (cur_diff_norm < min_vkey_diff_norm) {
      argmin_vkey_shift = shift_idx;
      min_vkey_diff_norm = cur_diff_norm;
    }
  }

  return argmin_vkey_shift;
}

std::pair<double, int> ScanContextManager::distanceBtnScanContext(
  const Eigen::MatrixXd & sc1,
  const Eigen::MatrixXd & sc2) const
{
  const Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(sc1);
  const Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(sc2);
  const int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);

  const int search_radius =
    std::round(0.5 * SEARCH_RATIO * static_cast<double>(sc1.cols()));
  std::vector<int> shift_idx_search_space{argmin_vkey_shift};
  for (int ii = 1; ii < search_radius + 1; ++ii) {
    shift_idx_search_space.push_back((argmin_vkey_shift + ii + sc1.cols()) % sc1.cols());
    shift_idx_search_space.push_back((argmin_vkey_shift - ii + sc1.cols()) % sc1.cols());
  }
  std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

  int argmin_shift = 0;
  double min_sc_dist = std::numeric_limits<double>::max();
  for (const int num_shift : shift_idx_search_space) {
    const Eigen::MatrixXd sc2_shifted = circshift(sc2, num_shift);
    const double cur_sc_dist = distDirectSC(sc1, sc2_shifted);
    if (cur_sc_dist < min_sc_dist) {
      argmin_shift = num_shift;
      min_sc_dist = cur_sc_dist;
    }
  }

  return std::make_pair(min_sc_dist, argmin_shift);
}

Eigen::MatrixXd ScanContextManager::makeScancontext(
  pcl::PointCloud<PointType> & scan_down) const
{
  TicTocLogger timer(debug_timing_);

  constexpr int NO_POINT = -1000;
  Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);

  for (const auto & scan_point : scan_down.points) {
    PointType point = scan_point;
    point.z += static_cast<float>(lidar_height_);

    const float azim_range = std::sqrt(point.x * point.x + point.y * point.y);
    const float azim_angle = xy2theta(point.x, point.y);

    if (azim_range > pc_max_radius_) {
      continue;
    }

    const int ring_idx = std::max(
      std::min(PC_NUM_RING, static_cast<int>(
        std::ceil((azim_range / pc_max_radius_) * static_cast<float>(PC_NUM_RING)))),
      1);
    const int sector_idx = std::max(
      std::min(PC_NUM_SECTOR, static_cast<int>(
        std::ceil((azim_angle / 360.0f) * static_cast<float>(PC_NUM_SECTOR)))),
      1);

    if (desc(ring_idx - 1, sector_idx - 1) < point.z) {
      desc(ring_idx - 1, sector_idx - 1) = point.z;
    }
  }

  for (int row_idx = 0; row_idx < desc.rows(); ++row_idx) {
    for (int col_idx = 0; col_idx < desc.cols(); ++col_idx) {
      if (desc(row_idx, col_idx) == NO_POINT) {
        desc(row_idx, col_idx) = 0.0;
      }
    }
  }

  timer.toc("PolarContext making");
  return desc;
}

Eigen::MatrixXd ScanContextManager::makeRingkeyFromScancontext(
  const Eigen::MatrixXd & desc) const
{
  Eigen::MatrixXd invariant_key(desc.rows(), 1);
  for (int row_idx = 0; row_idx < desc.rows(); ++row_idx) {
    invariant_key(row_idx, 0) = desc.row(row_idx).mean();
  }

  return invariant_key;
}

Eigen::MatrixXd ScanContextManager::makeSectorkeyFromScancontext(
  const Eigen::MatrixXd & desc) const
{
  Eigen::MatrixXd variant_key(1, desc.cols());
  for (int col_idx = 0; col_idx < desc.cols(); ++col_idx) {
    variant_key(0, col_idx) = desc.col(col_idx).mean();
  }

  return variant_key;
}

const Eigen::MatrixXd & ScanContextManager::getConstRefRecentSCD() const
{
  return polarcontexts_.back();
}

void ScanContextManager::saveScancontextAndKeys(const Eigen::MatrixXd & scan_context)
{
  Eigen::MatrixXd ringkey = makeRingkeyFromScancontext(scan_context);
  Eigen::MatrixXd sectorkey = makeSectorkeyFromScancontext(scan_context);
  std::vector<float> polarcontext_invkey_vec = eig2stdvec(ringkey);

  polarcontexts_.push_back(scan_context);
  polarcontext_invkeys_.push_back(ringkey);
  polarcontext_vkeys_.push_back(sectorkey);
  polarcontext_invkeys_mat_.push_back(polarcontext_invkey_vec);
}

void ScanContextManager::makeAndSaveScancontextAndKeys(
  pcl::PointCloud<PointType> & scan_down)
{
  const Eigen::MatrixXd scan_context = makeScancontext(scan_down);
  saveScancontextAndKeys(scan_context);
}

void ScanContextManager::setSCdistThres(const double new_threshold)
{
  sc_dist_thres_ = new_threshold;
}

void ScanContextManager::setMaximumRadius(const double max_radius)
{
  pc_max_radius_ = max_radius;
}

void ScanContextManager::setNumExcludeRecent(const int num_exclude_recent)
{
  num_exclude_recent_ = std::max(1, num_exclude_recent);
}

void ScanContextManager::setNumCandidatesFromTree(const int num_candidates_from_tree)
{
  num_candidates_from_tree_ = std::max(1, num_candidates_from_tree);
}

void ScanContextManager::setTreeMakingPeriod(const int tree_making_period)
{
  tree_making_period_ = std::max(1, tree_making_period);
}

void ScanContextManager::setLidarHeight(const double lidar_height)
{
  lidar_height_ = lidar_height;
}

void ScanContextManager::setDebugTiming(const bool enabled)
{
  debug_timing_ = enabled;
}

int ScanContextManager::numExcludeRecent() const
{
  return num_exclude_recent_;
}

std::pair<int, float> ScanContextManager::detectLoopClosureIDBetweenSession(
  std::vector<float> & current_key,
  Eigen::MatrixXd & current_desc)
{
  int loop_id = -1;

  if (polarcontext_invkeys_mat_.empty()) {
    return std::make_pair(loop_id, 0.0f);
  }

  if (!is_tree_batch_made_) {
    polarcontext_invkeys_to_search_.clear();
    polarcontext_invkeys_to_search_.assign(
      polarcontext_invkeys_mat_.begin(),
      polarcontext_invkeys_mat_.end());

    polarcontext_tree_batch_.reset();
    polarcontext_tree_batch_ = std::make_unique<InvKeyTree>(
      PC_NUM_RING,
      polarcontext_invkeys_to_search_,
      10);

    is_tree_batch_made_ = true;
  }

  double min_dist = std::numeric_limits<double>::max();
  int nn_align = 0;
  int nn_idx = 0;

  const int num_candidates = std::min(
    num_candidates_from_tree_,
    static_cast<int>(polarcontext_invkeys_to_search_.size()));

  std::vector<size_t> candidate_indexes(num_candidates);
  std::vector<float> out_dists_sqr(num_candidates);

  nanoflann::KNNResultSet<float> knnsearch_result(num_candidates);
  knnsearch_result.init(candidate_indexes.data(), out_dists_sqr.data());
  polarcontext_tree_batch_->index->findNeighbors(
    knnsearch_result,
    current_key.data(),
    nanoflann::SearchParams(10));

  TicTocLogger timer(debug_timing_);
  for (int candidate_iter_idx = 0; candidate_iter_idx < num_candidates; ++candidate_iter_idx) {
    const Eigen::MatrixXd polarcontext_candidate =
      polarcontexts_[candidate_indexes[candidate_iter_idx]];
    const auto sc_dist_result = distanceBtnScanContext(current_desc, polarcontext_candidate);

    if (sc_dist_result.first < min_dist) {
      min_dist = sc_dist_result.first;
      nn_align = sc_dist_result.second;
      nn_idx = static_cast<int>(candidate_indexes[candidate_iter_idx]);
    }
  }
  timer.toc("Distance calc");

  if (min_dist < sc_dist_thres_) {
    loop_id = nn_idx;
  }

  const float yaw_diff_rad =
    static_cast<float>(deg2rad(static_cast<double>(nn_align) * PC_UNIT_SECTORANGLE));
  return std::make_pair(loop_id, yaw_diff_rad);
}

std::pair<int, float> ScanContextManager::detectLoopClosureID()
{
  int loop_id = -1;

  if (polarcontext_invkeys_mat_.size() < static_cast<size_t>(num_exclude_recent_ + 1)) {
    return std::make_pair(loop_id, 0.0f);
  }

  const auto current_key = polarcontext_invkeys_mat_.back();
  const auto current_desc = polarcontexts_.back();

  if (tree_making_period_counter_ % tree_making_period_ == 0) {
    TicTocLogger timer(debug_timing_);

    polarcontext_invkeys_to_search_.clear();
    polarcontext_invkeys_to_search_.assign(
      polarcontext_invkeys_mat_.begin(),
      polarcontext_invkeys_mat_.end() - num_exclude_recent_);

    polarcontext_tree_.reset();
    polarcontext_tree_ = std::make_unique<InvKeyTree>(
      PC_NUM_RING,
      polarcontext_invkeys_to_search_,
      10);

    timer.toc("Tree construction");
  }
  ++tree_making_period_counter_;

  if (!polarcontext_tree_ || polarcontext_invkeys_to_search_.empty()) {
    return std::make_pair(loop_id, 0.0f);
  }

  double min_dist = std::numeric_limits<double>::max();
  int nn_align = 0;
  int nn_idx = 0;

  const int num_candidates = std::min(
    num_candidates_from_tree_,
    static_cast<int>(polarcontext_invkeys_to_search_.size()));

  std::vector<size_t> candidate_indexes(num_candidates);
  std::vector<float> out_dists_sqr(num_candidates);

  TicTocLogger tree_timer(debug_timing_);
  nanoflann::KNNResultSet<float> knnsearch_result(num_candidates);
  knnsearch_result.init(candidate_indexes.data(), out_dists_sqr.data());
  polarcontext_tree_->index->findNeighbors(
    knnsearch_result,
    current_key.data(),
    nanoflann::SearchParams(10));
  tree_timer.toc("Tree search");

  TicTocLogger dist_timer(debug_timing_);
  for (int candidate_iter_idx = 0; candidate_iter_idx < num_candidates; ++candidate_iter_idx) {
    const Eigen::MatrixXd polarcontext_candidate =
      polarcontexts_[candidate_indexes[candidate_iter_idx]];
    const auto sc_dist_result = distanceBtnScanContext(current_desc, polarcontext_candidate);

    if (sc_dist_result.first < min_dist) {
      min_dist = sc_dist_result.first;
      nn_align = sc_dist_result.second;
      nn_idx = static_cast<int>(candidate_indexes[candidate_iter_idx]);
    }
  }
  dist_timer.toc("Distance calc");

  if (min_dist < sc_dist_thres_) {
    loop_id = nn_idx;
    std::cout.precision(3);
    std::cout << "[Loop found] Nearest distance: " << min_dist
      << " between " << polarcontexts_.size() - 1
      << " and " << nn_idx << "." << std::endl;
  } else {
    std::cout.precision(3);
    std::cout << "[Not loop] Nearest distance: " << min_dist
      << " between " << polarcontexts_.size() - 1
      << " and " << nn_idx << "." << std::endl;
  }

  const float yaw_diff_rad =
    static_cast<float>(deg2rad(static_cast<double>(nn_align) * PC_UNIT_SECTORANGLE));
  return std::make_pair(loop_id, yaw_diff_rad);
}

}  // namespace sc_pgo
