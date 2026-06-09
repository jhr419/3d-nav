#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>

#include "sc_pgo/common.hpp"
#include "sc_pgo/scan_context/kdtree_vector_of_vectors_adaptor.hpp"
#include "sc_pgo/scan_context/nanoflann.hpp"
#include "sc_pgo/tic_toc.hpp"

namespace sc_pgo
{

using KeyMat = std::vector<std::vector<float>>;
using InvKeyTree = KDTreeVectorOfVectorsAdaptor<KeyMat, float>;

float xy2theta(float x, float y);
Eigen::MatrixXd circshift(const Eigen::MatrixXd & mat, int num_shift);
std::vector<float> eig2stdvec(const Eigen::MatrixXd & eigmat);

class ScanContextManager
{
public:
  ScanContextManager() = default;

  Eigen::MatrixXd makeScancontext(pcl::PointCloud<PointType> & scan_down) const;
  Eigen::MatrixXd makeRingkeyFromScancontext(const Eigen::MatrixXd & desc) const;
  Eigen::MatrixXd makeSectorkeyFromScancontext(const Eigen::MatrixXd & desc) const;

  int fastAlignUsingVkey(const Eigen::MatrixXd & vkey1, const Eigen::MatrixXd & vkey2) const;
  double distDirectSC(const Eigen::MatrixXd & sc1, const Eigen::MatrixXd & sc2) const;
  std::pair<double, int> distanceBtnScanContext(
    const Eigen::MatrixXd & sc1,
    const Eigen::MatrixXd & sc2) const;

  void makeAndSaveScancontextAndKeys(pcl::PointCloud<PointType> & scan_down);
  void saveScancontextAndKeys(const Eigen::MatrixXd & scan_context);
  std::pair<int, float> detectLoopClosureID();
  std::pair<int, float> detectLoopClosureIDBetweenSession(
    std::vector<float> & current_key,
    Eigen::MatrixXd & current_desc);

  const Eigen::MatrixXd & getConstRefRecentSCD() const;

  void setSCdistThres(double new_threshold);
  void setMaximumRadius(double max_radius);
  void setNumExcludeRecent(int num_exclude_recent);
  void setNumCandidatesFromTree(int num_candidates_from_tree);
  void setTreeMakingPeriod(int tree_making_period);
  void setLidarHeight(double lidar_height);
  void setDebugTiming(bool enabled);

  int numExcludeRecent() const;

private:
  double lidar_height_ = 2.0;
  static constexpr int PC_NUM_RING = 20;
  static constexpr int PC_NUM_SECTOR = 60;
  double pc_max_radius_ = 80.0;
  static constexpr double PC_UNIT_SECTORANGLE = 360.0 / static_cast<double>(PC_NUM_SECTOR);

  int num_exclude_recent_ = 30;
  int num_candidates_from_tree_ = 3;
  static constexpr double SEARCH_RATIO = 0.1;
  double sc_dist_thres_ = 0.2;
  int tree_making_period_ = 30;
  int tree_making_period_counter_ = 0;
  bool debug_timing_ = false;

  std::vector<double> polarcontexts_timestamp_;
  std::vector<Eigen::MatrixXd> polarcontexts_;
  std::vector<Eigen::MatrixXd> polarcontext_invkeys_;
  std::vector<Eigen::MatrixXd> polarcontext_vkeys_;

  KeyMat polarcontext_invkeys_mat_;
  KeyMat polarcontext_invkeys_to_search_;
  std::unique_ptr<InvKeyTree> polarcontext_tree_;

  bool is_tree_batch_made_ = false;
  std::unique_ptr<InvKeyTree> polarcontext_tree_batch_;
};

}  // namespace sc_pgo
