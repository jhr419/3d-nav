/**
 * @file scan_context.cpp
 * @brief Lightweight Scan Context implementation.
 */

#include "dddmr_sc_pgo/scan_context.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace dddmr_sc_pgo
{

ScanContextManager::ScanContextManager()
    : max_radius_(80.0)
    , num_rings_(20)
    , num_sectors_(60)
    , max_tree_depth_(20)
    , lidar_height_(0.0)
    , loop_exclude_recent_num_(30)
    , loop_candidate_num_(10)
    , loop_distance_threshold_(0.15)
    , search_ratio_(0.1)
    , last_loop_similarity_(0.0)
    , curr_keyframe_id_(0)
{
}

void ScanContextManager::setParam(
    double max_radius,
    int num_rings,
    int num_sectors,
    int max_tree_depth)
{
    max_radius_ = std::max(1.0, max_radius);
    num_rings_ = std::max(1, num_rings);
    num_sectors_ = std::max(1, num_sectors);
    max_tree_depth_ = std::max(1, max_tree_depth);
}

void ScanContextManager::setLidarHeight(double lidar_height)
{
    lidar_height_ = lidar_height;
}

void ScanContextManager::setLoopDetectionParam(
    int exclude_recent_num,
    int candidate_num,
    double distance_threshold,
    double search_ratio)
{
    loop_exclude_recent_num_ = std::max(1, exclude_recent_num);
    loop_candidate_num_ = std::max(1, candidate_num);
    loop_distance_threshold_ = std::max(0.0, distance_threshold);
    search_ratio_ = std::max(0.0, std::min(1.0, search_ratio));
}

Eigen::MatrixXf ScanContextManager::projectToPolarGrid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    const float empty_value = std::numeric_limits<float>::lowest();
    Eigen::MatrixXf polar_matrix =
        Eigen::MatrixXf::Constant(num_rings_, num_sectors_, empty_value);

    const double ring_width = max_radius_ / static_cast<double>(num_rings_);
    const double sector_width = 2.0 * M_PI / static_cast<double>(num_sectors_);

    for (const auto& point : cloud->points)
    {
        const double dist_xy = std::hypot(point.x, point.y);
        if (dist_xy > max_radius_ || dist_xy < 0.5)
        {
            continue;
        }

        int ring_idx = static_cast<int>(dist_xy / ring_width);
        ring_idx = std::min(ring_idx, num_rings_ - 1);

        double angle = std::atan2(point.y, point.x);
        if (angle < 0.0)
        {
            angle += 2.0 * M_PI;
        }

        int sector_idx = static_cast<int>(angle / sector_width);
        sector_idx = std::min(sector_idx, num_sectors_ - 1);

        const float height = static_cast<float>(point.z + lidar_height_);
        polar_matrix(ring_idx, sector_idx) =
            std::max(polar_matrix(ring_idx, sector_idx), height);
    }

    for (int r = 0; r < polar_matrix.rows(); ++r)
    {
        for (int c = 0; c < polar_matrix.cols(); ++c)
        {
            if (polar_matrix(r, c) == empty_value)
            {
                polar_matrix(r, c) = 0.0f;
            }
        }
    }

    return polar_matrix;
}

Eigen::MatrixXf ScanContextManager::makeAndSaveScanContext(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
    Eigen::MatrixXf sc = projectToPolarGrid(cloud);
    scan_contexts_.push_back(sc);
    ++curr_keyframe_id_;
    return sc;
}

Eigen::MatrixXf ScanContextManager::getLastScanContext() const
{
    if (scan_contexts_.empty())
    {
        return Eigen::MatrixXf();
    }
    return scan_contexts_.back();
}

double ScanContextManager::computeCosineSimilarity(
    const Eigen::MatrixXf& sc1,
    const Eigen::MatrixXf& sc2)
{
    if (sc1.rows() != sc2.rows() || sc1.cols() != sc2.cols())
    {
        return 0.0;
    }

    Eigen::VectorXf v1 = Eigen::Map<const Eigen::VectorXf>(
        sc1.data(), sc1.rows() * sc1.cols());
    Eigen::VectorXf v2 = Eigen::Map<const Eigen::VectorXf>(
        sc2.data(), sc2.rows() * sc2.cols());

    const double norm1 = v1.norm();
    const double norm2 = v2.norm();
    if (norm1 < 1e-10 || norm2 < 1e-10)
    {
        return 0.0;
    }

    return std::max(-1.0, std::min(1.0, static_cast<double>(v1.dot(v2)) / (norm1 * norm2)));
}

double ScanContextManager::computeCorrelation(
    const Eigen::MatrixXf& sc1,
    const Eigen::MatrixXf& sc2)
{
    if (sc1.rows() != sc2.rows() || sc1.cols() != sc2.cols())
    {
        return 0.0;
    }

    Eigen::VectorXf v1 = Eigen::Map<const Eigen::VectorXf>(
        sc1.data(), sc1.rows() * sc1.cols());
    Eigen::VectorXf v2 = Eigen::Map<const Eigen::VectorXf>(
        sc2.data(), sc2.rows() * sc2.cols());

    const float mean1 = v1.mean();
    const float mean2 = v2.mean();
    v1.array() -= mean1;
    v2.array() -= mean2;

    const double norm1 = v1.norm();
    const double norm2 = v2.norm();
    if (norm1 < 1e-10 || norm2 < 1e-10)
    {
        return 0.0;
    }

    return std::max(-1.0, std::min(1.0, static_cast<double>(v1.dot(v2)) / (norm1 * norm2)));
}

Eigen::MatrixXf ScanContextManager::shiftedBySector(const Eigen::MatrixXf& sc, int shift)
{
    Eigen::MatrixXf shifted(sc.rows(), sc.cols());
    for (int c = 0; c < sc.cols(); ++c)
    {
        const int src_col = (c + shift) % sc.cols();
        shifted.col(c) = sc.col(src_col);
    }
    return shifted;
}

double ScanContextManager::alignAndCompare(
    const Eigen::MatrixXf& sc1,
    const Eigen::MatrixXf& sc2)
{
    double best_similarity = -1.0;
    for (int shift = 0; shift < sc2.cols(); ++shift)
    {
        const Eigen::MatrixXf shifted = shiftedBySector(sc2, shift);
        const double similarity = computeCorrelation(sc1, shifted);
        best_similarity = std::max(best_similarity, similarity);
        if (best_similarity > 0.98)
        {
            break;
        }
    }
    return std::max(0.0, best_similarity);
}

std::pair<int, double> ScanContextManager::detectLoopClosureID()
{
    last_loop_similarity_ = 0.0;

    const int current_id = static_cast<int>(scan_contexts_.size()) - 1;
    if (current_id <= loop_exclude_recent_num_)
    {
        return {-1, std::numeric_limits<double>::infinity()};
    }

    const Eigen::MatrixXf& current_sc = scan_contexts_.back();
    std::vector<std::pair<double, int>> candidates;
    candidates.reserve(current_id - loop_exclude_recent_num_);

    const int history_end = current_id - loop_exclude_recent_num_;
    for (int i = 0; i < history_end; ++i)
    {
        const double similarity = alignAndCompare(current_sc, scan_contexts_[i]);
        const double distance = 1.0 - similarity;
        candidates.emplace_back(distance, i);
    }

    if (candidates.empty())
    {
        return {-1, std::numeric_limits<double>::infinity()};
    }

    std::sort(candidates.begin(), candidates.end(),
        [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

    const int ratio_count =
        static_cast<int>(std::ceil(static_cast<double>(candidates.size()) * search_ratio_));
    const int search_count =
        std::max(1, std::min(static_cast<int>(candidates.size()),
                             std::max(loop_candidate_num_, ratio_count)));

    int best_id = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (int i = 0; i < search_count; ++i)
    {
        if (candidates[i].first < best_distance)
        {
            best_distance = candidates[i].first;
            best_id = candidates[i].second;
        }
    }

    last_loop_similarity_ = 1.0 - best_distance;
    if (best_distance <= loop_distance_threshold_)
    {
        return {best_id, best_distance};
    }

    return {-1, best_distance};
}

double ScanContextManager::lastLoopSimilarity() const
{
    return last_loop_similarity_;
}

size_t ScanContextManager::size() const
{
    return scan_contexts_.size();
}

void ScanContextManager::reset()
{
    scan_contexts_.clear();
    curr_keyframe_id_ = 0;
    last_loop_similarity_ = 0.0;
}

}  // namespace dddmr_sc_pgo
