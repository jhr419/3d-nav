/**
 * @file scan_context.h
 * @brief Lightweight Scan Context manager for loop candidate retrieval.
 */

#ifndef DDDMR_SCAN_CONTEXT_H
#define DDDMR_SCAN_CONTEXT_H

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <utility>
#include <vector>

namespace dddmr_sc_pgo
{

class ScanContextManager
{
public:
    ScanContextManager();

    void setParam(double max_radius, int num_rings, int num_sectors, int max_tree_depth);
    void setLidarHeight(double lidar_height);
    void setLoopDetectionParam(
        int exclude_recent_num,
        int candidate_num,
        double distance_threshold,
        double search_ratio);

    Eigen::MatrixXf makeAndSaveScanContext(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    Eigen::MatrixXf getLastScanContext() const;

    /**
     * @brief Detect the best historical loop candidate.
     * @return {candidate_index, scan-context distance}. Distance is lower-is-better.
     */
    std::pair<int, double> detectLoopClosureID();

    double lastLoopSimilarity() const;
    size_t size() const;
    void reset();

    static double computeCosineSimilarity(const Eigen::MatrixXf& sc1, const Eigen::MatrixXf& sc2);
    static double computeCorrelation(const Eigen::MatrixXf& sc1, const Eigen::MatrixXf& sc2);

private:
    Eigen::MatrixXf projectToPolarGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
    static double alignAndCompare(const Eigen::MatrixXf& sc1, const Eigen::MatrixXf& sc2);
    static Eigen::MatrixXf shiftedBySector(const Eigen::MatrixXf& sc, int shift);

    double max_radius_;
    int num_rings_;
    int num_sectors_;
    int max_tree_depth_;
    double lidar_height_;

    int loop_exclude_recent_num_;
    int loop_candidate_num_;
    double loop_distance_threshold_;
    double search_ratio_;
    double last_loop_similarity_;

    std::vector<Eigen::MatrixXf> scan_contexts_;
    int curr_keyframe_id_;
};

}  // namespace dddmr_sc_pgo

#endif  // DDDMR_SCAN_CONTEXT_H
