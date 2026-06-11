/**
 * @file sc_pgo_node.cpp
 * @brief Scan Context loop detection and GTSAM pose graph optimization for FAST-LIO2.
 */

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "dddmr_sc_pgo/scan_context.h"

namespace
{

gtsam::Symbol X(int key)
{
    return gtsam::Symbol('x', key);
}

struct KeyFrame
{
    int id{0};
    rclcpp::Time stamp;
    Eigen::Matrix4f odom_pose{Eigen::Matrix4f::Identity()};
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_local;
    Eigen::MatrixXf scan_context;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

Eigen::Matrix4f matrixFromOdometry(const nav_msgs::msg::Odometry& odom)
{
    const auto& p = odom.pose.pose.position;
    const auto& q = odom.pose.pose.orientation;

    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block<3, 1>(0, 3) = Eigen::Vector3f(p.x, p.y, p.z);

    Eigen::Quaternionf quat(q.w, q.x, q.y, q.z);
    quat.normalize();
    matrix.block<3, 3>(0, 0) = quat.toRotationMatrix();
    return matrix;
}

gtsam::Pose3 pose3FromMatrix(const Eigen::Matrix4f& matrix)
{
    const Eigen::Matrix3d rotation = matrix.block<3, 3>(0, 0).cast<double>();
    const Eigen::Vector3d translation = matrix.block<3, 1>(0, 3).cast<double>();
    return gtsam::Pose3(gtsam::Rot3(rotation), gtsam::Point3(translation));
}

Eigen::Matrix4f matrixFromPose3(const gtsam::Pose3& pose)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
    matrix.block<3, 3>(0, 0) = pose.rotation().matrix().cast<float>();
    matrix.block<3, 1>(0, 3) =
        Eigen::Vector3f(static_cast<float>(pose.x()),
                        static_cast<float>(pose.y()),
                        static_cast<float>(pose.z()));
    return matrix;
}

}  // namespace

class SCPGONode : public rclcpp::Node
{
public:
    SCPGONode()
        : Node("sc_pgo_node")
    {
        loadParameters();
        configureScanContext();
        initISAM2();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 100,
            std::bind(&SCPGONode::odomCallback, this, std::placeholders::_1));

        keyframe_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            keyframe_cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&SCPGONode::cloudCallback, this, std::placeholders::_1));

        optimized_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "/pgo/optimized_odometry", 20);
        optimized_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
            "/pgo/optimized_path", 10);
        optimized_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/pgo/optimized_map", rclcpp::QoS(1).transient_local());
        loop_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/pgo/loop_markers", 10);

        const double map_period =
            optimized_map_publish_frequency_ > 0.0
                ? 1.0 / optimized_map_publish_frequency_
                : 5.0;
        optimized_map_timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(map_period)),
            std::bind(&SCPGONode::optimizedMapTimerCallback, this));

        RCLCPP_INFO(this->get_logger(), "SC-PGO started");
        RCLCPP_INFO(this->get_logger(), "Subscribe odom: %s", odom_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Subscribe keyframe cloud: %s", keyframe_cloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Reference FAST-LIO map topic: %s", map_topic_.c_str());
        RCLCPP_INFO(this->get_logger(),
            "Publish: /pgo/optimized_odometry, /pgo/optimized_path, /pgo/optimized_map, /pgo/loop_markers");
        RCLCPP_INFO(this->get_logger(), "Frames: map=%s odom=%s base=%s lidar=%s",
            map_frame_.c_str(), odom_frame_.c_str(), base_frame_.c_str(), lidar_frame_.c_str());
        if (publish_tf_ && publish_map_to_odom_ && map_frame_ == odom_frame_)
        {
            RCLCPP_WARN(this->get_logger(),
                "map_frame and odom_frame are both '%s'; map->odom TF will be skipped.",
                map_frame_.c_str());
        }
    }

private:
    void loadParameters()
    {
        odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/Odometry");
        cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/cloud_registered");
        map_topic_ = this->declare_parameter<std::string>("map_topic", "/Laser_map");
        keyframe_cloud_topic_ =
            this->declare_parameter<std::string>("keyframe_cloud_topic", cloud_topic_);

        map_frame_ = this->declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = this->declare_parameter<std::string>("odom_frame", "camera_init");
        base_frame_ = this->declare_parameter<std::string>("base_frame", "body");
        lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "livox_frame");

        publish_tf_ = this->declare_parameter<bool>("publish_tf", true);
        publish_map_to_odom_ = this->declare_parameter<bool>("publish_map_to_odom", true);
        cloud_in_world_frame_ = this->declare_parameter<bool>("cloud_in_world_frame", true);

        keyframe_distance_thresh_ =
            this->declare_parameter<double>("keyframe_distance_thresh", 1.0);
        keyframe_angle_thresh_rad_ =
            this->declare_parameter<double>("keyframe_angle_thresh_deg", 10.0) * M_PI / 180.0;
        keyframe_time_thresh_ =
            this->declare_parameter<double>("keyframe_time_thresh", 1.0);
        keyframe_cloud_voxel_size_ =
            this->declare_parameter<double>("keyframe_cloud_voxel_size", 0.5);

        loop_detect_enabled_ =
            this->declare_parameter<bool>("loop_detect_enabled", true);
        loop_detect_frequency_ =
            this->declare_parameter<double>("loop_detect_frequency", 1.0);
        loop_exclude_recent_num_ =
            this->declare_parameter<int>("loop_exclude_recent_num", 30);
        loop_candidate_num_ =
            this->declare_parameter<int>("loop_candidate_num", 10);
        loop_score_threshold_ =
            this->declare_parameter<double>("loop_score_threshold", 0.15);
        floor_height_threshold_ =
            this->declare_parameter<double>("floor_height_threshold", 4.0);

        sc_lidar_height_ =
            this->declare_parameter<double>("sc_lidar_height", 0.0);
        sc_max_radius_ =
            this->declare_parameter<double>("sc_max_radius", 80.0);
        sc_num_rings_ =
            this->declare_parameter<int>("sc_num_rings", 20);
        sc_num_sectors_ =
            this->declare_parameter<int>("sc_num_sectors", 60);
        sc_tree_depth_ =
            this->declare_parameter<int>("sc_tree_depth", 20);
        sc_search_ratio_ =
            this->declare_parameter<double>("sc_search_ratio", 0.1);

        loop_icp_max_corr_dist_ =
            this->declare_parameter<double>("loop_icp_max_corr_dist", 2.0);
        loop_icp_fitness_threshold_ =
            this->declare_parameter<double>("loop_icp_fitness_threshold", 0.4);
        loop_icp_max_iterations_ =
            this->declare_parameter<int>("loop_icp_max_iterations", 50);
        loop_submap_nearby_frames_ =
            this->declare_parameter<int>("loop_submap_nearby_frames", 10);

        odom_trans_noise_ =
            this->declare_parameter<double>("odom_trans_noise", 0.1);
        odom_rot_noise_ =
            this->declare_parameter<double>("odom_rot_noise", 0.05);
        loop_noise_ =
            this->declare_parameter<double>("loop_noise", 0.5);

        optimized_map_publish_frequency_ =
            this->declare_parameter<double>("optimized_map_publish_frequency", 0.2);
        optimized_map_voxel_size_ =
            this->declare_parameter<double>("optimized_map_voxel_size", 0.2);
        save_optimized_map_ =
            this->declare_parameter<bool>("save_optimized_map", true);
        save_directory_ =
            this->declare_parameter<std::string>("save_directory", "/home/jhr/3dnav_ws/maps/pgo");

        keyframe_cloud_voxel_size_ = std::max(0.01, keyframe_cloud_voxel_size_);
        optimized_map_voxel_size_ = std::max(0.01, optimized_map_voxel_size_);
        loop_submap_nearby_frames_ = std::max(0, loop_submap_nearby_frames_);

        keyframe_downsample_filter_.setLeafSize(
            keyframe_cloud_voxel_size_,
            keyframe_cloud_voxel_size_,
            keyframe_cloud_voxel_size_);
    }

    void configureScanContext()
    {
        sc_manager_.setParam(sc_max_radius_, sc_num_rings_, sc_num_sectors_, sc_tree_depth_);
        sc_manager_.setLidarHeight(sc_lidar_height_);
        sc_manager_.setLoopDetectionParam(
            loop_exclude_recent_num_,
            loop_candidate_num_,
            loop_score_threshold_,
            sc_search_ratio_);

        RCLCPP_INFO(this->get_logger(),
            "Scan Context: radius=%.1f rings=%d sectors=%d exclude=%d threshold=%.3f",
            sc_max_radius_, sc_num_rings_, sc_num_sectors_,
            loop_exclude_recent_num_, loop_score_threshold_);
    }

    void initISAM2()
    {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.01;
        params.relinearizeSkip = 1;
        params.optimizationParams = gtsam::ISAM2DoglegParams();
        isam_ = std::make_unique<gtsam::ISAM2>(params);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        latest_odom_pose_ = matrixFromOdometry(*msg);
        latest_odom_stamp_ = rclcpp::Time(msg->header.stamp);
        latest_odom_frame_ = msg->header.frame_id.empty() ? odom_frame_ : msg->header.frame_id;
        latest_base_frame_ = msg->child_frame_id.empty() ? base_frame_ : msg->child_frame_id;

        if (shouldCreateKeyframe(*latest_odom_pose_, *latest_odom_stamp_))
        {
            pending_keyframe_pose_ = latest_odom_pose_;
            pending_keyframe_stamp_ = latest_odom_stamp_;
        }

        ++odom_count_;
        if (odom_count_ % 500 == 0)
        {
            RCLCPP_INFO(this->get_logger(),
                "Received odom=%zu keyframes=%zu loop_edges=%zu",
                odom_count_, keyframes_.size(), loop_pairs_.size());
        }
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!pending_keyframe_pose_ || !pending_keyframe_stamp_)
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);
        if (cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Skip empty keyframe cloud.");
            return;
        }

        const Eigen::Matrix4f keyframe_pose = *pending_keyframe_pose_;
        const rclcpp::Time keyframe_stamp = *pending_keyframe_stamp_;
        pending_keyframe_pose_.reset();
        pending_keyframe_stamp_.reset();

        pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud =
            convertCloudToKeyframeLocal(cloud, msg->header.frame_id, keyframe_pose);

        createKeyframe(local_cloud, keyframe_pose, keyframe_stamp);
    }

    bool shouldCreateKeyframe(
        const Eigen::Matrix4f& current_pose,
        const rclcpp::Time& current_stamp) const
    {
        if (!last_keyframe_pose_ || !last_keyframe_stamp_)
        {
            return true;
        }

        const Eigen::Vector3f delta_t =
            current_pose.block<3, 1>(0, 3) - last_keyframe_pose_->block<3, 1>(0, 3);
        const double distance = delta_t.norm();

        Eigen::Quaternionf q_last(last_keyframe_pose_->block<3, 3>(0, 0));
        Eigen::Quaternionf q_curr(current_pose.block<3, 3>(0, 0));
        q_last.normalize();
        q_curr.normalize();
        const double angle = q_last.angularDistance(q_curr);

        const double dt = (current_stamp - *last_keyframe_stamp_).seconds();

        return distance >= keyframe_distance_thresh_ ||
               angle >= keyframe_angle_thresh_rad_ ||
               dt >= keyframe_time_thresh_;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr convertCloudToKeyframeLocal(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
        const std::string& cloud_frame,
        const Eigen::Matrix4f& odom_pose) const
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>());

        bool cloud_is_local = !cloud_in_world_frame_;
        if (cloud_frame == base_frame_ || cloud_frame == lidar_frame_)
        {
            cloud_is_local = true;
        }

        if (cloud_is_local)
        {
            *local_cloud = *cloud;
        }
        else
        {
            pcl::transformPointCloud(*cloud, *local_cloud, odom_pose.inverse());
        }

        return local_cloud;
    }

    void createKeyframe(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& local_cloud,
        const Eigen::Matrix4f& odom_pose,
        const rclcpp::Time& stamp)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(
            new pcl::PointCloud<pcl::PointXYZI>());
        keyframe_downsample_filter_.setInputCloud(local_cloud);
        keyframe_downsample_filter_.filter(*downsampled_cloud);

        if (downsampled_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Skip keyframe with empty downsampled cloud.");
            return;
        }

        auto keyframe = std::make_shared<KeyFrame>();
        keyframe->id = next_keyframe_id_++;
        keyframe->stamp = stamp;
        keyframe->odom_pose = odom_pose;
        keyframe->cloud_local = downsampled_cloud;
        keyframe->scan_context = sc_manager_.makeAndSaveScanContext(downsampled_cloud);

        keyframes_.push_back(keyframe);
        last_keyframe_pose_ = odom_pose;
        last_keyframe_stamp_ = stamp;

        addOdometryFactor(keyframe);
        detectLoop(keyframe);
        runOptimization();
        publishOptimizedResults(keyframe);

        const Eigen::Vector3f t = odom_pose.block<3, 1>(0, 3);
        RCLCPP_INFO(this->get_logger(),
            "Added keyframe index=%d pose=[%.3f, %.3f, %.3f] cloud_size=%zu",
            keyframe->id, t.x(), t.y(), t.z(), downsampled_cloud->size());
    }

    void addOdometryFactor(const std::shared_ptr<KeyFrame>& keyframe)
    {
        const gtsam::Pose3 current_pose = pose3FromMatrix(keyframe->odom_pose);

        if (keyframes_.size() == 1)
        {
            auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
            new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(
                X(keyframe->id), current_pose, prior_noise));
            new_values_.insert(X(keyframe->id), current_pose);
            return;
        }

        const auto& previous = keyframes_[keyframes_.size() - 2];
        const Eigen::Matrix4f relative_matrix =
            previous->odom_pose.inverse() * keyframe->odom_pose;
        const gtsam::Pose3 relative_pose = pose3FromMatrix(relative_matrix);

        auto odom_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << odom_rot_noise_, odom_rot_noise_, odom_rot_noise_,
             odom_trans_noise_, odom_trans_noise_, odom_trans_noise_).finished());
        new_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(previous->id), X(keyframe->id), relative_pose, odom_noise));
        new_values_.insert(X(keyframe->id), current_pose);
    }

    void detectLoop(const std::shared_ptr<KeyFrame>& current_keyframe)
    {
        if (!loop_detect_enabled_)
        {
            return;
        }
        if (static_cast<int>(keyframes_.size()) <= loop_exclude_recent_num_ + 1)
        {
            return;
        }
        if (last_loop_detect_stamp_ && loop_detect_frequency_ > 0.0)
        {
            const double dt = (current_keyframe->stamp - *last_loop_detect_stamp_).seconds();
            if (dt >= 0.0 && dt < 1.0 / loop_detect_frequency_)
            {
                return;
            }
        }
        last_loop_detect_stamp_ = current_keyframe->stamp;

        const auto [candidate_id, sc_distance] = sc_manager_.detectLoopClosureID();
        if (candidate_id < 0)
        {
            if (std::isfinite(sc_distance))
            {
                RCLCPP_DEBUG(this->get_logger(),
                    "Loop candidate rejected by SC threshold: current=%d best_distance=%.3f threshold=%.3f",
                    current_keyframe->id, sc_distance, loop_score_threshold_);
            }
            return;
        }

        RCLCPP_INFO(this->get_logger(),
            "Loop candidate found: candidate=%d current=%d SC distance=%.3f similarity=%.3f",
            candidate_id, current_keyframe->id,
            sc_distance, sc_manager_.lastLoopSimilarity());
        addLoopFactor(candidate_id, current_keyframe->id, sc_distance);
    }

    void addLoopFactor(int candidate_id, int current_id, double sc_distance)
    {
        if (candidate_id < 0 || current_id < 0 ||
            candidate_id >= static_cast<int>(keyframes_.size()) ||
            current_id >= static_cast<int>(keyframes_.size()) ||
            candidate_id == current_id)
        {
            return;
        }

        const std::pair<int, int> loop_pair(candidate_id, current_id);
        if (loop_pair_set_.count(loop_pair) > 0)
        {
            return;
        }

        const auto& candidate = keyframes_[candidate_id];
        const auto& current = keyframes_[current_id];
        const double z_diff = std::abs(candidate->odom_pose(2, 3) - current->odom_pose(2, 3));
        if (floor_height_threshold_ > 0.0 && z_diff > floor_height_threshold_)
        {
            RCLCPP_WARN(this->get_logger(),
                "Loop candidate rejected: candidate=%d current=%d z_diff=%.3f > %.3f",
                candidate_id, current_id, z_diff, floor_height_threshold_);
            return;
        }

        Eigen::Matrix4f relative_pose = Eigen::Matrix4f::Identity();
        double icp_fitness = std::numeric_limits<double>::infinity();
        if (!performICPVerification(candidate_id, current_id, relative_pose, icp_fitness))
        {
            RCLCPP_WARN(this->get_logger(),
                "Loop candidate rejected by ICP: candidate=%d current=%d SC distance=%.3f ICP fitness=%.3f",
                candidate_id, current_id, sc_distance, icp_fitness);
            return;
        }

        const double scaled_loop_noise =
            loop_noise_ * std::max(1.0, icp_fitness / std::max(1e-3, loop_icp_fitness_threshold_));
        auto loop_noise_model = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << scaled_loop_noise, scaled_loop_noise, scaled_loop_noise,
             scaled_loop_noise, scaled_loop_noise, scaled_loop_noise).finished());

        new_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            X(candidate_id), X(current_id), pose3FromMatrix(relative_pose), loop_noise_model));

        loop_pair_set_.insert(loop_pair);
        loop_pairs_.push_back(loop_pair);
        RCLCPP_INFO(this->get_logger(),
            "Loop accepted: candidate=%d current=%d SC distance=%.3f ICP fitness=%.3f loop_edges=%zu",
            candidate_id, current_id, sc_distance, icp_fitness, loop_pairs_.size());
    }

    bool performICPVerification(
        int candidate_id,
        int current_id,
        Eigen::Matrix4f& relative_pose,
        double& fitness) const
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr target =
            buildSubmapInOdomFrame(candidate_id, loop_submap_nearby_frames_);
        pcl::PointCloud<pcl::PointXYZI>::Ptr source =
            buildSubmapInOdomFrame(current_id, loop_submap_nearby_frames_);

        if (target->size() < 50 || source->size() < 50)
        {
            fitness = std::numeric_limits<double>::infinity();
            return false;
        }

        pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
        icp.setInputSource(source);
        icp.setInputTarget(target);
        icp.setMaxCorrespondenceDistance(loop_icp_max_corr_dist_);
        icp.setMaximumIterations(loop_icp_max_iterations_);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-6);

        pcl::PointCloud<pcl::PointXYZI> aligned;
        icp.align(aligned);

        if (!icp.hasConverged())
        {
            fitness = std::numeric_limits<double>::infinity();
            return false;
        }

        fitness = icp.getFitnessScore(loop_icp_max_corr_dist_);
        if (fitness > loop_icp_fitness_threshold_)
        {
            return false;
        }

        const Eigen::Matrix4f correction = icp.getFinalTransformation();
        const Eigen::Matrix4f corrected_current_pose =
            correction * keyframes_[current_id]->odom_pose;
        relative_pose =
            keyframes_[candidate_id]->odom_pose.inverse() * corrected_current_pose;
        return true;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr buildSubmapInOdomFrame(
        int center_id,
        int nearby_frames) const
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr submap(new pcl::PointCloud<pcl::PointXYZI>());
        const int begin = std::max(0, center_id - nearby_frames);
        const int end =
            std::min(static_cast<int>(keyframes_.size()) - 1, center_id + nearby_frames);

        for (int i = begin; i <= end; ++i)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(
                new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(
                *keyframes_[i]->cloud_local, *transformed, keyframes_[i]->odom_pose);
            *submap += *transformed;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setLeafSize(keyframe_cloud_voxel_size_, keyframe_cloud_voxel_size_, keyframe_cloud_voxel_size_);
        voxel.setInputCloud(submap);
        voxel.filter(*downsampled);
        return downsampled;
    }

    void runOptimization()
    {
        if (new_factors_.empty() && new_values_.empty())
        {
            return;
        }

        try
        {
            isam_->update(new_factors_, new_values_);
            optimized_values_ = isam_->calculateEstimate();
            RCLCPP_INFO(this->get_logger(),
                "PGO optimized: keyframes=%zu loop_edges=%zu",
                keyframes_.size(), loop_pairs_.size());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "GTSAM optimization failed: %s", e.what());
        }

        new_factors_.resize(0);
        new_values_.clear();
    }

    void publishOptimizedResults(const std::shared_ptr<KeyFrame>& latest_keyframe)
    {
        if (optimized_values_.empty() ||
            !optimized_values_.exists(X(latest_keyframe->id)))
        {
            return;
        }

        const gtsam::Pose3 latest_pose =
            optimized_values_.at<gtsam::Pose3>(X(latest_keyframe->id));
        publishOptimizedOdom(*latest_keyframe, latest_pose);
        publishOptimizedPath();
        publishLoopMarkers();

        if (publish_tf_ && publish_map_to_odom_)
        {
            publishMapToOdomTF(*latest_keyframe, latest_pose);
        }
    }

    void publishOptimizedOdom(
        const KeyFrame& keyframe,
        const gtsam::Pose3& optimized_pose)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = keyframe.stamp;
        odom.header.frame_id = map_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = optimized_pose.x();
        odom.pose.pose.position.y = optimized_pose.y();
        odom.pose.pose.position.z = optimized_pose.z();

        Eigen::Quaterniond q = optimized_pose.rotation().toQuaternion();
        q.normalize();
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        optimized_odom_pub_->publish(odom);
    }

    void publishOptimizedPath()
    {
        nav_msgs::msg::Path path;
        path.header.stamp = this->now();
        path.header.frame_id = map_frame_;

        for (const auto& keyframe : keyframes_)
        {
            if (!optimized_values_.exists(X(keyframe->id)))
            {
                continue;
            }

            const gtsam::Pose3 pose = optimized_values_.at<gtsam::Pose3>(X(keyframe->id));
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = map_frame_;
            pose_stamped.header.stamp = keyframe->stamp;
            pose_stamped.pose.position.x = pose.x();
            pose_stamped.pose.position.y = pose.y();
            pose_stamped.pose.position.z = pose.z();

            Eigen::Quaterniond q = pose.rotation().toQuaternion();
            q.normalize();
            pose_stamped.pose.orientation.x = q.x();
            pose_stamped.pose.orientation.y = q.y();
            pose_stamped.pose.orientation.z = q.z();
            pose_stamped.pose.orientation.w = q.w();

            path.poses.push_back(pose_stamped);
        }

        optimized_path_pub_->publish(path);
    }

    void publishMapToOdomTF(
        const KeyFrame& keyframe,
        const gtsam::Pose3& optimized_map_to_base)
    {
        if (map_frame_ == odom_frame_)
        {
            return;
        }

        const Eigen::Matrix4f map_to_base = matrixFromPose3(optimized_map_to_base);
        const Eigen::Matrix4f map_to_odom = map_to_base * keyframe.odom_pose.inverse();

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = keyframe.stamp;
        transform.header.frame_id = map_frame_;
        transform.child_frame_id = odom_frame_;
        transform.transform.translation.x = map_to_odom(0, 3);
        transform.transform.translation.y = map_to_odom(1, 3);
        transform.transform.translation.z = map_to_odom(2, 3);

        Eigen::Quaternionf q(map_to_odom.block<3, 3>(0, 0));
        q.normalize();
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }

    void optimizedMapTimerCallback()
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        publishOptimizedMap();
    }

    void publishOptimizedMap()
    {
        if (optimized_values_.empty() || keyframes_.empty())
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& keyframe : keyframes_)
        {
            if (!optimized_values_.exists(X(keyframe->id)))
            {
                continue;
            }

            const gtsam::Pose3 pose = optimized_values_.at<gtsam::Pose3>(X(keyframe->id));
            pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(
                new pcl::PointCloud<pcl::PointXYZI>());
            pcl::transformPointCloud(
                *keyframe->cloud_local, *transformed, matrixFromPose3(pose));
            *map_cloud += *transformed;
        }

        if (map_cloud->empty())
        {
            return;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_map(
            new pcl::PointCloud<pcl::PointXYZI>());
        pcl::VoxelGrid<pcl::PointXYZI> voxel;
        voxel.setLeafSize(
            optimized_map_voxel_size_,
            optimized_map_voxel_size_,
            optimized_map_voxel_size_);
        voxel.setInputCloud(map_cloud);
        voxel.filter(*downsampled_map);

        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(*downsampled_map, msg);
        msg.header.stamp = this->now();
        msg.header.frame_id = map_frame_;
        optimized_map_pub_->publish(msg);

        if (save_optimized_map_)
        {
            saveOptimizedOutputs(downsampled_map);
        }
    }

    void publishLoopMarkers()
    {
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;

        for (const auto& loop_pair : loop_pairs_)
        {
            if (!optimized_values_.exists(X(loop_pair.first)) ||
                !optimized_values_.exists(X(loop_pair.second)))
            {
                continue;
            }

            const gtsam::Pose3 pose_a =
                optimized_values_.at<gtsam::Pose3>(X(loop_pair.first));
            const gtsam::Pose3 pose_b =
                optimized_values_.at<gtsam::Pose3>(X(loop_pair.second));

            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->now();
            marker.header.frame_id = map_frame_;
            marker.ns = "pgo_loop_edges";
            marker.id = marker_id++;
            marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.12;
            marker.color.r = 1.0;
            marker.color.g = 0.15;
            marker.color.b = 0.05;
            marker.color.a = 1.0;

            geometry_msgs::msg::Point p_a;
            p_a.x = pose_a.x();
            p_a.y = pose_a.y();
            p_a.z = pose_a.z();
            geometry_msgs::msg::Point p_b;
            p_b.x = pose_b.x();
            p_b.y = pose_b.y();
            p_b.z = pose_b.z();
            marker.points.push_back(p_a);
            marker.points.push_back(p_b);
            markers.markers.push_back(marker);
        }

        loop_marker_pub_->publish(markers);
    }

    void saveOptimizedOutputs(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& optimized_map)
    {
        if (last_saved_keyframe_count_ == keyframes_.size() &&
            last_saved_loop_count_ == loop_pairs_.size())
        {
            return;
        }

        try
        {
            std::filesystem::create_directories(save_directory_);
            const std::string map_path = save_directory_ + "/optimized_map.pcd";
            const std::string trajectory_path = save_directory_ + "/optimized_trajectory.txt";

            pcl::io::savePCDFileBinary(map_path, *optimized_map);

            std::ofstream trajectory(trajectory_path);
            trajectory << "# timestamp x y z qx qy qz qw\n";
            trajectory << std::fixed << std::setprecision(9);
            for (const auto& keyframe : keyframes_)
            {
                if (!optimized_values_.exists(X(keyframe->id)))
                {
                    continue;
                }
                const gtsam::Pose3 pose =
                    optimized_values_.at<gtsam::Pose3>(X(keyframe->id));
                Eigen::Quaterniond q = pose.rotation().toQuaternion();
                q.normalize();
                trajectory << keyframe->stamp.seconds() << " "
                           << pose.x() << " " << pose.y() << " " << pose.z() << " "
                           << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                           << "\n";
            }

            last_saved_keyframe_count_ = keyframes_.size();
            last_saved_loop_count_ = loop_pairs_.size();
            RCLCPP_INFO(this->get_logger(),
                "Saved optimized map and trajectory to %s", save_directory_.c_str());
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(),
                "Failed to save optimized outputs: %s", e.what());
        }
    }

    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr keyframe_cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr optimized_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_marker_pub_;
    rclcpp::TimerBase::SharedPtr optimized_map_timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Parameters
    std::string odom_topic_;
    std::string cloud_topic_;
    std::string map_topic_;
    std::string keyframe_cloud_topic_;
    std::string map_frame_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string lidar_frame_;
    bool publish_tf_{true};
    bool publish_map_to_odom_{true};
    bool cloud_in_world_frame_{true};

    double keyframe_distance_thresh_{1.0};
    double keyframe_angle_thresh_rad_{10.0 * M_PI / 180.0};
    double keyframe_time_thresh_{1.0};
    double keyframe_cloud_voxel_size_{0.5};

    bool loop_detect_enabled_{true};
    double loop_detect_frequency_{1.0};
    int loop_exclude_recent_num_{30};
    int loop_candidate_num_{10};
    double loop_score_threshold_{0.15};
    double floor_height_threshold_{4.0};

    double sc_lidar_height_{0.0};
    double sc_max_radius_{80.0};
    int sc_num_rings_{20};
    int sc_num_sectors_{60};
    int sc_tree_depth_{20};
    double sc_search_ratio_{0.1};

    double loop_icp_max_corr_dist_{2.0};
    double loop_icp_fitness_threshold_{0.4};
    int loop_icp_max_iterations_{50};
    int loop_submap_nearby_frames_{10};

    double odom_trans_noise_{0.1};
    double odom_rot_noise_{0.05};
    double loop_noise_{0.5};

    double optimized_map_publish_frequency_{0.2};
    double optimized_map_voxel_size_{0.2};
    bool save_optimized_map_{true};
    std::string save_directory_;

    // State
    mutable std::mutex data_mutex_;
    std::vector<std::shared_ptr<KeyFrame>> keyframes_;
    std::vector<std::pair<int, int>> loop_pairs_;
    std::set<std::pair<int, int>> loop_pair_set_;
    std::optional<Eigen::Matrix4f> latest_odom_pose_;
    std::optional<rclcpp::Time> latest_odom_stamp_;
    std::string latest_odom_frame_;
    std::string latest_base_frame_;
    std::optional<Eigen::Matrix4f> pending_keyframe_pose_;
    std::optional<rclcpp::Time> pending_keyframe_stamp_;
    std::optional<Eigen::Matrix4f> last_keyframe_pose_;
    std::optional<rclcpp::Time> last_keyframe_stamp_;
    std::optional<rclcpp::Time> last_loop_detect_stamp_;
    size_t odom_count_{0};
    int next_keyframe_id_{0};
    size_t last_saved_keyframe_count_{0};
    size_t last_saved_loop_count_{0};

    // Back end
    dddmr_sc_pgo::ScanContextManager sc_manager_;
    pcl::VoxelGrid<pcl::PointXYZI> keyframe_downsample_filter_;
    std::unique_ptr<gtsam::ISAM2> isam_;
    gtsam::NonlinearFactorGraph new_factors_;
    gtsam::Values new_values_;
    gtsam::Values optimized_values_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try
    {
        rclcpp::spin(std::make_shared<SCPGONode>());
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("sc_pgo_node"), "SC-PGO exited: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
