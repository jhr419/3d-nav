#include <algorithm>
#include <cmath>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "sc_pgo/common.hpp"
#include "sc_pgo/scan_context/scan_context.hpp"

namespace sc_pgo
{
namespace
{

template<typename MsgT>
double stampSeconds(const MsgT & msg)
{
  return rclcpp::Time(msg.header.stamp).seconds();
}

template<typename MsgT>
double stampSeconds(const std::shared_ptr<MsgT> & msg)
{
  return stampSeconds(*msg);
}

std::string ensureTrailingSlash(std::string path)
{
  if (path.empty()) {
    return path;
  }
  if (path.back() != '/') {
    path.push_back('/');
  }
  return path;
}

std::string padZeros(const int value, const int num_digits = 6)
{
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << value;
  return out.str();
}

Eigen::Affine3f poseToAffine3f(const Pose6D & pose)
{
  return pcl::getTransformation(
    static_cast<float>(pose.x),
    static_cast<float>(pose.y),
    static_cast<float>(pose.z),
    static_cast<float>(pose.roll),
    static_cast<float>(pose.pitch),
    static_cast<float>(pose.yaw));
}

Pose6D affine3fToPose(const Eigen::Affine3f & affine)
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float roll = 0.0f;
  float pitch = 0.0f;
  float yaw = 0.0f;
  pcl::getTranslationAndEulerAngles(affine, x, y, z, roll, pitch, yaw);
  return Pose6D{x, y, z, roll, pitch, yaw};
}

gtsam::Pose3 pose6DToGtsamPose3(const Pose6D & pose)
{
  return gtsam::Pose3(
    gtsam::Rot3::RzRyRx(pose.roll, pose.pitch, pose.yaw),
    gtsam::Point3(pose.x, pose.y, pose.z));
}

geometry_msgs::msg::Quaternion rpyToQuaternion(
  const double roll,
  const double pitch,
  const double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

Pose6D odomToPose6D(const nav_msgs::msg::Odometry & odom)
{
  const auto & position = odom.pose.pose.position;
  const auto & orientation = odom.pose.pose.orientation;

  tf2::Quaternion q(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w);
  q.normalize();

  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  return Pose6D{position.x, position.y, position.z, roll, pitch, yaw};
}

Pose6D diffTransformation(const Pose6D & previous, const Pose6D & current)
{
  const Eigen::Affine3f previous_tf = poseToAffine3f(previous);
  const Eigen::Affine3f current_tf = poseToAffine3f(current);
  Eigen::Affine3f delta_tf;
  delta_tf.matrix() = previous_tf.matrix().inverse() * current_tf.matrix();

  Pose6D delta = affine3fToPose(delta_tf);
  delta.x = std::abs(delta.x);
  delta.y = std::abs(delta.y);
  delta.z = std::abs(delta.z);
  delta.roll = std::abs(delta.roll);
  delta.pitch = std::abs(delta.pitch);
  delta.yaw = std::abs(delta.yaw);
  return delta;
}

pcl::PointCloud<PointType>::Ptr transformCloud(
  const pcl::PointCloud<PointType>::Ptr & cloud,
  const Pose6D & pose)
{
  pcl::PointCloud<PointType>::Ptr transformed(new pcl::PointCloud<PointType>());
  if (!cloud || cloud->empty()) {
    return transformed;
  }
  pcl::transformPointCloud(*cloud, *transformed, poseToAffine3f(pose));
  return transformed;
}

void removeNanPoints(pcl::PointCloud<PointType>::Ptr & cloud)
{
  if (!cloud) {
    return;
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
}

void downsampleCloud(
  const pcl::PointCloud<PointType>::Ptr & cloud,
  const double leaf_size)
{
  if (!cloud || cloud->empty() || leaf_size <= 0.0) {
    return;
  }

  pcl::VoxelGrid<PointType> voxel;
  voxel.setLeafSize(
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size),
    static_cast<float>(leaf_size));
  voxel.setInputCloud(cloud);
  pcl::PointCloud<PointType> filtered;
  voxel.filter(filtered);
  *cloud = filtered;
}

gtsam::noiseModel::Diagonal::shared_ptr makeDiagonalNoise(
  const std::vector<double> & variances,
  const std::vector<double> & fallback)
{
  const auto & source = variances.size() == fallback.size() ? variances : fallback;
  gtsam::Vector vector(static_cast<int>(fallback.size()));
  for (size_t i = 0; i < fallback.size(); ++i) {
    vector(static_cast<int>(i)) = source[i];
  }
  return gtsam::noiseModel::Diagonal::Variances(vector);
}

gtsam::noiseModel::Base::shared_ptr makeRobustNoise(
  const std::vector<double> & variances,
  const std::vector<double> & fallback)
{
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Cauchy::Create(1.0),
    makeDiagonalNoise(variances, fallback));
}

}  // namespace

class ScPgoNode : public rclcpp::Node
{
public:
  ScPgoNode()
  : Node("sc_pgo_node")
  {
    declareAndLoadParameters();
    configureOutputDirectory();
    configureNoises();
    configureScanContext();

    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = isam_relinearize_threshold_;
    parameters.relinearizeSkip = isam_relinearize_skip_;
    isam_ = std::make_unique<gtsam::ISAM2>(parameters);

    input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    process_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    loop_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    icp_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    isam_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    viz_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    createRosInterfaces();
    createTimers();

    RCLCPP_INFO(
      get_logger(),
      "SC-PGO ROS2 node ready: odom=%s cloud=%s sc_cloud=%s save_dir=%s",
      odom_topic_.c_str(),
      cloud_topic_.c_str(),
      effectiveScanContextTopic().c_str(),
      save_directory_.c_str());
  }

  ~ScPgoNode() override
  {
    if (time_save_stream_.is_open()) {
      time_save_stream_.close();
    }
  }

private:
  struct GpsMatch
  {
    bool valid = false;
    sensor_msgs::msg::NavSatFix msg;
  };

  void declareAndLoadParameters()
  {
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/cloud_registered_body");
    scan_context_cloud_topic_ =
      declare_parameter<std::string>("scan_context_cloud_topic", "");
    gps_topic_ = declare_parameter<std::string>("gps_topic", "/gps/fix");
    use_gps_ = declare_parameter<bool>("use_gps", false);

    map_frame_ = declare_parameter<std::string>("map_frame", "camera_init");
    pgo_child_frame_ = declare_parameter<std::string>("pgo_child_frame", "aft_pgo");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    topic_queue_size_ = declare_parameter<int>("topic_queue_size", 100);
    max_buffer_size_ = declare_parameter<int>("max_buffer_size", 300);
    sync_time_tolerance_s_ = declare_parameter<double>("sync_time_tolerance_s", 0.08);
    gps_time_tolerance_s_ = declare_parameter<double>("gps_time_tolerance_s", 0.1);

    keyframe_meter_gap_ = declare_parameter<double>("keyframe_meter_gap", 0.5);
    keyframe_deg_gap_ = declare_parameter<double>("keyframe_deg_gap", 10.0);
    keyframe_rad_gap_ = deg2rad(keyframe_deg_gap_);
    keyframe_min_points_ = declare_parameter<int>("keyframe_min_points", 100);

    scan_context_leaf_size_ = declare_parameter<double>("scan_context_leaf_size", 0.4);
    map_visualization_leaf_size_ =
      declare_parameter<double>("map_visualization_leaf_size", 0.4);
    map_keyframe_skip_ = declare_parameter<int>("map_keyframe_skip", 2);

    sc_dist_thres_ = declare_parameter<double>("sc_dist_thres", 0.3);
    sc_max_radius_ = declare_parameter<double>("sc_max_radius", 80.0);
    sc_lidar_height_ = declare_parameter<double>("sc_lidar_height", 2.0);
    sc_num_exclude_recent_ = declare_parameter<int>("sc_num_exclude_recent", 30);
    sc_num_candidates_ = declare_parameter<int>("sc_num_candidates", 3);
    sc_tree_making_period_ = declare_parameter<int>("sc_tree_making_period", 30);
    sc_debug_timing_ = declare_parameter<bool>("sc_debug_timing", false);
    enable_loop_closure_ = declare_parameter<bool>("enable_loop_closure", true);

    loop_detection_period_s_ = declare_parameter<double>("loop_detection_period_s", 1.0);
    loop_icp_period_s_ = declare_parameter<double>("loop_icp_period_s", 0.05);
    loop_icp_history_search_num_ =
      declare_parameter<int>("loop_icp_history_search_num", 25);
    loop_icp_max_correspondence_distance_ =
      declare_parameter<double>("loop_icp_max_correspondence_distance", 150.0);
    loop_icp_max_iterations_ = declare_parameter<int>("loop_icp_max_iterations", 100);
    loop_icp_transformation_epsilon_ =
      declare_parameter<double>("loop_icp_transformation_epsilon", 1.0e-6);
    loop_icp_euclidean_fitness_epsilon_ =
      declare_parameter<double>("loop_icp_euclidean_fitness_epsilon", 1.0e-6);
    loop_icp_fitness_score_threshold_ =
      declare_parameter<double>("loop_icp_fitness_score_threshold", 0.3);
    loop_queue_warn_threshold_ = declare_parameter<int>("loop_queue_warn_threshold", 30);

    process_period_s_ = declare_parameter<double>("process_period_s", 0.005);
    isam_update_period_s_ = declare_parameter<double>("isam_update_period_s", 1.0);
    path_publish_period_s_ = declare_parameter<double>("path_publish_period_s", 0.1);
    map_publish_period_s_ = declare_parameter<double>("map_publish_period_s", 10.0);
    isam_relinearize_threshold_ =
      declare_parameter<double>("isam_relinearize_threshold", 0.01);
    isam_relinearize_skip_ = declare_parameter<int>("isam_relinearize_skip", 1);

    save_directory_ = ensureTrailingSlash(
      declare_parameter<std::string>("save_directory", "/tmp/sc_pgo"));
    save_scans_ = declare_parameter<bool>("save_scans", true);
    reset_scan_directory_on_start_ =
      declare_parameter<bool>("reset_scan_directory_on_start", true);

    prior_variances_ = declare_parameter<std::vector<double>>(
      "prior_variances",
      {1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12});
    odom_variances_ = declare_parameter<std::vector<double>>(
      "odom_variances",
      {1.0e-6, 1.0e-6, 1.0e-6, 1.0e-4, 1.0e-4, 1.0e-4});
    loop_variances_ = declare_parameter<std::vector<double>>(
      "loop_variances",
      {0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    gps_variances_ = declare_parameter<std::vector<double>>(
      "gps_variances",
      {1.0e9, 1.0e9, 250.0});

    topic_queue_size_ = std::max(1, topic_queue_size_);
    max_buffer_size_ = std::max(10, max_buffer_size_);
    map_keyframe_skip_ = std::max(1, map_keyframe_skip_);
  }

  void configureOutputDirectory()
  {
    if (save_directory_.empty()) {
      return;
    }

    optimized_kitti_file_ = save_directory_ + "optimized_poses.txt";
    odom_kitti_file_ = save_directory_ + "odom_poses.txt";
    scans_directory_ = save_directory_ + "Scans/";

    try {
      std::filesystem::create_directories(save_directory_);
      if (save_scans_) {
        if (reset_scan_directory_on_start_) {
          std::filesystem::remove_all(scans_directory_);
        }
        std::filesystem::create_directories(scans_directory_);
      }
      time_save_stream_.open(save_directory_ + "times.txt", std::fstream::out);
      time_save_stream_.precision(std::numeric_limits<double>::max_digits10);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to prepare SC-PGO save directory '%s': %s",
        save_directory_.c_str(),
        ex.what());
      save_scans_ = false;
    }
  }

  void configureNoises()
  {
    prior_noise_ = makeDiagonalNoise(
      prior_variances_,
      {1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12, 1.0e-12});
    odom_noise_ = makeDiagonalNoise(
      odom_variances_,
      {1.0e-6, 1.0e-6, 1.0e-6, 1.0e-4, 1.0e-4, 1.0e-4});
    robust_loop_noise_ = makeRobustNoise(
      loop_variances_,
      {0.5, 0.5, 0.5, 0.5, 0.5, 0.5});
    robust_gps_noise_ = makeRobustNoise(
      gps_variances_,
      {1.0e9, 1.0e9, 250.0});
  }

  void configureScanContext()
  {
    sc_manager_.setSCdistThres(sc_dist_thres_);
    sc_manager_.setMaximumRadius(sc_max_radius_);
    sc_manager_.setLidarHeight(sc_lidar_height_);
    sc_manager_.setNumExcludeRecent(sc_num_exclude_recent_);
    sc_manager_.setNumCandidatesFromTree(sc_num_candidates_);
    sc_manager_.setTreeMakingPeriod(sc_tree_making_period_);
    sc_manager_.setDebugTiming(sc_debug_timing_);
  }

  std::string effectiveScanContextTopic() const
  {
    return scan_context_cloud_topic_.empty() ? cloud_topic_ : scan_context_cloud_topic_;
  }

  bool useSeparateScanContextCloud() const
  {
    return !scan_context_cloud_topic_.empty() && scan_context_cloud_topic_ != cloud_topic_;
  }

  void createRosInterfaces()
  {
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = input_group_;

    const rclcpp::QoS qos{rclcpp::KeepLast(topic_queue_size_)};

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      qos,
      std::bind(&ScPgoNode::odomCallback, this, std::placeholders::_1),
      sub_options);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      qos,
      std::bind(&ScPgoNode::cloudCallback, this, std::placeholders::_1),
      sub_options);

    if (useSeparateScanContextCloud()) {
      scan_context_cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        scan_context_cloud_topic_,
        qos,
        std::bind(&ScPgoNode::scanContextCloudCallback, this, std::placeholders::_1),
        sub_options);
    }

    if (use_gps_) {
      gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_,
        qos,
        std::bind(&ScPgoNode::gpsCallback, this, std::placeholders::_1),
        sub_options);
    }

    pgo_odom_pub_ =
      create_publisher<nav_msgs::msg::Odometry>("/aft_pgo_odom", topic_queue_size_);
    pgo_path_pub_ =
      create_publisher<nav_msgs::msg::Path>("/aft_pgo_path", topic_queue_size_);
    pgo_map_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/aft_pgo_map", 1);
    loop_scan_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/loop_scan_local", topic_queue_size_);
    loop_submap_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>("/loop_submap_local", topic_queue_size_);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

  void createTimers()
  {
    process_timer_ = create_wall_timer(
      toDuration(process_period_s_),
      std::bind(&ScPgoNode::processPoseGraph, this),
      process_group_);

    loop_detection_timer_ = create_wall_timer(
      toDuration(loop_detection_period_s_),
      std::bind(&ScPgoNode::performScanContextLoopClosure, this),
      loop_group_);

    icp_timer_ = create_wall_timer(
      toDuration(loop_icp_period_s_),
      std::bind(&ScPgoNode::processIcpQueue, this),
      icp_group_);

    isam_timer_ = create_wall_timer(
      toDuration(isam_update_period_s_),
      std::bind(&ScPgoNode::processIsamUpdate, this),
      isam_group_);

    path_timer_ = create_wall_timer(
      toDuration(path_publish_period_s_),
      std::bind(&ScPgoNode::publishPath, this),
      viz_group_);

    map_timer_ = create_wall_timer(
      toDuration(map_publish_period_s_),
      std::bind(&ScPgoNode::publishMap, this),
      viz_group_);
  }

  std::chrono::nanoseconds toDuration(const double seconds) const
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(std::max(0.001, seconds)));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    odom_buffer_.push_back(msg);
    trimBuffer(odom_buffer_);
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    cloud_buffer_.push_back(msg);
    trimBuffer(cloud_buffer_);
  }

  void scanContextCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    scan_context_cloud_buffer_.push_back(msg);
    trimBuffer(scan_context_cloud_buffer_);
  }

  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    gps_buffer_.push_back(msg);
    trimBuffer(gps_buffer_);
  }

  template<typename MsgT>
  void trimBuffer(std::deque<std::shared_ptr<MsgT>> & buffer)
  {
    while (static_cast<int>(buffer.size()) > max_buffer_size_) {
      buffer.pop_front();
    }
  }

  void processPoseGraph()
  {
    while (rclcpp::ok()) {
      nav_msgs::msg::Odometry::SharedPtr odom_msg;
      sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg;
      sensor_msgs::msg::PointCloud2::SharedPtr scan_context_cloud_msg;
      GpsMatch gps_match;

      if (!popSynchronizedMeasurements(
          odom_msg,
          cloud_msg,
          scan_context_cloud_msg,
          gps_match))
      {
        return;
      }

      handleSynchronizedMeasurements(
        *odom_msg,
        *cloud_msg,
        *scan_context_cloud_msg,
        gps_match);
    }
  }

  bool popSynchronizedMeasurements(
    nav_msgs::msg::Odometry::SharedPtr & odom_msg,
    sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
    sensor_msgs::msg::PointCloud2::SharedPtr & scan_context_cloud_msg,
    GpsMatch & gps_match)
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (odom_buffer_.empty() || cloud_buffer_.empty()) {
      return false;
    }

    while (!cloud_buffer_.empty()) {
      const double cloud_time = stampSeconds(cloud_buffer_.front());

      while (!odom_buffer_.empty() &&
        stampSeconds(odom_buffer_.front()) < cloud_time - sync_time_tolerance_s_)
      {
        odom_buffer_.pop_front();
      }

      if (odom_buffer_.empty()) {
        return false;
      }

      const double odom_time = stampSeconds(odom_buffer_.front());
      if (odom_time > cloud_time + sync_time_tolerance_s_) {
        cloud_buffer_.pop_front();
        continue;
      }

      sensor_msgs::msg::PointCloud2::SharedPtr matched_scan_context_cloud =
        cloud_buffer_.front();

      if (useSeparateScanContextCloud()) {
        while (!scan_context_cloud_buffer_.empty() &&
          stampSeconds(scan_context_cloud_buffer_.front()) < cloud_time - sync_time_tolerance_s_)
        {
          scan_context_cloud_buffer_.pop_front();
        }

        if (scan_context_cloud_buffer_.empty()) {
          return false;
        }

        if (stampSeconds(scan_context_cloud_buffer_.front()) >
          cloud_time + sync_time_tolerance_s_)
        {
          cloud_buffer_.pop_front();
          odom_buffer_.pop_front();
          continue;
        }

        matched_scan_context_cloud = scan_context_cloud_buffer_.front();
        scan_context_cloud_buffer_.pop_front();
      }

      odom_msg = odom_buffer_.front();
      cloud_msg = cloud_buffer_.front();
      scan_context_cloud_msg = matched_scan_context_cloud;
      odom_buffer_.pop_front();
      cloud_buffer_.pop_front();
      gps_match = popGpsForTime(cloud_time);
      return true;
    }

    return false;
  }

  GpsMatch popGpsForTime(const double stamp)
  {
    GpsMatch match;
    if (!use_gps_) {
      return match;
    }

    while (!gps_buffer_.empty()) {
      const auto gps = gps_buffer_.front();
      const double gps_time = stampSeconds(gps);
      if (std::abs(gps_time - stamp) < gps_time_tolerance_s_) {
        match.valid = true;
        match.msg = *gps;
        break;
      }
      if (gps_time > stamp + gps_time_tolerance_s_) {
        break;
      }
      gps_buffer_.pop_front();
    }

    return match;
  }

  void handleSynchronizedMeasurements(
    const nav_msgs::msg::Odometry & odom_msg,
    const sensor_msgs::msg::PointCloud2 & cloud_msg,
    const sensor_msgs::msg::PointCloud2 & scan_context_cloud_msg,
    const GpsMatch & gps_match)
  {
    pcl::PointCloud<PointType>::Ptr keyframe_cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(cloud_msg, *keyframe_cloud);
    removeNanPoints(keyframe_cloud);
    if (static_cast<int>(keyframe_cloud->size()) < keyframe_min_points_) {
      return;
    }

    const Pose6D current_pose = odomToPose6D(odom_msg);
    if (!shouldAddKeyframe(current_pose)) {
      return;
    }

    pcl::PointCloud<PointType>::Ptr keyframe_cloud_ds(new pcl::PointCloud<PointType>());
    *keyframe_cloud_ds = *keyframe_cloud;
    downsampleCloud(keyframe_cloud_ds, scan_context_leaf_size_);

    pcl::PointCloud<PointType>::Ptr scan_context_cloud(new pcl::PointCloud<PointType>());
    if (&scan_context_cloud_msg == &cloud_msg) {
      scan_context_cloud = keyframe_cloud;
    } else {
      pcl::fromROSMsg(scan_context_cloud_msg, *scan_context_cloud);
      removeNanPoints(scan_context_cloud);
    }

    pcl::PointCloud<PointType>::Ptr scan_context_cloud_ds(new pcl::PointCloud<PointType>());
    *scan_context_cloud_ds = *scan_context_cloud;
    downsampleCloud(scan_context_cloud_ds, scan_context_leaf_size_);

    if (static_cast<int>(scan_context_cloud_ds->size()) < keyframe_min_points_) {
      return;
    }

    if (use_gps_ && gps_match.valid && !gps_offset_initialized_) {
      gps_altitude_init_offset_ = gps_match.msg.altitude;
      gps_offset_initialized_ = true;
    }

    int current_node_idx = 0;
    int previous_node_idx = -1;
    Pose6D previous_pose;

    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      keyframe_laser_clouds_.push_back(keyframe_cloud_ds);
      keyframe_poses_.push_back(current_pose);
      keyframe_poses_updated_.push_back(current_pose);
      keyframe_times_.push_back(rclcpp::Time(cloud_msg.header.stamp).seconds());
      sc_manager_.makeAndSaveScancontextAndKeys(*scan_context_cloud_ds);

      current_node_idx = static_cast<int>(keyframe_poses_.size()) - 1;
      previous_node_idx = current_node_idx - 1;
      if (previous_node_idx >= 0) {
        previous_pose = keyframe_poses_[previous_node_idx];
      }
      map_redraw_needed_ = true;
    }

    addKeyframeFactors(current_node_idx, previous_node_idx, previous_pose, current_pose, gps_match);
    saveKeyframeArtifacts(current_node_idx, keyframe_cloud, stampSeconds(cloud_msg));

    RCLCPP_INFO(
      get_logger(),
      "SC-PGO keyframe %d: points=%zu sc_points=%zu",
      current_node_idx,
      keyframe_cloud_ds->size(),
      scan_context_cloud_ds->size());
  }

  bool shouldAddKeyframe(const Pose6D & current_pose)
  {
    bool add_keyframe = false;

    if (!have_last_odom_pose_) {
      add_keyframe = true;
    } else {
      const Pose6D delta = diffTransformation(last_odom_pose_, current_pose);
      const double delta_translation =
        std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);

      translation_accumulated_ += delta_translation;
      rotation_accumulated_ += delta.roll + delta.pitch + delta.yaw;

      add_keyframe =
        translation_accumulated_ > keyframe_meter_gap_ ||
        rotation_accumulated_ > keyframe_rad_gap_;
    }

    last_odom_pose_ = current_pose;
    have_last_odom_pose_ = true;

    if (add_keyframe) {
      translation_accumulated_ = 0.0;
      rotation_accumulated_ = 0.0;
    }

    return add_keyframe;
  }

  void addKeyframeFactors(
    const int current_node_idx,
    const int previous_node_idx,
    const Pose6D & previous_pose,
    const Pose6D & current_pose,
    const GpsMatch & gps_match)
  {
    std::lock_guard<std::mutex> lock(posegraph_mutex_);

    if (!posegraph_made_) {
      const gtsam::Pose3 pose_origin = pose6DToGtsamPose3(current_pose);
      graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
        current_node_idx,
        pose_origin,
        prior_noise_));
      initial_estimate_.insert(current_node_idx, pose_origin);
      posegraph_made_ = true;
      RCLCPP_INFO(get_logger(), "Pose graph prior node %d added", current_node_idx);
      return;
    }

    if (previous_node_idx < 0) {
      return;
    }

    const gtsam::Pose3 pose_from = pose6DToGtsamPose3(previous_pose);
    const gtsam::Pose3 pose_to = pose6DToGtsamPose3(current_pose);
    graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      previous_node_idx,
      current_node_idx,
      pose_from.between(pose_to),
      odom_noise_));

    if (use_gps_ && gps_match.valid && gps_offset_initialized_) {
      const double altitude_offset = gps_match.msg.altitude - gps_altitude_init_offset_;
      double x = 0.0;
      double y = 0.0;
      {
        std::lock_guard<std::mutex> recent_lock(recent_pose_mutex_);
        x = recent_optimized_x_;
        y = recent_optimized_y_;
      }
      const gtsam::Point3 gps_constraint(x, y, altitude_offset);
      graph_.add(gtsam::GPSFactor(current_node_idx, gps_constraint, robust_gps_noise_));
      RCLCPP_INFO(get_logger(), "GPS factor added at node %d", current_node_idx);
    }

    initial_estimate_.insert(current_node_idx, pose_to);
  }

  void saveKeyframeArtifacts(
    const int current_node_idx,
    const pcl::PointCloud<PointType>::Ptr & keyframe_cloud,
    const double stamp)
  {
    if (!save_scans_ || scans_directory_.empty()) {
      return;
    }

    const std::string file_name = scans_directory_ + padZeros(current_node_idx) + ".pcd";
    pcl::io::savePCDFileBinary(file_name, *keyframe_cloud);
    if (time_save_stream_.is_open()) {
      time_save_stream_ << stamp << std::endl;
    }
  }

  void performScanContextLoopClosure()
  {
    if (!enable_loop_closure_) {
      return;
    }

    int previous_node_idx = -1;
    int current_node_idx = -1;

    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      if (static_cast<int>(keyframe_poses_.size()) < sc_manager_.numExcludeRecent()) {
        return;
      }

      const auto detect_result = sc_manager_.detectLoopClosureID();
      previous_node_idx = detect_result.first;
      current_node_idx = static_cast<int>(keyframe_poses_.size()) - 1;
    }

    if (previous_node_idx == -1) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(loop_queue_mutex_);
      sc_loop_icp_buffer_.push_back(std::make_pair(previous_node_idx, current_node_idx));
    }

    RCLCPP_INFO(
      get_logger(),
      "Loop detected between keyframes %d and %d",
      previous_node_idx,
      current_node_idx);
  }

  void processIcpQueue()
  {
    std::pair<int, int> loop_pair;
    {
      std::lock_guard<std::mutex> lock(loop_queue_mutex_);
      if (sc_loop_icp_buffer_.empty()) {
        return;
      }
      if (static_cast<int>(sc_loop_icp_buffer_.size()) > loop_queue_warn_threshold_) {
        RCLCPP_WARN(
          get_logger(),
          "Too many loop closure candidates are waiting for ICP: %zu",
          sc_loop_icp_buffer_.size());
      }
      loop_pair = sc_loop_icp_buffer_.front();
      sc_loop_icp_buffer_.pop_front();
    }

    const auto relative_pose = doIcpVirtualRelative(loop_pair.first, loop_pair.second);
    if (!relative_pose) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(posegraph_mutex_);
      graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        loop_pair.first,
        loop_pair.second,
        relative_pose.value(),
        robust_loop_noise_));
    }
  }

  std::optional<gtsam::Pose3> doIcpVirtualRelative(
    const int loop_keyframe_idx,
    const int current_keyframe_idx)
  {
    pcl::PointCloud<PointType>::Ptr current_keyframe_cloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr target_keyframe_cloud(new pcl::PointCloud<PointType>());

    loopFindNearKeyframesCloud(current_keyframe_cloud, current_keyframe_idx, 0, loop_keyframe_idx);
    loopFindNearKeyframesCloud(
      target_keyframe_cloud,
      loop_keyframe_idx,
      loop_icp_history_search_num_,
      loop_keyframe_idx);

    if (current_keyframe_cloud->empty() || target_keyframe_cloud->empty()) {
      return std::nullopt;
    }

    publishLoopCloud(loop_scan_pub_, current_keyframe_cloud);
    publishLoopCloud(loop_submap_pub_, target_keyframe_cloud);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(loop_icp_max_correspondence_distance_);
    icp.setMaximumIterations(loop_icp_max_iterations_);
    icp.setTransformationEpsilon(loop_icp_transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(loop_icp_euclidean_fitness_epsilon_);
    icp.setRANSACIterations(0);
    icp.setInputSource(current_keyframe_cloud);
    icp.setInputTarget(target_keyframe_cloud);

    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    if (!icp.hasConverged() ||
      icp.getFitnessScore() > loop_icp_fitness_score_threshold_)
    {
      RCLCPP_WARN(
        get_logger(),
        "[SC loop] ICP fitness test failed (%f > %f). Reject this loop.",
        icp.getFitnessScore(),
        loop_icp_fitness_score_threshold_);
      return std::nullopt;
    }

    RCLCPP_INFO(
      get_logger(),
      "[SC loop] ICP fitness test passed (%f < %f). Add this loop.",
      icp.getFitnessScore(),
      loop_icp_fitness_score_threshold_);

    const Eigen::Affine3f correction_lidar_frame(icp.getFinalTransformation());
    const Pose6D correction = affine3fToPose(correction_lidar_frame);
    const gtsam::Pose3 pose_from(
      gtsam::Rot3::RzRyRx(correction.roll, correction.pitch, correction.yaw),
      gtsam::Point3(correction.x, correction.y, correction.z));
    const gtsam::Pose3 pose_to(
      gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0),
      gtsam::Point3(0.0, 0.0, 0.0));

    return pose_from.between(pose_to);
  }

  void loopFindNearKeyframesCloud(
    pcl::PointCloud<PointType>::Ptr & near_keyframes,
    const int key,
    const int submap_size,
    const int root_idx)
  {
    near_keyframes->clear();
    std::lock_guard<std::mutex> lock(keyframe_mutex_);

    if (root_idx < 0 || root_idx >= static_cast<int>(keyframe_poses_updated_.size())) {
      return;
    }

    for (int i = -submap_size; i <= submap_size; ++i) {
      const int key_near = key + i;
      if (key_near < 0 || key_near >= static_cast<int>(keyframe_laser_clouds_.size())) {
        continue;
      }
      *near_keyframes += *transformCloud(
        keyframe_laser_clouds_[key_near],
        keyframe_poses_updated_[root_idx]);
    }

    downsampleCloud(near_keyframes, scan_context_leaf_size_);
  }

  void publishLoopCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & publisher,
    const pcl::PointCloud<PointType>::Ptr & cloud) const
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    publisher->publish(msg);
  }

  void processIsamUpdate()
  {
    if (!posegraph_made_) {
      return;
    }

    gtsam::Values estimate;
    bool updated = false;

    {
      std::lock_guard<std::mutex> lock(posegraph_mutex_);
      if (graph_.size() == 0 && initial_estimate_.size() == 0) {
        return;
      }

      isam_->update(graph_, initial_estimate_);
      isam_->update();
      graph_.resize(0);
      initial_estimate_.clear();
      isam_current_estimate_ = isam_->calculateEstimate();
      estimate = isam_current_estimate_;
      updated = true;
    }

    if (!updated) {
      return;
    }

    updatePoses(estimate);
    saveOptimizedVerticesKITTIformat(estimate, optimized_kitti_file_);
    saveOdometryVerticesKITTIformat(odom_kitti_file_);

    RCLCPP_INFO(get_logger(), "Running iSAM2 optimization, variables=%zu", estimate.size());
  }

  void updatePoses(const gtsam::Values & estimate)
  {
    if (estimate.empty()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      for (size_t idx = 0; idx < keyframe_poses_updated_.size(); ++idx) {
        if (!estimate.exists(idx)) {
          continue;
        }
        const gtsam::Pose3 pose = estimate.at<gtsam::Pose3>(idx);
        auto & target = keyframe_poses_updated_[idx];
        target.x = pose.translation().x();
        target.y = pose.translation().y();
        target.z = pose.translation().z();
        target.roll = pose.rotation().roll();
        target.pitch = pose.rotation().pitch();
        target.yaw = pose.rotation().yaw();
      }

      recent_idx_updated_ = static_cast<int>(keyframe_poses_updated_.size()) - 1;
      map_redraw_needed_ = true;
    }

    const int last_idx = static_cast<int>(estimate.size()) - 1;
    if (estimate.exists(last_idx)) {
      const gtsam::Pose3 last_optimized_pose = estimate.at<gtsam::Pose3>(last_idx);
      std::lock_guard<std::mutex> recent_lock(recent_pose_mutex_);
      recent_optimized_x_ = last_optimized_pose.translation().x();
      recent_optimized_y_ = last_optimized_pose.translation().y();
    }
  }

  void publishPath()
  {
    nav_msgs::msg::Odometry latest_odom;
    nav_msgs::msg::Path path;
    bool have_odom = false;

    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      if (recent_idx_updated_ < 0 ||
        keyframe_poses_updated_.empty() ||
        keyframe_times_.empty())
      {
        return;
      }

      path.header.frame_id = map_frame_;
      path.header.stamp = now();

      const int max_idx = std::min(
        recent_idx_updated_,
        static_cast<int>(keyframe_poses_updated_.size()) - 1);

      for (int idx = 0; idx <= max_idx; ++idx) {
        const Pose6D & pose = keyframe_poses_updated_[idx];
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = map_frame_;
        odom.child_frame_id = pgo_child_frame_;
        const auto stamp_ns = static_cast<int64_t>(keyframe_times_[idx] * 1.0e9);
        odom.header.stamp =
          static_cast<builtin_interfaces::msg::Time>(rclcpp::Time(stamp_ns, RCL_ROS_TIME));
        odom.pose.pose.position.x = pose.x;
        odom.pose.pose.position.y = pose.y;
        odom.pose.pose.position.z = pose.z;
        odom.pose.pose.orientation = rpyToQuaternion(pose.roll, pose.pitch, pose.yaw);

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.pose = odom.pose.pose;
        path.poses.push_back(pose_stamped);

        latest_odom = odom;
        have_odom = true;
      }
    }

    if (!have_odom) {
      return;
    }

    pgo_odom_pub_->publish(latest_odom);
    pgo_path_pub_->publish(path);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = latest_odom.header.stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = pgo_child_frame_;
      transform.transform.translation.x = latest_odom.pose.pose.position.x;
      transform.transform.translation.y = latest_odom.pose.pose.position.y;
      transform.transform.translation.z = latest_odom.pose.pose.position.z;
      transform.transform.rotation = latest_odom.pose.pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }
  }

  void publishMap()
  {
    pcl::PointCloud<PointType>::Ptr map_cloud(new pcl::PointCloud<PointType>());

    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      if (recent_idx_updated_ < 0 || keyframe_laser_clouds_.empty()) {
        return;
      }

      const int max_idx = std::min(
        recent_idx_updated_,
        static_cast<int>(keyframe_laser_clouds_.size()) - 1);

      for (int idx = 0; idx <= max_idx; ++idx) {
        if (idx % map_keyframe_skip_ != 0) {
          continue;
        }
        *map_cloud += *transformCloud(keyframe_laser_clouds_[idx], keyframe_poses_updated_[idx]);
      }
      map_redraw_needed_ = false;
    }

    downsampleCloud(map_cloud, map_visualization_leaf_size_);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*map_cloud, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    pgo_map_pub_->publish(msg);
  }

  void saveOdometryVerticesKITTIformat(const std::string & file_name)
  {
    if (file_name.empty()) {
      return;
    }

    std::vector<Pose6D> poses;
    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      poses = keyframe_poses_;
    }

    std::fstream stream(file_name.c_str(), std::fstream::out);
    for (const auto & pose6d : poses) {
      writePoseKittiLine(stream, pose6DToGtsamPose3(pose6d));
    }
  }

  void saveOptimizedVerticesKITTIformat(
    const gtsam::Values & estimates,
    const std::string & file_name)
  {
    if (file_name.empty()) {
      return;
    }

    size_t pose_count = 0;
    {
      std::lock_guard<std::mutex> lock(keyframe_mutex_);
      pose_count = keyframe_poses_.size();
    }

    std::fstream stream(file_name.c_str(), std::fstream::out);
    for (size_t idx = 0; idx < pose_count; ++idx) {
      if (!estimates.exists(idx)) {
        continue;
      }
      writePoseKittiLine(stream, estimates.at<gtsam::Pose3>(idx));
    }
  }

  void writePoseKittiLine(std::fstream & stream, const gtsam::Pose3 & pose) const
  {
    const gtsam::Point3 t = pose.translation();
    const gtsam::Matrix3 rotation = pose.rotation().matrix();

    stream << rotation(0, 0) << " " << rotation(0, 1) << " " << rotation(0, 2) << " " << t.x()
      << " " << rotation(1, 0) << " " << rotation(1, 1) << " " << rotation(1, 2) << " " << t.y()
      << " " << rotation(2, 0) << " " << rotation(2, 1) << " " << rotation(2, 2) << " " << t.z()
      << std::endl;
  }

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string scan_context_cloud_topic_;
  std::string gps_topic_;
  bool use_gps_ = false;

  std::string map_frame_;
  std::string pgo_child_frame_;
  bool publish_tf_ = true;

  int topic_queue_size_ = 100;
  int max_buffer_size_ = 300;
  double sync_time_tolerance_s_ = 0.08;
  double gps_time_tolerance_s_ = 0.1;

  double keyframe_meter_gap_ = 0.5;
  double keyframe_deg_gap_ = 10.0;
  double keyframe_rad_gap_ = deg2rad(10.0);
  int keyframe_min_points_ = 100;
  double translation_accumulated_ = 0.0;
  double rotation_accumulated_ = 0.0;
  bool have_last_odom_pose_ = false;
  Pose6D last_odom_pose_;

  double scan_context_leaf_size_ = 0.4;
  double map_visualization_leaf_size_ = 0.4;
  int map_keyframe_skip_ = 2;

  double sc_dist_thres_ = 0.3;
  double sc_max_radius_ = 80.0;
  double sc_lidar_height_ = 2.0;
  int sc_num_exclude_recent_ = 30;
  int sc_num_candidates_ = 3;
  int sc_tree_making_period_ = 30;
  bool sc_debug_timing_ = false;
  bool enable_loop_closure_ = true;

  double loop_detection_period_s_ = 1.0;
  double loop_icp_period_s_ = 0.05;
  int loop_icp_history_search_num_ = 25;
  double loop_icp_max_correspondence_distance_ = 150.0;
  int loop_icp_max_iterations_ = 100;
  double loop_icp_transformation_epsilon_ = 1.0e-6;
  double loop_icp_euclidean_fitness_epsilon_ = 1.0e-6;
  double loop_icp_fitness_score_threshold_ = 0.3;
  int loop_queue_warn_threshold_ = 30;

  double process_period_s_ = 0.005;
  double isam_update_period_s_ = 1.0;
  double path_publish_period_s_ = 0.1;
  double map_publish_period_s_ = 10.0;
  double isam_relinearize_threshold_ = 0.01;
  int isam_relinearize_skip_ = 1;

  std::string save_directory_;
  std::string optimized_kitti_file_;
  std::string odom_kitti_file_;
  std::string scans_directory_;
  bool save_scans_ = true;
  bool reset_scan_directory_on_start_ = true;
  std::fstream time_save_stream_;

  std::vector<double> prior_variances_;
  std::vector<double> odom_variances_;
  std::vector<double> loop_variances_;
  std::vector<double> gps_variances_;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
  gtsam::noiseModel::Base::shared_ptr robust_loop_noise_;
  gtsam::noiseModel::Base::shared_ptr robust_gps_noise_;

  std::unique_ptr<gtsam::ISAM2> isam_;
  gtsam::NonlinearFactorGraph graph_;
  gtsam::Values initial_estimate_;
  gtsam::Values isam_current_estimate_;
  bool posegraph_made_ = false;

  ScanContextManager sc_manager_;

  std::deque<nav_msgs::msg::Odometry::SharedPtr> odom_buffer_;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> cloud_buffer_;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> scan_context_cloud_buffer_;
  std::deque<sensor_msgs::msg::NavSatFix::SharedPtr> gps_buffer_;
  std::deque<std::pair<int, int>> sc_loop_icp_buffer_;

  std::vector<pcl::PointCloud<PointType>::Ptr> keyframe_laser_clouds_;
  std::vector<Pose6D> keyframe_poses_;
  std::vector<Pose6D> keyframe_poses_updated_;
  std::vector<double> keyframe_times_;
  int recent_idx_updated_ = -1;
  bool map_redraw_needed_ = false;

  bool gps_offset_initialized_ = false;
  double gps_altitude_init_offset_ = 0.0;
  double recent_optimized_x_ = 0.0;
  double recent_optimized_y_ = 0.0;

  std::mutex buffer_mutex_;
  std::mutex keyframe_mutex_;
  std::mutex posegraph_mutex_;
  std::mutex loop_queue_mutex_;
  std::mutex recent_pose_mutex_;

  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::CallbackGroup::SharedPtr process_group_;
  rclcpp::CallbackGroup::SharedPtr loop_group_;
  rclcpp::CallbackGroup::SharedPtr icp_group_;
  rclcpp::CallbackGroup::SharedPtr isam_group_;
  rclcpp::CallbackGroup::SharedPtr viz_group_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_context_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pgo_odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pgo_path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pgo_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr loop_submap_pub_;

  rclcpp::TimerBase::SharedPtr process_timer_;
  rclcpp::TimerBase::SharedPtr loop_detection_timer_;
  rclcpp::TimerBase::SharedPtr icp_timer_;
  rclcpp::TimerBase::SharedPtr isam_timer_;
  rclcpp::TimerBase::SharedPtr path_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace sc_pgo

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sc_pgo::ScPgoNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
    rclcpp::ExecutorOptions(),
    4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
