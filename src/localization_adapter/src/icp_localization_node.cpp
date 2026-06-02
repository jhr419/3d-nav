#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace
{

using PointType = pcl::PointXYZI;

Eigen::Affine3d poseToAffine(const geometry_msgs::msg::Pose & pose)
{
  Eigen::Translation3d translation(
    pose.position.x,
    pose.position.y,
    pose.position.z);
  Eigen::Quaterniond rotation(
    pose.orientation.w,
    pose.orientation.x,
    pose.orientation.y,
    pose.orientation.z);
  return translation * rotation.normalized();
}

geometry_msgs::msg::Pose affineToPose(const Eigen::Affine3d & affine)
{
  geometry_msgs::msg::Pose pose;
  const Eigen::Vector3d t = affine.translation();
  const Eigen::Quaterniond q(affine.rotation());
  pose.position.x = t.x();
  pose.position.y = t.y();
  pose.position.z = t.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

Eigen::Affine3d makeTransform(
  const std::vector<double> & xyz,
  const std::vector<double> & rpy)
{
  const double x = xyz.size() > 0 ? xyz[0] : 0.0;
  const double y = xyz.size() > 1 ? xyz[1] : 0.0;
  const double z = xyz.size() > 2 ? xyz[2] : 0.0;
  const double roll = rpy.size() > 0 ? rpy[0] : 0.0;
  const double pitch = rpy.size() > 1 ? rpy[1] : 0.0;
  const double yaw = rpy.size() > 2 ? rpy[2] : 0.0;
  return Eigen::Translation3d(x, y, z) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
}

void downsample(
  pcl::PointCloud<PointType>::Ptr cloud,
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

}  // namespace

class IcpLocalizationNode : public rclcpp::Node
{
public:
  IcpLocalizationNode()
  : Node("icp_localization_node")
  {
    map_pcd_path_ = declare_parameter<std::string>("map_pcd_path", "");
    icp_map_pcd_path_ = declare_parameter<std::string>("icp_map_pcd_path", map_pcd_path_);
    visualization_map_pcd_path_ =
      declare_parameter<std::string>("visualization_map_pcd_path", map_pcd_path_);
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/Odometry");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/cloud_registered_body");
    initialpose_topic_ = declare_parameter<std::string>("initialpose_topic", "/initialpose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "camera_init");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    publish_base_tf_ = declare_parameter<bool>("publish_base_tf", true);
    publish_map_cloud_ = declare_parameter<bool>("publish_map_cloud", true);

    min_range_ = declare_parameter<double>("min_range", 0.5);
    max_range_ = declare_parameter<double>("max_range", 60.0);
    scan_leaf_size_ = declare_parameter<double>("scan_leaf_size", 0.25);
    map_leaf_size_ = declare_parameter<double>("map_leaf_size", 0.25);
    min_scan_points_ = declare_parameter<int>("min_scan_points", 120);
    source_non_ground_min_z_ = declare_parameter<double>("source_non_ground_min_z", -0.2);
    source_non_ground_filter_en_ = declare_parameter<bool>("source_non_ground_filter_en", true);
    max_correspondence_distance_ = declare_parameter<double>("max_correspondence_distance", 1.5);
    transformation_epsilon_ = declare_parameter<double>("transformation_epsilon", 0.01);
    euclidean_fitness_epsilon_ = declare_parameter<double>("euclidean_fitness_epsilon", 0.01);
    max_iterations_ = declare_parameter<int>("max_iterations", 40);
    fitness_score_threshold_ = declare_parameter<double>("fitness_score_threshold", 1.0);
    relocalization_interval_s_ = declare_parameter<double>("relocalization_interval_s", 0.2);
    use_initial_pose_param_ = declare_parameter<bool>("use_initial_pose_param", false);

    const auto base_to_body_xyz =
      declare_parameter<std::vector<double>>("base_to_body_xyz", {0.0, 0.0, 0.0});
    const auto base_to_body_rpy =
      declare_parameter<std::vector<double>>("base_to_body_rpy", {0.0, 0.0, 0.0});
    const auto initial_pose_xyz =
      declare_parameter<std::vector<double>>("initial_pose_xyz", {0.0, 0.0, 0.0});
    const auto initial_pose_rpy =
      declare_parameter<std::vector<double>>("initial_pose_rpy", {0.0, 0.0, 0.0});

    base_to_body_ = makeTransform(base_to_body_xyz, base_to_body_rpy);
    body_to_base_ = base_to_body_.inverse();
    initial_map_to_base_ = makeTransform(initial_pose_xyz, initial_pose_rpy);

    // 如果icp_map_pcd_path_和visualization_map_pcd_path_没有单独设置，就使用map_pcd_path_。这样用户只需要提供一个地图文件即可满足ICP和可视化的需求。
    if (icp_map_pcd_path_.empty()) {
      icp_map_pcd_path_ = map_pcd_path_;
    }
    if (visualization_map_pcd_path_.empty()) {
      visualization_map_pcd_path_ = map_pcd_path_;
    }

    icp_map_cloud_ = loadMapCloud(icp_map_pcd_path_);
    if (!icp_map_cloud_ || icp_map_cloud_->empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to load ICP target map PCD: '%s'. Set icp_map_pcd_path to the non-ground map.",
        icp_map_pcd_path_.c_str());
    } else {
      downsample(icp_map_cloud_, map_leaf_size_);
      icp_.setInputTarget(icp_map_cloud_);
      RCLCPP_INFO(
        get_logger(),
        "Loaded ICP target map: %s points=%zu leaf=%.3f",
        icp_map_pcd_path_.c_str(), icp_map_cloud_->size(), map_leaf_size_);
    }

    visualization_map_cloud_ = loadMapCloud(visualization_map_pcd_path_);
    if (!visualization_map_cloud_ || visualization_map_cloud_->empty()) {
      RCLCPP_WARN(
        get_logger(),
        "Visualization map PCD is empty: '%s'. Falling back to ICP target map for display.",
        visualization_map_pcd_path_.c_str());
      visualization_map_cloud_ = icp_map_cloud_;
    } else {
      downsample(visualization_map_cloud_, map_leaf_size_);
      RCLCPP_INFO(
        get_logger(),
        "Loaded visualization map: %s points=%zu leaf=%.3f",
        visualization_map_pcd_path_.c_str(), visualization_map_cloud_->size(), map_leaf_size_);
    }

    icp_.setMaximumIterations(max_iterations_);
    icp_.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp_.setTransformationEpsilon(transformation_epsilon_);
    icp_.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 50,
      std::bind(&IcpLocalizationNode::odomCallback, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&IcpLocalizationNode::scanCallback, this, std::placeholders::_1));
    initialpose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      initialpose_topic_, 5,
      std::bind(&IcpLocalizationNode::initialPoseCallback, this, std::placeholders::_1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_pose", 10);
    aligned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("icp_aligned_cloud", 2);
    icp_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "icp_map", rclcpp::QoS(1).transient_local().reliable());
    visualization_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "icp_visualization_map", rclcpp::QoS(1).transient_local().reliable());
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    if (publish_map_cloud_) {
      map_publish_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&IcpLocalizationNode::publishMapCloud, this));
    }

    RCLCPP_INFO(
      get_logger(),
      "ICP localization waiting for odom=%s scan=%s initialpose=%s",
      odom_topic_.c_str(), scan_topic_.c_str(), initialpose_topic_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_odom_stamp_ = msg->header.stamp;
    odom_frame_ = msg->header.frame_id.empty() ? odom_frame_ : msg->header.frame_id;
    latest_odom_to_body_ = poseToAffine(msg->pose.pose);
    latest_odom_to_base_ = latest_odom_to_body_ * body_to_base_;
    if (!have_odom_) {
      RCLCPP_INFO(get_logger(), "Received first odometry from %s.", odom_topic_.c_str());
    }
    have_odom_ = true;

    if (!have_initial_pose_ && use_initial_pose_param_) {
      map_to_odom_ = initial_map_to_base_ * latest_odom_to_base_.inverse();
      have_initial_pose_ = true;
      RCLCPP_INFO(get_logger(), "Initialized map->odom from initial_pose_* parameters.");
    } else if (!have_initial_pose_ && have_pending_initial_pose_) {
      applyInitialPoseLocked(pending_initial_map_to_base_);
      RCLCPP_INFO(get_logger(), "Applied cached initial pose after odometry became available.");
    }
  }

  pcl::PointCloud<PointType>::Ptr loadMapCloud(const std::string & path) const
  {
    if (path.empty()) {
      return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }

    // Existing maps may come from FAST-LIO, GLIM, or CloudCompare and often
    // do not expose a standard "intensity" field. ICP only needs xyz.
    pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
    if (pcl::io::loadPCDFile(path, xyz_cloud) < 0) {
      return pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    }

    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    cloud->reserve(xyz_cloud.size());
    for (const auto & p_xyz : xyz_cloud.points) {
      if (!std::isfinite(p_xyz.x) || !std::isfinite(p_xyz.y) || !std::isfinite(p_xyz.z)) {
        continue;
      }
      PointType point;
      point.x = p_xyz.x;
      point.y = p_xyz.y;
      point.z = p_xyz.z;
      point.intensity = 0.0F;
      cloud->push_back(point);
    }
    return cloud;
  }

  void initialPoseCallback(
    const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const Eigen::Affine3d map_to_base = poseToAffine(msg->pose.pose);
    if (!have_odom_) {
      pending_initial_map_to_base_ = map_to_base;
      have_pending_initial_pose_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Cached initial pose in frame '%s'; waiting for odometry on %s before starting ICP.",
        msg->header.frame_id.c_str(), odom_topic_.c_str());
      return;
    }

    applyInitialPoseLocked(map_to_base);
    RCLCPP_INFO(
      get_logger(),
      "Accepted initial pose in frame '%s'; ICP relocalization is enabled.",
      msg->header.frame_id.c_str());
  }

  void applyInitialPoseLocked(const Eigen::Affine3d & map_to_base)
  {
    map_to_odom_ = map_to_base * latest_odom_to_base_.inverse();
    have_initial_pose_ = true;
    have_pending_initial_pose_ = false;
  }

  void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!have_scan_) {
      RCLCPP_INFO(get_logger(), "Received first scan from %s.", scan_topic_.c_str());
      have_scan_ = true;
    }
    if (!icp_map_cloud_ || icp_map_cloud_->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "ICP map is empty; cannot localize.");
      return;
    }
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for odometry on %s.", odom_topic_.c_str());
      return;
    }
    if (!have_initial_pose_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Waiting for initial pose on %s.", initialpose_topic_.c_str());
      return;
    }

    const rclcpp::Time stamp(msg->header.stamp);
    if ((stamp - last_icp_stamp_).seconds() < relocalization_interval_s_) {
      publishTfAndPose(msg->header.stamp, map_to_odom_ * latest_odom_to_base_, -1.0);
      return;
    }
    last_icp_stamp_ = stamp;

    auto source_base = makeSourceCloud(*msg);
    if (static_cast<int>(source_base->size()) < min_scan_points_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 3000,
        "Filtered scan has too few points for ICP: %zu.", source_base->size());
      return;
    }

    const Eigen::Affine3d guess_map_to_base = map_to_odom_ * latest_odom_to_base_;
    pcl::PointCloud<PointType> aligned;
    icp_.setInputSource(source_base);
    icp_.align(aligned, guess_map_to_base.matrix().cast<float>());

    const double fitness = icp_.getFitnessScore();
    if (!icp_.hasConverged() || fitness > fitness_score_threshold_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ICP rejected: converged=%s fitness=%.4f threshold=%.4f",
        icp_.hasConverged() ? "true" : "false",
        fitness, fitness_score_threshold_);
      publishTfAndPose(msg->header.stamp, guess_map_to_base, fitness);
      publishAlignedCloud(aligned, msg->header.stamp);
      return;
    }

    const Eigen::Affine3f refined_f(icp_.getFinalTransformation());
    const Eigen::Affine3d refined_map_to_base(refined_f.matrix().cast<double>());
    map_to_odom_ = refined_map_to_base * latest_odom_to_base_.inverse();
    publishTfAndPose(msg->header.stamp, refined_map_to_base, fitness);
    publishAlignedCloud(aligned, msg->header.stamp);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "ICP accepted: fitness=%.4f source_points=%zu", fitness, source_base->size());
  }

  pcl::PointCloud<PointType>::Ptr makeSourceCloud(const sensor_msgs::msg::PointCloud2 & msg) const
  {
    pcl::PointCloud<PointType>::Ptr cloud_body(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(msg, *cloud_body);

    pcl::PointCloud<PointType>::Ptr cloud_base(new pcl::PointCloud<PointType>());
    cloud_base->reserve(cloud_body->size());
    for (const auto & p_body : cloud_body->points) {
      if (!std::isfinite(p_body.x) || !std::isfinite(p_body.y) || !std::isfinite(p_body.z)) {
        continue;
      }
      const double range = std::sqrt(
        p_body.x * p_body.x + p_body.y * p_body.y + p_body.z * p_body.z);
      if (range < min_range_ || range > max_range_) {
        continue;
      }

      const Eigen::Vector3d base_xyz =
        body_to_base_ * Eigen::Vector3d(p_body.x, p_body.y, p_body.z);
      if (source_non_ground_filter_en_ && base_xyz.z() < source_non_ground_min_z_) {
        continue;
      }
      PointType p_base;
      p_base.x = static_cast<float>(base_xyz.x());
      p_base.y = static_cast<float>(base_xyz.y());
      p_base.z = static_cast<float>(base_xyz.z());
      p_base.intensity = p_body.intensity;
      cloud_base->push_back(p_base);
    }

    downsample(cloud_base, scan_leaf_size_);
    return cloud_base;
  }

  void publishTfAndPose(
    const builtin_interfaces::msg::Time & stamp,
    const Eigen::Affine3d & map_to_base,
    const double fitness)
  {
    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = odom_frame_;
      const auto pose = affineToPose(map_to_odom_);
      transform.transform.translation.x = pose.position.x;
      transform.transform.translation.y = pose.position.y;
      transform.transform.translation.z = pose.position.z;
      transform.transform.rotation = pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    if (publish_base_tf_) {
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = stamp;
      transform.header.frame_id = map_frame_;
      transform.child_frame_id = base_frame_;
      const auto pose = affineToPose(map_to_base);
      transform.transform.translation.x = pose.position.x;
      transform.transform.translation.y = pose.position.y;
      transform.transform.translation.z = pose.position.z;
      transform.transform.rotation = pose.orientation;
      tf_broadcaster_->sendTransform(transform);
    }

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.pose = affineToPose(map_to_base);
    const double covariance = fitness >= 0.0 ? std::max(0.001, fitness) : 0.25;
    pose_msg.pose.covariance[0] = covariance;
    pose_msg.pose.covariance[7] = covariance;
    pose_msg.pose.covariance[14] = covariance;
    pose_msg.pose.covariance[35] = covariance;
    pose_pub_->publish(pose_msg);
  }

  void publishAlignedCloud(
    const pcl::PointCloud<PointType> & aligned,
    const builtin_interfaces::msg::Time & stamp) const
  {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(aligned, cloud_msg);
    cloud_msg.header.stamp = stamp;
    cloud_msg.header.frame_id = map_frame_;
    aligned_pub_->publish(cloud_msg);
  }

  void publishMapCloud() const
  {
    if (visualization_map_cloud_ && !visualization_map_cloud_->empty()) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*visualization_map_cloud_, cloud_msg);
      cloud_msg.header.stamp = now();
      cloud_msg.header.frame_id = map_frame_;
      visualization_map_pub_->publish(cloud_msg);
    }

    if (icp_map_cloud_ && !icp_map_cloud_->empty()) {
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*icp_map_cloud_, cloud_msg);
      cloud_msg.header.stamp = now();
      cloud_msg.header.frame_id = map_frame_;
      icp_map_pub_->publish(cloud_msg);
    }
  }

  std::mutex mutex_;
  std::string map_pcd_path_;
  std::string icp_map_pcd_path_;
  std::string visualization_map_pcd_path_;
  std::string odom_topic_;
  std::string scan_topic_;
  std::string initialpose_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  bool publish_tf_ = true;
  bool publish_base_tf_ = true;
  bool publish_map_cloud_ = true;
  bool use_initial_pose_param_ = false;

  double min_range_ = 0.5;
  double max_range_ = 60.0;
  double scan_leaf_size_ = 0.25;
  double map_leaf_size_ = 0.25;
  int min_scan_points_ = 120;
  double source_non_ground_min_z_ = -0.2;
  bool source_non_ground_filter_en_ = true;
  double max_correspondence_distance_ = 1.5;
  double transformation_epsilon_ = 0.01;
  double euclidean_fitness_epsilon_ = 0.01;
  int max_iterations_ = 40;
  double fitness_score_threshold_ = 1.0;
  double relocalization_interval_s_ = 0.2;

  Eigen::Affine3d base_to_body_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d body_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d initial_map_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d pending_initial_map_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d latest_odom_to_body_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d latest_odom_to_base_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d map_to_odom_ = Eigen::Affine3d::Identity();
  builtin_interfaces::msg::Time latest_odom_stamp_;
  rclcpp::Time last_icp_stamp_{0, 0, RCL_ROS_TIME};
  bool have_odom_ = false;
  bool have_scan_ = false;
  bool have_initial_pose_ = false;
  bool have_pending_initial_pose_ = false;

  pcl::PointCloud<PointType>::Ptr icp_map_cloud_;
  pcl::PointCloud<PointType>::Ptr visualization_map_cloud_;
  pcl::IterativeClosestPoint<PointType, PointType> icp_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr icp_map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr visualization_map_pub_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
