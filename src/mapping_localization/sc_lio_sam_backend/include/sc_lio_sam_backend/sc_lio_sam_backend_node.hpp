#pragma once

#include "sc_lio_sam_backend/keyframe_manager.hpp"
#include "sc_lio_sam_backend/pose_graph_optimizer.hpp"
#include "sc_lio_sam_backend/scan_context_manager.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/registration/icp.h>

#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace sc_lio_sam_backend
{

class ScLioSamBackendNode : public rclcpp::Node
{
public:
  explicit ScLioSamBackendNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ScLioSamBackendNode() override;

private:
  void declareAndLoadParameters();
  void createRosInterfaces();

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void frontendMapCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void frontendPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
  void loopTimerCallback();
  void mapTimerCallback();
  void saveMapCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void addKeyframe(
    const builtin_interfaces::msg::Time & stamp,
    double stamp_sec,
    const Eigen::Isometry3d & raw_pose,
    const PointCloudPtr & local_cloud,
    const ScanContextDescriptor & descriptor);

  bool optimizeGraphAndUpdateKeyframes();
  void publishOptimizedPathLocked();
  void publishRawPathLocked();
  void publishOptimizedOdometryLocked();
  void publishLoopMarkersLocked();
  void publishTfLocked();

  PointCloudPtr pointCloud2ToPclLocal(
    const sensor_msgs::msg::PointCloud2 & msg,
    const nav_msgs::msg::Odometry & odom_msg,
    const Eigen::Isometry3d & raw_pose) const;
  bool cloudIsInOdomFrame(
    const std::string & cloud_frame,
    const nav_msgs::msg::Odometry & odom_msg) const;
  PointCloudPtr downsampleCloud(const PointCloudConstPtr & cloud, double leaf_size) const;
  PointCloudPtr transformCloud(const PointCloudConstPtr & cloud, const Eigen::Isometry3d & pose) const;
  PointCloudPtr buildKeyframeMap(
    const std::vector<Keyframe> & keyframes,
    int center_id,
    int search_num) const;
  PointCloudPtr buildGlobalMap(const std::vector<Keyframe> & keyframes) const;
  bool saveOptimizedMap(const std::vector<Keyframe> & keyframes, std::string * message) const;

  static Eigen::Isometry3d poseMsgToEigen(const geometry_msgs::msg::Pose & pose_msg);
  static geometry_msgs::msg::Pose eigenToPoseMsg(const Eigen::Isometry3d & pose);
  static geometry_msgs::msg::TransformStamped eigenToTransformMsg(
    const Eigen::Isometry3d & pose,
    const builtin_interfaces::msg::Time & stamp,
    const std::string & parent,
    const std::string & child);
  static std::string normalizedFrame(std::string frame);

  std::string odom_topic_;
  std::string cloud_topic_;
  std::string frontend_map_topic_;
  std::string frontend_path_topic_;
  std::string imu_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string lidar_frame_;
  std::string cloud_frame_mode_;

  bool use_imu_ = false;
  bool use_scan_context_ = true;
  bool enable_loop_closure_ = true;
  bool enable_pose_graph_optimization_ = true;
  bool publish_tf_ = true;
  bool publish_map_to_odom_ = true;
  bool passthrough_frontend_map_ = true;
  bool passthrough_frontend_path_when_loop_disabled_ = true;
  bool save_pcd_ = false;
  std::string save_directory_;

  double keyframe_distance_threshold_ = 1.0;
  double keyframe_angle_threshold_deg_ = 10.0;
  double loop_search_radius_ = 15.0;
  double loop_search_time_diff_threshold_ = 30.0;
  double loop_fitness_score_threshold_ = 0.3;
  int loop_keyframe_search_num_ = 5;
  double keyframe_voxel_leaf_size_ = 0.4;
  double icp_voxel_leaf_size_ = 0.5;
  double map_voxel_leaf_size_ = 0.3;
  double loop_detection_period_sec_ = 1.0;
  double map_publish_period_sec_ = 3.0;
  double max_odom_cloud_time_diff_ = 0.2;
  double icp_max_correspondence_distance_ = 30.0;
  int icp_max_iterations_ = 80;
  int min_icp_source_points_ = 100;
  int min_icp_target_points_ = 300;
  double max_loop_translation_correction_ = 0.0;
  double max_loop_rotation_correction_deg_ = 0.0;

  KeyframeManager keyframe_manager_;
  ScanContextManager scan_context_manager_;
  PoseGraphOptimizer pose_graph_optimizer_;

  std::mutex data_mutex_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_frontend_map_;
  nav_msgs::msg::Path::SharedPtr latest_frontend_path_;
  mutable bool warned_unknown_cloud_frame_ = false;

  std::set<std::pair<int, int>> loop_edge_pairs_;
  std::vector<LoopEdge> loop_edges_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr frontend_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr frontend_path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr optimized_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr optimized_odom_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr loop_markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr optimized_map_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_service_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
  rclcpp::TimerBase::SharedPtr map_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace sc_lio_sam_backend
