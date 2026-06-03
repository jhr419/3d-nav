#include <memory>
#include <fstream>
#include <string>
#include <cstring>
#include <cstdint>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap/AbstractOcTree.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include "global_planner/global_planner_core.hpp"

namespace global_planner
{

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planner_node")
  {
    frame_id_ = declare_parameter<std::string>("frame_id", "map");
    debug_mode_ = declare_parameter<bool>("debug_mode", true);
    replan_on_pose_update_ = declare_parameter<bool>("replan_on_pose_update", false);
    map_source_ = declare_parameter<std::string>("map_source", "file");
    map_file_path_ = declare_parameter<std::string>("map_file_path", "");
    pcd_octomap_resolution_ = declare_parameter<double>("pcd_octomap_resolution", 0.20);
    pcd_voxel_leaf_size_ = declare_parameter<double>("pcd_voxel_leaf_size", 0.0);
    pcd_min_z_ = declare_parameter<double>("pcd_min_z", -1000.0);
    pcd_max_z_ = declare_parameter<double>("pcd_max_z", 1000.0);
    const std::string octomap_topic = declare_parameter<std::string>("octomap_topic", "/octomap_full");
    const std::string start_topic = declare_parameter<std::string>("start_topic", "/global_planner/start");
    const std::string localization_pose_topic =
      declare_parameter<std::string>("localization_pose_topic", "/icp_pose");
    const std::string goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
    const std::string path_topic = declare_parameter<std::string>("path_topic", "/global_path");
    const std::string occupied_map_cloud_topic =
      declare_parameter<std::string>("occupied_map_cloud_topic", "/global_planner/occupied_map");
    publish_occupied_map_cloud_ =
      declare_parameter<bool>("publish_occupied_map_cloud", true);

    PlannerOptions options;
    options.robot_radius = declare_parameter<double>("robot_radius", options.robot_radius);
    options.max_iterations = declare_parameter<int>("max_iterations", options.max_iterations);
    options.snap_search_radius_cells =
      declare_parameter<int>("snap_search_radius_cells", options.snap_search_radius_cells);
    options.require_ground_support =
      declare_parameter<bool>("require_ground_support", options.require_ground_support);
    options.strict_direct_ground_support =
      declare_parameter<bool>("strict_direct_ground_support", options.strict_direct_ground_support);
    options.ground_support_xy_radius_cells =
      declare_parameter<int>("ground_support_xy_radius_cells", options.ground_support_xy_radius_cells);
    options.ground_support_depth_cells =
      declare_parameter<int>("ground_support_depth_cells", options.ground_support_depth_cells);
    options.enable_preblocked_costmap =
      declare_parameter<bool>("enable_preblocked_costmap", options.enable_preblocked_costmap);
    options.preblocked_costmap_radius_cells =
      declare_parameter<int>("preblocked_costmap_radius_cells", options.preblocked_costmap_radius_cells);
    options.preblocked_costmap_weight =
      declare_parameter<double>("preblocked_costmap_weight", options.preblocked_costmap_weight);
    options.lowest_traversable_only =
      declare_parameter<bool>("lowest_traversable_only", options.lowest_traversable_only);
    planner_.setOptions(options);

    path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic, rclcpp::QoS(1).transient_local());
    occupied_map_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      occupied_map_cloud_topic, rclcpp::QoS(1).transient_local().reliable());
    if (map_source_ == "file") {
      loadMapFromFile();
    } else if (map_source_ == "topic") {
      octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        octomap_topic,
        rclcpp::QoS(1).transient_local(),
        std::bind(&GlobalPlannerNode::onOctomap, this, std::placeholders::_1));
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "Invalid map_source='%s'. Use 'file' or 'topic'.",
        map_source_.c_str());
    }

    if (debug_mode_) {
      manual_start_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        start_topic,
        rclcpp::QoS(10),
        std::bind(&GlobalPlannerNode::onManualStart, this, std::placeholders::_1));
    } else {
      localization_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        localization_pose_topic,
        rclcpp::QoS(10),
        std::bind(&GlobalPlannerNode::onLocalizationPose, this, std::placeholders::_1));
    }

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic,
      rclcpp::QoS(10),
      std::bind(&GlobalPlannerNode::onGoal, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Global planner ready: mode=%s map_source=%s map=%s start=%s goal=%s path=%s occupied_map=%s",
      debug_mode_ ? "debug" : "localization",
      map_source_.c_str(),
      map_source_ == "file" ? map_file_path_.c_str() : octomap_topic.c_str(),
      debug_mode_ ? start_topic.c_str() : localization_pose_topic.c_str(),
      goal_topic.c_str(),
      path_topic.c_str(),
      occupied_map_cloud_topic.c_str());
  }

private:
  void loadMapFromFile()
  {
    if (map_file_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "map_source is 'file' but map_file_path is empty");
      return;
    }

    std::ifstream file_check(map_file_path_, std::ios::binary);
    if (!file_check.good()) {
      RCLCPP_ERROR(get_logger(), "Cannot open map file: %s", map_file_path_.c_str());
      return;
    }
    file_check.close();

    std::shared_ptr<octomap::OcTree> octree;
    if (hasSuffix(map_file_path_, ".pcd")) {
      octree = loadPcdAsOctomap();
      if (!octree) {
        return;
      }
    } else if (hasSuffix(map_file_path_, ".bt")) {
      auto binary_tree = std::make_shared<octomap::OcTree>(0.1);
      if (!binary_tree->readBinary(map_file_path_)) {
        RCLCPP_ERROR(get_logger(), "Failed to read binary OctoMap file: %s", map_file_path_.c_str());
        return;
      }
      octree = binary_tree;
    } else {
      std::unique_ptr<octomap::AbstractOcTree> abstract_tree(
        octomap::AbstractOcTree::read(map_file_path_));
      if (!abstract_tree) {
        RCLCPP_ERROR(get_logger(), "Failed to read OctoMap file: %s", map_file_path_.c_str());
        return;
      }

      auto * raw_octree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
      if (raw_octree == nullptr) {
        RCLCPP_ERROR(
          get_logger(),
          "OctoMap file is not an octomap::OcTree: %s",
          map_file_path_.c_str());
        return;
      }
      abstract_tree.release();
      octree = std::shared_ptr<octomap::OcTree>(raw_octree);
    }

    planner_.setOctomap(octree);
    publishOccupiedMapCloud(octree);
    RCLCPP_INFO(
      get_logger(),
      "Map loaded from file: %s resolution=%.3f traversable=%zu preblocked=%zu",
      map_file_path_.c_str(),
      octree->getResolution(),
      planner_.traversableCells().size(),
      planner_.preblockedCells().size());
  }

  std::shared_ptr<octomap::OcTree> loadPcdAsOctomap()
  {
    if (pcd_octomap_resolution_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "pcd_octomap_resolution must be positive");
      return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile(map_file_path_, *cloud) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to read PCD file: %s", map_file_path_.c_str());
      return nullptr;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
    filtered->reserve(cloud->size());
    for (const auto & point : cloud->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
        continue;
      }
      if (point.z < pcd_min_z_ || point.z > pcd_max_z_) {
        continue;
      }
      filtered->push_back(point);
    }

    if (pcd_voxel_leaf_size_ > 0.0 && !filtered->empty()) {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setLeafSize(
        static_cast<float>(pcd_voxel_leaf_size_),
        static_cast<float>(pcd_voxel_leaf_size_),
        static_cast<float>(pcd_voxel_leaf_size_));
      voxel_grid.setInputCloud(filtered);
      pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
      voxel_grid.filter(*downsampled);
      filtered = downsampled;
    }

    if (filtered->empty()) {
      RCLCPP_ERROR(get_logger(), "PCD map has no valid points after filtering: %s", map_file_path_.c_str());
      return nullptr;
    }

    auto octree = std::make_shared<octomap::OcTree>(pcd_octomap_resolution_);
    for (const auto & point : filtered->points) {
      octree->updateNode(octomap::point3d(point.x, point.y, point.z), true);
    }
    octree->updateInnerOccupancy();

    RCLCPP_INFO(
      get_logger(),
      "Converted PCD to OctoMap: raw_points=%zu used_points=%zu resolution=%.3f leaf=%.3f z=[%.3f, %.3f]",
      cloud->size(),
      filtered->size(),
      pcd_octomap_resolution_,
      pcd_voxel_leaf_size_,
      pcd_min_z_,
      pcd_max_z_);

    return octree;
  }

  bool hasSuffix(const std::string & text, const std::string & suffix) const
  {
    return text.size() >= suffix.size() &&
           text.compare(text.size() - suffix.size(), suffix.size(), suffix) == 0;
  }

  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(octomap_msgs::fullMsgToMap(*msg));
    if (!abstract_tree) {
      RCLCPP_WARN(get_logger(), "Received OctoMap message could not be converted");
      return;
    }

    auto * octree = dynamic_cast<octomap::OcTree *>(abstract_tree.get());
    if (octree == nullptr) {
      RCLCPP_WARN(get_logger(), "Received OctoMap is not an octomap::OcTree");
      return;
    }

    abstract_tree.release();
    auto octree_ptr = std::shared_ptr<octomap::OcTree>(octree);
    planner_.setOctomap(octree_ptr);
    publishOccupiedMapCloud(octree_ptr);
    if (!msg->header.frame_id.empty()) {
      frame_id_ = msg->header.frame_id;
    }

    RCLCPP_INFO(
      get_logger(),
      "OctoMap updated: resolution=%.3f traversable=%zu preblocked=%zu",
      octree->getResolution(),
      planner_.traversableCells().size(),
      planner_.preblockedCells().size());

    tryPlan();
  }

  void onManualStart(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (!debug_mode_) {
      return;
    }

    start_ = Point3D{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};
    has_start_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Manual start updated: [%.3f, %.3f, %.3f]",
      start_.x,
      start_.y,
      start_.z);
    tryPlan();
  }

  void publishOccupiedMapCloud(const std::shared_ptr<octomap::OcTree> & octree)
  {
    if (!publish_occupied_map_cloud_ || !octree) {
      return;
    }

    sensor_msgs::msg::PointCloud2 cloud_msg;
    cloud_msg.header.stamp = now();
    cloud_msg.header.frame_id = frame_id_;
    cloud_msg.height = 1;
    cloud_msg.is_bigendian = false;
    cloud_msg.is_dense = true;
    cloud_msg.point_step = 3 * sizeof(float);

    cloud_msg.fields.resize(3);
    const std::vector<std::string> names{"x", "y", "z"};
    for (std::size_t i = 0; i < names.size(); ++i) {
      cloud_msg.fields[i].name = names[i];
      cloud_msg.fields[i].offset = static_cast<std::uint32_t>(i * sizeof(float));
      cloud_msg.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
      cloud_msg.fields[i].count = 1;
    }

    std::vector<float> points;
    for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
      if (!octree->isNodeOccupied(*it)) {
        continue;
      }
      points.push_back(static_cast<float>(it.getX()));
      points.push_back(static_cast<float>(it.getY()));
      points.push_back(static_cast<float>(it.getZ()));
    }

    cloud_msg.width = static_cast<std::uint32_t>(points.size() / 3);
    cloud_msg.row_step = cloud_msg.width * cloud_msg.point_step;
    cloud_msg.data.resize(points.size() * sizeof(float));
    if (!points.empty()) {
      std::memcpy(cloud_msg.data.data(), points.data(), cloud_msg.data.size());
    }
    occupied_map_cloud_pub_->publish(cloud_msg);
  }

  void onLocalizationPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    if (debug_mode_) {
      return;
    }

    const bool first_pose = !has_start_;
    start_ = Point3D{
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z};
    has_start_ = true;

    if (first_pose) {
      RCLCPP_INFO(
        get_logger(),
        "Received first localization pose: [%.3f, %.3f, %.3f]",
        start_.x,
        start_.y,
        start_.z);
    }

    if (first_pose || replan_on_pose_update_) {
      tryPlan();
    }
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_ = Point3D{
      msg->pose.position.x,
      msg->pose.position.y,
      msg->pose.position.z};
    has_goal_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Goal updated: [%.3f, %.3f, %.3f]",
      goal_.x,
      goal_.y,
      goal_.z);
    tryPlan();
  }

  void tryPlan()
  {
    if (!planner_.hasMap() || !has_start_ || !has_goal_) {
      return;
    }

    const PlanResult result = planner_.makePlan(start_, goal_);
    if (!result.success) {
      RCLCPP_WARN(get_logger(), "Global planning failed: %s", result.message.c_str());
      return;
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = now();
    path_msg.header.frame_id = frame_id_;
    path_msg.poses.reserve(result.path.size());
    for (const auto & point : result.path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position.x = point.x;
      pose.pose.position.y = point.y;
      pose.pose.position.z = point.z;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
    }

    path_pub_->publish(path_msg);
    RCLCPP_INFO(
      get_logger(),
      "Global path published: waypoints=%zu iterations=%d snapped_start=%s snapped_goal=%s",
      result.path.size(),
      result.iterations,
      result.snapped_start ? "true" : "false",
      result.snapped_goal ? "true" : "false");
  }

  GlobalPlannerCore planner_;
  std::string frame_id_;
  std::string map_source_;
  std::string map_file_path_;
  double pcd_octomap_resolution_{0.20};
  double pcd_voxel_leaf_size_{0.0};
  double pcd_min_z_{-1000.0};
  double pcd_max_z_{1000.0};
  bool debug_mode_{true};
  bool replan_on_pose_update_{false};
  bool publish_occupied_map_cloud_{true};
  bool has_start_{false};
  bool has_goal_{false};
  Point3D start_;
  Point3D goal_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occupied_map_cloud_pub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr manual_start_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    localization_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
};

}  // namespace global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planner::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
