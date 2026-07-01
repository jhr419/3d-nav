#include <memory>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <string>
#include <optional>

#include "octomap/AbstractOcTree.h"
#include "octomap/OcTree.h"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/msg/octomap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

class OctomapToOccupiedMarkersNode : public rclcpp::Node
{
public:
  OctomapToOccupiedMarkersNode()
  : Node("octomap_to_occupied_markers")
  {
    declare_parameter<std::string>("octomap_topic", "/octomap");
    declare_parameter<std::string>("marker_topic", "/octomap_occupied_markers");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<int>("marker_stride", 1);
    declare_parameter<int>("max_marker_points", 0);
    declare_parameter<double>("republish_period_sec", 1.0);

    const auto octomap_topic = get_parameter("octomap_topic").as_string();
    const auto marker_topic = get_parameter("marker_topic").as_string();

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      marker_topic, rclcpp::QoS(1).transient_local().reliable());

    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic, rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&OctomapToOccupiedMarkersNode::onOctomap, this, std::placeholders::_1));

    const double republish_period =
      std::max(0.0, static_cast<double>(get_parameter("republish_period_sec").as_double()));
    if (republish_period > 0.0) {
      republish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(republish_period)),
        std::bind(&OctomapToOccupiedMarkersNode::republishMarker, this));
    }

    RCLCPP_INFO(
      get_logger(), "octomap_to_occupied_markers started. octomap_topic=%s marker_topic=%s",
      octomap_topic.c_str(), marker_topic.c_str());
  }

private:
  void onOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::unique_ptr<octomap::AbstractOcTree> tree_ptr(octomap_msgs::msgToMap(*msg));
    if (!tree_ptr) {
      RCLCPP_ERROR(get_logger(), "Failed to decode octomap message.");
      return;
    }

    auto * oc_tree = dynamic_cast<octomap::OcTree *>(tree_ptr.get());
    if (!oc_tree) {
      RCLCPP_ERROR(get_logger(), "Decoded map is not octomap::OcTree.");
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.stamp = msg->header.stamp;
    marker.header.frame_id = msg->header.frame_id.empty() ?
      get_parameter("frame_id").as_string() : msg->header.frame_id;
    marker.ns = "occupied_voxels";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = oc_tree->getResolution();
    marker.scale.y = oc_tree->getResolution();
    marker.scale.z = oc_tree->getResolution();
    marker.color.r = 0.95F;
    marker.color.g = 0.45F;
    marker.color.b = 0.15F;
    marker.color.a = 0.95F;

    std::size_t occupied_count = 0;
    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it) {
      if (oc_tree->isNodeOccupied(*it)) {
        ++occupied_count;
      }
    }

    int marker_stride = std::max(1, static_cast<int>(get_parameter("marker_stride").as_int()));
    const int max_marker_points = static_cast<int>(get_parameter("max_marker_points").as_int());
    if (max_marker_points > 0 && occupied_count > static_cast<std::size_t>(max_marker_points)) {
      marker_stride = std::max(
        marker_stride,
        static_cast<int>(std::ceil(
          static_cast<double>(occupied_count) / static_cast<double>(max_marker_points))));
    }

    marker.points.reserve(marker_stride > 1 ? occupied_count / marker_stride + 1 : occupied_count);
    std::size_t occupied_index = 0;
    for (auto it = oc_tree->begin_leafs(); it != oc_tree->end_leafs(); ++it) {
      if (!oc_tree->isNodeOccupied(*it)) {
        continue;
      }
      if ((occupied_index++ % static_cast<std::size_t>(marker_stride)) != 0U) {
        continue;
      }
      geometry_msgs::msg::Point point;
      point.x = it.getX();
      point.y = it.getY();
      point.z = it.getZ();
      marker.points.push_back(point);
    }

    latest_marker_ = marker;
    marker_pub_->publish(marker);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 3000,
      "Published occupied marker from OctoMap: %zu/%zu voxels stride=%d",
      marker.points.size(), occupied_count, marker_stride);
  }

  void republishMarker()
  {
    if (!latest_marker_) {
      return;
    }
    auto marker = *latest_marker_;
    marker.header.stamp = now();
    marker_pub_->publish(marker);
  }

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;
  std::optional<visualization_msgs::msg::Marker> latest_marker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OctomapToOccupiedMarkersNode>());
  rclcpp::shutdown();
  return 0;
}
