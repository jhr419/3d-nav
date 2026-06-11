#include <memory>
#include <string>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class IcpPoseToStartPointNode : public rclcpp::Node
{
public:
  IcpPoseToStartPointNode()
  : Node("icp_pose_to_start_point")
  {
    declare_parameter<std::string>("input_pose_topic", "/icp_pose");
    declare_parameter<std::string>("output_point_topic", "/start_point");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<bool>("use_pose_stamp", true);
    declare_parameter<bool>("publish_z", true);
    declare_parameter<double>("z_offset", 0.0);

    const auto input_pose_topic = get_parameter("input_pose_topic").as_string();
    const auto output_point_topic = get_parameter("output_point_topic").as_string();

    start_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(
      output_point_topic, rclcpp::QoS(1).transient_local().reliable());
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      input_pose_topic, rclcpp::QoS(10).best_effort(),
      std::bind(&IcpPoseToStartPointNode::onPose, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "icp_pose_to_start_point started. input=%s output=%s frame_id=%s",
      input_pose_topic.c_str(), output_point_topic.c_str(),
      get_parameter("frame_id").as_string().c_str());
  }

private:
  void onPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    geometry_msgs::msg::PointStamped start;
    if (get_parameter("use_pose_stamp").as_bool()) {
      start.header.stamp = msg->header.stamp;
    } else {
      start.header.stamp = now();
    }

    std::string frame_id = get_parameter("frame_id").as_string();
    if (frame_id.empty()) {
      frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    }
    start.header.frame_id = frame_id;

    start.point.x = msg->pose.pose.position.x;
    start.point.y = msg->pose.pose.position.y;
    start.point.z = get_parameter("publish_z").as_bool() ?
      msg->pose.pose.position.z + get_parameter("z_offset").as_double() :
      get_parameter("z_offset").as_double();

    start_pub_->publish(start);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IcpPoseToStartPointNode>());
  rclcpp::shutdown();
  return 0;
}
