#include <memory>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/time.h>
#if __has_include("tf2_geometry_msgs/tf2_geometry_msgs.hpp")
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>

class EgoNavigationGoalBridgeNode : public rclcpp::Node
{
public:
  EgoNavigationGoalBridgeNode()
  : Node("ego_navigation_goal_bridge"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("base_frame_candidates", "base_link,base_footprint,odin1_base_link");
    declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    declare_parameter<std::string>("goal_point_topic", "/goal_point");
    declare_parameter<std::string>("start_point_topic", "/start_point");
    declare_parameter<bool>("publish_start_from_tf", true);

    tf_buffer_.setCreateTimerInterface(
      std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(), get_node_timers_interface()));

    const auto goal_pose_topic = get_parameter("goal_pose_topic").as_string();
    const auto goal_point_topic = get_parameter("goal_point_topic").as_string();
    const auto start_point_topic = get_parameter("start_point_topic").as_string();

    const auto latched_qos = rclcpp::QoS(1).transient_local().reliable();
    goal_point_pub_ =
      create_publisher<geometry_msgs::msg::PointStamped>(goal_point_topic, latched_qos);
    start_point_pub_ =
      create_publisher<geometry_msgs::msg::PointStamped>(start_point_topic, latched_qos);
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_pose_topic, latched_qos,
      std::bind(&EgoNavigationGoalBridgeNode::onGoalPose, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "goal bridge ready. %s -> %s, start=%s from TF %s->%s",
      goal_pose_topic.c_str(), goal_point_topic.c_str(), start_point_topic.c_str(),
      get_parameter("map_frame").as_string().c_str(),
      get_parameter("base_frame_candidates").as_string().c_str());
  }

private:
  void onGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    const auto map_frame = get_parameter("map_frame").as_string();

    if (get_parameter("publish_start_from_tf").as_bool()) {
      publishStartFromTf(msg->header.stamp);
    }

    geometry_msgs::msg::PointStamped goal;
    goal.header.stamp = msg->header.stamp;
    goal.header.frame_id = msg->header.frame_id.empty() ? map_frame : msg->header.frame_id;
    goal.point.x = msg->pose.position.x;
    goal.point.y = msg->pose.position.y;
    goal.point.z = msg->pose.position.z;
    goal_point_pub_->publish(goal);

    RCLCPP_INFO(
      get_logger(), "Forwarded goal_pose to goal_point [%.3f, %.3f, %.3f] in %s",
      goal.point.x, goal.point.y, goal.point.z, goal.header.frame_id.c_str());
  }

  void publishStartFromTf(const builtin_interfaces::msg::Time & stamp)
  {
    const auto map_frame = get_parameter("map_frame").as_string();

    geometry_msgs::msg::TransformStamped transform;
    std::string used_base_frame;
    std::string last_error;
    for (const auto & base_frame : baseFrameCandidates()) {
      try {
        transform = tf_buffer_.lookupTransform(map_frame, base_frame, tf2::TimePointZero);
        used_base_frame = base_frame;
        break;
      } catch (const tf2::TransformException & ex) {
        last_error = ex.what();
      }
    }

    if (used_base_frame.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Cannot publish /start_point from TF %s to any base candidate: %s",
        map_frame.c_str(), last_error.c_str());
      return;
    }

    geometry_msgs::msg::PointStamped start;
    start.header.stamp = stamp;
    start.header.frame_id = map_frame;
    start.point.x = transform.transform.translation.x;
    start.point.y = transform.transform.translation.y;
    start.point.z = transform.transform.translation.z;
    start_point_pub_->publish(start);
  }

  static std::string trim(const std::string & input)
  {
    std::size_t first = 0;
    while (first < input.size() && std::isspace(static_cast<unsigned char>(input[first]))) {
      ++first;
    }
    std::size_t last = input.size();
    while (last > first && std::isspace(static_cast<unsigned char>(input[last - 1]))) {
      --last;
    }
    return input.substr(first, last - first);
  }

  static std::vector<std::string> splitList(const std::string & text)
  {
    std::vector<std::string> out;
    std::stringstream ss(text);
    std::string item;
    while (std::getline(ss, item, ',')) {
      item = trim(item);
      if (!item.empty() && std::find(out.begin(), out.end(), item) == out.end()) {
        out.push_back(item);
      }
    }
    return out;
  }

  std::vector<std::string> baseFrameCandidates()
  {
    std::vector<std::string> frames;
    const auto base_frame = get_parameter("base_frame").as_string();
    if (!base_frame.empty()) {
      frames.push_back(base_frame);
    }
    for (const auto & frame : splitList(get_parameter("base_frame_candidates").as_string())) {
      if (std::find(frames.begin(), frames.end(), frame) == frames.end()) {
        frames.push_back(frame);
      }
    }
    return frames;
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr start_point_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EgoNavigationGoalBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
