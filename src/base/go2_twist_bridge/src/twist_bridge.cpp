#include <algorithm>
#include <cmath>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <unitree_api/msg/request.hpp>

#include "rclcpp/rclcpp.hpp"
#include <sport_model.hpp>

using namespace std::placeholders;

class TwistBridge : public rclcpp::Node
{
public:
  TwistBridge(): Node("twist_bridge_node")
  {
    declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    declare_parameter<std::string>("request_topic", "/api/sport/request");
    declare_parameter<double>("linear_x_scale", 1.0);
    declare_parameter<double>("linear_y_scale", 1.0);
    declare_parameter<double>("angular_z_scale", 1.0);
    declare_parameter<double>("max_linear_x", 0.6);
    declare_parameter<double>("max_linear_y", 0.4);
    declare_parameter<double>("max_angular_z", 1.0);
    declare_parameter<double>("linear_deadband", 0.02);
    declare_parameter<double>("angular_deadband", 0.02);
    declare_parameter<bool>("send_stopmove_when_zero", true);

    const auto cmd_vel_topic = get_parameter("cmd_vel_topic").as_string();
    const auto request_topic = get_parameter("request_topic").as_string();

    RCLCPP_INFO(
      this->get_logger(),
      "twist_bridge_node online. cmd_vel=%s request=%s scale=(%.2f, %.2f, %.2f) "
      "limit=(%.2f, %.2f, %.2f)",
      cmd_vel_topic.c_str(), request_topic.c_str(),
      get_parameter("linear_x_scale").as_double(),
      get_parameter("linear_y_scale").as_double(),
      get_parameter("angular_z_scale").as_double(),
      get_parameter("max_linear_x").as_double(),
      get_parameter("max_linear_y").as_double(),
      get_parameter("max_angular_z").as_double());

    request_pub_ = this->create_publisher<unitree_api::msg::Request>(request_topic, 10);
    twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(cmd_vel_topic, 10,
      std::bind(&TwistBridge::TwistCallback, this, _1)
    );
  }
private:
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr request_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

  static double applyDeadband(double value, double deadband)
  {
    return std::abs(value) < deadband ? 0.0 : value;
  }

  double scaledAndClamped(double value, const char * scale_param, const char * limit_param) const
  {
    const double scaled = value * get_parameter(scale_param).as_double();
    const double limit = std::max(0.0, get_parameter(limit_param).as_double());
    return std::clamp(scaled, -limit, limit);
  }

  void TwistCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
  {
    unitree_api::msg::Request req;

    double x = scaledAndClamped(twist->linear.x, "linear_x_scale", "max_linear_x");
    double y = scaledAndClamped(twist->linear.y, "linear_y_scale", "max_linear_y");
    double z = scaledAndClamped(twist->angular.z, "angular_z_scale", "max_angular_z");

    x = applyDeadband(x, get_parameter("linear_deadband").as_double());
    y = applyDeadband(y, get_parameter("linear_deadband").as_double());
    z = applyDeadband(z, get_parameter("angular_deadband").as_double());

    if (x == 0.0 && y == 0.0 && z == 0.0) {
      req.header.identity.api_id = get_parameter("send_stopmove_when_zero").as_bool() ?
        ROBOT_SPORT_API_ID_STOPMOVE : ROBOT_SPORT_API_ID_BALANCESTAND;
    } else {
      nlohmann::json js;
      js["x"] = x;
      js["y"] = y;
      js["z"] = z;
      req.parameter = js.dump();
      req.header.identity.api_id = ROBOT_SPORT_API_ID_MOVE;
    }

    request_pub_->publish(req);
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 500,
      "cmd_vel in=(%.3f, %.3f, %.3f) go2=(%.3f, %.3f, %.3f) api_id=%ld",
      twist->linear.x, twist->linear.y, twist->angular.z,
      x, y, z, req.header.identity.api_id);
  }
};

int main(int argc, char*argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistBridge>());
  rclcpp::shutdown();
  
  return 0;
}
