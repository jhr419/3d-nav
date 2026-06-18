#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/low_state.hpp>
#include <unitree_go/msg/imu_state.hpp>
#include <unitree_go/msg/motor_state.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

//发布里程计消息
  //了解消息字段
  //从哪获取? 物理机器人已经发布了相关话题
  //先订阅状态话题,再解析转换成里程计消息,发布出去
//广播里程计相关坐标变化
  //发布base->odom的相对关系
  //从哪来? 可以从里程计数据中获取
  //发布即可
//发布关节状态信息
  //了解关节状态信息
  //怎么获取? 机器人低层状态信息 lowstate
  //先订阅低层状态信息,再解析转换成关节信息,发布即可

using namespace std::placeholders;

class Driver:public rclcpp::Node{
public:
  Driver():Node("driver_node"){
    RCLCPP_INFO(this->get_logger(), "Driver Node 创建");

    //声明参数
    this->declare_parameter<std::string>("odom_frame", "odom");
    this->declare_parameter<std::string>("base_frame", "base");
    this->declare_parameter("publish_tf",true);

    //获取参数
    odom_frame = this->get_parameter("odom_frame").as_string();
    base_frame = this->get_parameter("base_frame").as_string();
    publish_tf = this->get_parameter("publish_tf").as_bool();//默认禁用坐标变换,
    //坐标变换广播器
    tf_bro_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    mode_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
      "/lf/sportmodestate", 10,
      std::bind(&Driver::ModeCallback, this, _1));
    //发布关节
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    //订阅go2实体机器人上的 底层状态消息，主要是电机数据
    low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
      "/lf/lowstate", 10,
      std::bind(&Driver::LowStateCallback, this, _1)
    );    
  }
private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bro_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr mode_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  //动态调整参数
  std::string odom_frame, base_frame;
  bool publish_tf;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

  void LowStateCallback(const unitree_go::msg::LowState::SharedPtr low_state)
  {//订阅低层信息获取关节状态,组织消息并发布
    sensor_msgs::msg::JointState joint_state;

    joint_state.header.stamp = this->now();
    joint_state.name = {
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint"
    };//关节名称

    //遍历低层关节数据
    for (size_t i = 0; i < 12; i++)
    {
      auto motor = low_state->motor_state[i];
      joint_state.position.push_back(motor.q);
    }

    joint_state_pub_->publish(joint_state);
  }
  void ModeCallback(const unitree_go::msg::SportModeState::SharedPtr mode)
  {
    nav_msgs::msg::Odometry odom;

    odom.header.stamp.sec = mode->stamp.sec;
    odom.header.stamp.nanosec = mode->stamp.nanosec;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id= base_frame;
    //位姿
    odom.pose.pose.position.x = mode->position[0];
    odom.pose.pose.position.y = mode->position[1];
    odom.pose.pose.position.z = mode->position[2];
    odom.pose.pose.orientation.w = mode->imu_state.quaternion[0];
    odom.pose.pose.orientation.x = mode->imu_state.quaternion[1];
    odom.pose.pose.orientation.y = mode->imu_state.quaternion[2];
    odom.pose.pose.orientation.z = mode->imu_state.quaternion[3];

    //velocity
    odom.twist.twist.linear.x = mode->velocity[0];
    odom.twist.twist.linear.y = mode->velocity[1];
    odom.twist.twist.linear.z = mode->velocity[2];

    odom.twist.twist.angular.z = mode->yaw_speed;
    
    odom_pub_->publish(odom);

    if(!publish_tf)//如果不发布tf 直接跳过后面代码
    {
      return;
    }

    //发布一次坐标变换
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = odom_frame;
    transform.child_frame_id = base_frame;

    //设置偏移量
    transform.transform.translation.x = odom.pose.pose.position.x;
    transform.transform.translation.y = odom.pose.pose.position.y;
    transform.transform.translation.z = odom.pose.pose.position.z;

    //设置旋转角度
    transform.transform.rotation = odom.pose.pose.orientation;
    tf_bro_->sendTransform(transform);
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Driver>());
  rclcpp::shutdown();
  return 0;
}

