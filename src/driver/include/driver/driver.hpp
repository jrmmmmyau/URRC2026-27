#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace driver {

class Driver : public rclcpp::Node {
public:
  Driver();

private:
  using Float64Publisher = rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr;

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_commands();
  bool wheels_are_aligned() const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  Float64Publisher fl_steering_pub_;
  Float64Publisher fr_steering_pub_;
  Float64Publisher rl_steering_pub_;
  Float64Publisher rr_steering_pub_;

  Float64Publisher fl_wheel_pub_;
  Float64Publisher fr_wheel_pub_;
  Float64Publisher rl_wheel_pub_;
  Float64Publisher rr_wheel_pub_;

  double fl_angle_ = 0.0;
  double fr_angle_ = 0.0;
  double rl_angle_ = 0.0;
  double rr_angle_ = 0.0;

  double fl_target_ = 0.0;
  double fr_target_ = 0.0;
  double rl_target_ = 0.0;
  double rr_target_ = 0.0;

  double fl_speed_ = 0.0;
  double fr_speed_ = 0.0;
  double rl_speed_ = 0.0;
  double rr_speed_ = 0.0;

  bool received_joint_state_ = false;
};

}  // namespace driver
