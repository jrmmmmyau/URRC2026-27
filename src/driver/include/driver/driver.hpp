#pragma once

#include <array>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace driver {

class Driver : public rclcpp::Node {
public:
  Driver();

private:
  using FloatPublisher = rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr;

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void control_timer_callback();
  void publish_steering_commands();
  void publish_wheel_commands(const std::array<double, 4> &speeds);
  bool steering_is_aligned() const;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::array<FloatPublisher, 4> steering_publishers_;
  std::array<FloatPublisher, 4> wheel_publishers_;
  std::array<double, 4> current_steering_angles_{};
  std::array<double, 4> target_steering_angles_{};
  std::array<double, 4> target_wheel_speeds_{};
  bool have_joint_state_{false};
};

}  // namespace driver
