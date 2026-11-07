#pragma once

#include <array>
#include <chrono>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64.hpp"

namespace teleop_driver {

class Teleop_driver : public rclcpp::Node {
public:
  Teleop_driver();

private:
  using Float64Publisher = rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr;

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void publish_commands();
  bool wheels_are_aligned(double tolerance = 0.075) const;
  bool feedback_is_fresh() const;
  bool wheels_are_stopped() const;

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

  Float64Publisher camera_vel_pub_;

  double fl_angle_ = 0.0;
  double fr_angle_ = 0.0;
  double rl_angle_ = 0.0;
  double rr_angle_ = 0.0;

  double fl_target_ = 0.0;
  double fr_target_ = 0.0;
  double rl_target_ = 0.0;
  double rr_target_ = 0.0;

  double fl_speed_cmd_ = 0.0;
  double fr_speed_cmd_ = 0.0;
  double rl_speed_cmd_ = 0.0;
  double rr_speed_cmd_ = 0.0;

  double fl_speed_actual_ = 0.0;
  double fr_speed_actual_ = 0.0;
  double rl_speed_actual_ = 0.0;
  double rr_speed_actual_ = 0.0;

  // Hold these steering commands while the wheels brake.
  double fl_applied_target_ = 0.0;
  double fr_applied_target_ = 0.0;
  double rl_applied_target_ = 0.0;
  double rr_applied_target_ = 0.0;

  enum class Transition { BRAKING, STEERING, DRIVING };
  Transition transition_ = Transition::BRAKING;
  // Each steering position and wheel velocity has its own receive time.
  // JointState publishers may send these in separate messages.
  std::array<std::chrono::steady_clock::time_point, 8> feedback_times_{};
  std::chrono::steady_clock::time_point last_wait_log_{};
};

}  // namespace teleop_driver
