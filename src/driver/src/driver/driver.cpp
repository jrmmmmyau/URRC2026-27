#include "driver/driver.hpp"

#include <chrono>
#include <cmath>
#include <functional>

namespace driver {

namespace {
constexpr double wheel_radius = 0.09;
constexpr double wheel_x = 0.32;
constexpr double wheel_y = 0.24;
constexpr double steering_tolerance = 0.05;
}

Driver::Driver() : Node("driver") {
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&Driver::cmd_vel_callback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Driver::joint_state_callback, this, std::placeholders::_1));

  fl_steering_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/cmd_pos/fl_steering", 10);
  fr_steering_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/cmd_pos/fr_steering", 10);
  rl_steering_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/cmd_pos/rl_steering", 10);
  rr_steering_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/cmd_pos/rr_steering", 10);

  fl_wheel_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/cmd_vel/fl_wheel", 10);
  fr_wheel_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/cmd_vel/fr_wheel", 10);
  rl_wheel_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/cmd_vel/rl_wheel", 10);
  rr_wheel_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/cmd_vel/rr_wheel", 10);

  timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&Driver::publish_commands, this));

  RCLCPP_INFO(get_logger(), "Driver node started");
}

void Driver::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (msg->name[i] == "front_left_steering_joint") {
      fl_angle_ = msg->position[i];
    } else if (msg->name[i] == "front_right_steering_joint") {
      fr_angle_ = msg->position[i];
    } else if (msg->name[i] == "rear_left_steering_joint") {
      rl_angle_ = msg->position[i];
    } else if (msg->name[i] == "rear_right_steering_joint") {
      rr_angle_ = msg->position[i];
    }
  }
  received_joint_state_ = true;
}

void Driver::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  const double vx = msg->linear.x;
  const double angular_z = msg->angular.z;

  // Drive forward with all four wheels straight.
  if (std::abs(vx) > 0.01 && std::abs(angular_z) < steering_tolerance) {
    fl_target_ = 0.0;
    fr_target_ = 0.0;
    rl_target_ = 0.0;
    rr_target_ = 0.0;

    const double wheel_speed = vx / wheel_radius;
    fl_speed_ = wheel_speed;
    fr_speed_ = wheel_speed;
    rl_speed_ = wheel_speed;
    rr_speed_ = wheel_speed;
  }

  // Spin in place. The steering angles are the same ones used by ng/driver.
  else if (std::abs(angular_z) > steering_tolerance) {
    const double wheel_speed =
        std::abs(angular_z) * std::hypot(wheel_x, wheel_y) / wheel_radius;

    fl_target_ = std::atan2(wheel_x, -wheel_y);
    fr_target_ = std::atan2(wheel_x, wheel_y);
    rl_target_ = std::atan2(-wheel_x, -wheel_y);
    rr_target_ = std::atan2(-wheel_x, wheel_y);
    RCLCPP_INFO(this->get_logger(), "FL: %.3f | FR: %.3f | RL: %.3f | RR: %.3f",
            fl_target_, fr_target_, rl_target_, rr_target_);

    if (angular_z > 0.0) {
      fl_speed_ = wheel_speed;
      fr_speed_ = wheel_speed;
      rl_speed_ = wheel_speed;
      rr_speed_ = wheel_speed;
    } else {
      fl_speed_ = -wheel_speed;
      fr_speed_ = -wheel_speed;
      rl_speed_ = -wheel_speed;
      rr_speed_ = -wheel_speed;
    }
  }

  // Stop the wheels and return the steering to straight ahead.
  else {
    fl_target_ = 0.0;
    fr_target_ = 0.0;
    rl_target_ = 0.0;
    rr_target_ = 0.0;
    fl_speed_ = 0.0;
    fr_speed_ = 0.0;
    rl_speed_ = 0.0;
    rr_speed_ = 0.0;
  }
}

bool Driver::wheels_are_aligned() const {
  if (!received_joint_state_) {
    return false;
  }

  return std::abs(fl_target_ - fl_angle_) < steering_tolerance &&
         std::abs(fr_target_ - fr_angle_) < steering_tolerance &&
         std::abs(rl_target_ - rl_angle_) < steering_tolerance &&
         std::abs(rr_target_ - rr_angle_) < steering_tolerance;
}

void Driver::publish_commands() {
  std_msgs::msg::Float64 fl_steering;
  std_msgs::msg::Float64 fr_steering;
  std_msgs::msg::Float64 rl_steering;
  std_msgs::msg::Float64 rr_steering;

  fl_steering.data = fl_target_;
  fr_steering.data = fr_target_;
  rl_steering.data = rl_target_;
  rr_steering.data = rr_target_;

  fl_steering_pub_->publish(fl_steering);
  fr_steering_pub_->publish(fr_steering);
  rl_steering_pub_->publish(rl_steering);
  rr_steering_pub_->publish(rr_steering);

  double fl_speed = 0.0;
  double fr_speed = 0.0;
  double rl_speed = 0.0;
  double rr_speed = 0.0;

  if (wheels_are_aligned()) {
    fl_speed = fl_speed_;
    fr_speed = fr_speed_;
    rl_speed = rl_speed_;
    rr_speed = rr_speed_;
  }

  std_msgs::msg::Float64 fl_wheel;
  std_msgs::msg::Float64 fr_wheel;
  std_msgs::msg::Float64 rl_wheel;
  std_msgs::msg::Float64 rr_wheel;

  fl_wheel.data = fl_speed;
  fr_wheel.data = fr_speed;
  rl_wheel.data = rl_speed;
  rr_wheel.data = rr_speed;

  fl_wheel_pub_->publish(fl_wheel);
  fr_wheel_pub_->publish(fr_wheel);
  rl_wheel_pub_->publish(rl_wheel);
  rr_wheel_pub_->publish(rr_wheel);
}

}  // namespace driver
