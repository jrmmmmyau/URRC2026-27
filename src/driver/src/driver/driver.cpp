#include "driver/driver.hpp"

#include <chrono>
#include <cmath>
#include <functional>

namespace {
const double wheel_radius = 0.09;
const double half_length = 0.32;
const double half_width = 0.24;
const double angle_tolerance = 0.05;
const double pi = 3.14159265358979323846;

double angle_difference(double target, double current) {
  return std::atan2(std::sin(target - current),
                    std::cos(target - current));
}

double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

namespace driver {

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
  const double vy = msg->linear.y;
  const double wz = msg->angular.z;

  const double fl_vx = vx - wz * half_width;
  const double fl_vy = vy + wz * half_length;
  const double fr_vx = vx + wz * half_width;
  const double fr_vy = vy + wz * half_length;
  const double rl_vx = vx - wz * half_width;
  const double rl_vy = vy - wz * half_length;
  const double rr_vx = vx + wz * half_width;
  const double rr_vy = vy - wz * half_length;

  fl_target_ = std::atan2(fl_vy, fl_vx);
  fr_target_ = std::atan2(fr_vy, fr_vx);
  rl_target_ = std::atan2(rl_vy, rl_vx);
  rr_target_ = std::atan2(rr_vy, rr_vx);

  fl_speed_ = std::hypot(fl_vx, fl_vy) / wheel_radius;
  fr_speed_ = std::hypot(fr_vx, fr_vy) / wheel_radius;
  rl_speed_ = std::hypot(rl_vx, rl_vy) / wheel_radius;
  rr_speed_ = std::hypot(rr_vx, rr_vy) / wheel_radius;

  // Reverse a wheel if that avoids turning its steering joint more than 90°.
  if (std::abs(angle_difference(fl_target_, fl_angle_)) > pi / 2) {
    fl_target_ = wrap_angle(fl_target_ + pi);
    fl_speed_ = -fl_speed_;
  }
  if (std::abs(angle_difference(fr_target_, fr_angle_)) > pi / 2) {
    fr_target_ = wrap_angle(fr_target_ + pi);
    fr_speed_ = -fr_speed_;
  }
  if (std::abs(angle_difference(rl_target_, rl_angle_)) > pi / 2) {
    rl_target_ = wrap_angle(rl_target_ + pi);
    rl_speed_ = -rl_speed_;
  }
  if (std::abs(angle_difference(rr_target_, rr_angle_)) > pi / 2) {
    rr_target_ = wrap_angle(rr_target_ + pi);
    rr_speed_ = -rr_speed_;
  }

  if (std::abs(vx) < 0.001 && std::abs(vy) < 0.001 &&
      std::abs(wz) < 0.001) {
    fl_target_ = fr_target_ = rl_target_ = rr_target_ = 0.0;
    fl_speed_ = fr_speed_ = rl_speed_ = rr_speed_ = 0.0;
  }
}

bool Driver::wheels_are_aligned() const {
  if (!received_joint_state_) {
    return false;
  }

  return std::abs(angle_difference(fl_target_, fl_angle_)) < angle_tolerance &&
         std::abs(angle_difference(fr_target_, fr_angle_)) < angle_tolerance &&
         std::abs(angle_difference(rl_target_, rl_angle_)) < angle_tolerance &&
         std::abs(angle_difference(rr_target_, rr_angle_)) < angle_tolerance;
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
