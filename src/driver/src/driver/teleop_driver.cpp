#include "driver/teleop_driver.hpp"

#include <chrono>
#include <cmath>
#include <functional>

namespace teleop_driver {

namespace {
double wheel_radius = 0.09;
double wheel_x = 0.32;
double wheel_y = 0.24;
double steering_tolerance = 0.075;
constexpr double steering_release_tolerance = 0.15;  // hysteresis, radians
constexpr double stopped_speed_tolerance = 0.1;  // wheel rad/s
constexpr auto feedback_timeout = std::chrono::milliseconds(500);
}

Teleop_driver::Teleop_driver() : Node("teleop_driver") {
  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&Teleop_driver::cmd_vel_callback, this, std::placeholders::_1));

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Teleop_driver::joint_state_callback, this, std::placeholders::_1));

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
  camera_vel_pub_ = create_publisher<std_msgs::msg::Float64>( //this is for the camera, delete this in the future
    "/cmd_vel/camera", 10);
    
  
  
  timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&Teleop_driver::publish_commands, this));

  RCLCPP_INFO(get_logger(), "Teleop_driver node started");
}

void Teleop_driver::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  // Accumulate independent joint updates, without refreshing missing values.
  const auto received_at = std::chrono::steady_clock::now();
  const char *steering_names[] = {
      "front_left_steering_joint", "front_right_steering_joint",
      "rear_left_steering_joint", "rear_right_steering_joint"};
  const char *wheel_names[] = {
      "front_left_wheel_spin_joint", "front_right_wheel_spin_joint",
      "rear_left_wheel_spin_joint", "rear_right_wheel_spin_joint"};
  double *angles[] = {&fl_angle_, &fr_angle_, &rl_angle_, &rr_angle_};
  double *speeds[] = {&fl_speed_actual_, &fr_speed_actual_,
                      &rl_speed_actual_, &rr_speed_actual_};
  for (size_t i = 0; i < msg->name.size(); ++i) {
    for (size_t wheel = 0; wheel < 4; ++wheel) {
      if (msg->name[i] == steering_names[wheel]) {
        if (i < msg->position.size() && std::isfinite(msg->position[i])) {
          *angles[wheel] = msg->position[i];
          feedback_times_[wheel] = received_at;
        } else {
          feedback_times_[wheel] = {};  // explicitly invalid feedback
        }
      }
      if (msg->name[i] == wheel_names[wheel]) {
        if (i < msg->velocity.size() && std::isfinite(msg->velocity[i])) {
          *speeds[wheel] = msg->velocity[i];
          feedback_times_[wheel + 4] = received_at;
        } else {
          feedback_times_[wheel + 4] = {};
        }
      }
    }
  }

}

void Teleop_driver::cmd_vel_callback(
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
    fl_speed_cmd_ = wheel_speed;
    fr_speed_cmd_ = wheel_speed;
    rl_speed_cmd_ = wheel_speed;
    rr_speed_cmd_ = wheel_speed;
  }

  // Spin in place with each wheel tangent to its circle around the chassis center.
  else if (std::abs(angular_z) > steering_tolerance) {
    const double wheel_speed =
        std::abs(angular_z) * std::hypot(wheel_x, wheel_y) / wheel_radius;

    // A wheel at (x, y) needs velocity (-angular_z*y, angular_z*x).
    // Left wheels roll backward, so their steering angle is negative.
    const double spin_angle = std::atan2(wheel_x, wheel_y);
    fl_target_ = -spin_angle;
    fr_target_ = spin_angle;
    rl_target_ = spin_angle;
    rr_target_ = -spin_angle;

    if (angular_z > 0.0) {
      fl_speed_cmd_ = -wheel_speed;
      fr_speed_cmd_ = wheel_speed;
      rl_speed_cmd_ = -wheel_speed;
      rr_speed_cmd_ = wheel_speed;
    } else {
      fl_speed_cmd_ = wheel_speed;
      fr_speed_cmd_ = -wheel_speed;
      rl_speed_cmd_ = wheel_speed;
      rr_speed_cmd_ = -wheel_speed;
    }
  }

  // Stop the wheels while retaining the requested steering position.
  else {
    fl_speed_cmd_ = 0.0;
    fr_speed_cmd_ = 0.0;
    rl_speed_cmd_ = 0.0;
    rr_speed_cmd_ = 0.0;
  }
}

double angle_error(double target, double current){
  return std::atan2(std::sin(target-current), std::cos(target-current));
}

bool Teleop_driver::feedback_is_fresh() const {
  const auto now = std::chrono::steady_clock::now();
  for (const auto &stamp : feedback_times_) {
    if (stamp == std::chrono::steady_clock::time_point{} ||
        now - stamp >= feedback_timeout) {
      return false;
    }
  }
  return true;
}

bool Teleop_driver::wheels_are_aligned(double tolerance) const {
  if (!feedback_is_fresh()) {
    return false;
  }

  // RCLCPP_INFO(this->get_logger(),
  //     "Errors: FL=%.3f | FR=%.3f | RL=%.3f | RR=%.3f",
  //     angle_error(fl_target_, fl_angle_),
  //     angle_error(fr_target_, fr_angle_),
  //     angle_error(rl_target_, rl_angle_),
  //     angle_error(rr_target_, rr_angle_));

  return std::abs(angle_error(fl_target_ , fl_angle_)) < tolerance &&
         std::abs(angle_error(fr_target_ , fr_angle_)) < tolerance &&
         std::abs(angle_error(rl_target_ , rl_angle_)) < tolerance &&
         std::abs(angle_error(rr_target_ , rr_angle_)) < tolerance;
}

bool Teleop_driver::wheels_are_stopped() const {
  return feedback_is_fresh() &&
         std::abs(fl_speed_actual_) < stopped_speed_tolerance &&
         std::abs(fr_speed_actual_) < stopped_speed_tolerance &&
         std::abs(rl_speed_actual_) < stopped_speed_tolerance &&
         std::abs(rr_speed_actual_) < stopped_speed_tolerance;
}

void Teleop_driver::publish_commands() {
  const bool feedback_valid = feedback_is_fresh();
  const bool target_changed =
      fl_target_ != fl_applied_target_ || fr_target_ != fr_applied_target_ ||
      rl_target_ != rl_applied_target_ || rr_target_ != rr_applied_target_;

  if (!feedback_valid || target_changed) {
    transition_ = Transition::BRAKING;
  }
  if (feedback_valid) {
    if (transition_ == Transition::BRAKING && wheels_are_stopped()) {
      fl_applied_target_ = fl_target_;
      fr_applied_target_ = fr_target_;
      rl_applied_target_ = rl_target_;
      rr_applied_target_ = rr_target_;
      transition_ = Transition::STEERING;
    }
    if (transition_ == Transition::STEERING &&
        wheels_are_aligned(steering_tolerance)) {
      transition_ = Transition::DRIVING;
    } else if (transition_ == Transition::DRIVING &&
               !wheels_are_aligned(steering_release_tolerance)) {
      transition_ = Transition::BRAKING;
    }
  }

  std_msgs::msg::Float64 fl_steering;
  std_msgs::msg::Float64 fr_steering;
  std_msgs::msg::Float64 rl_steering;
  std_msgs::msg::Float64 rr_steering;

  fl_steering.data = fl_applied_target_;
  fr_steering.data = fr_applied_target_;
  rl_steering.data = rl_applied_target_;
  rr_steering.data = rr_applied_target_;

  fl_steering_pub_->publish(fl_steering);
  fr_steering_pub_->publish(fr_steering);
  rl_steering_pub_->publish(rl_steering);
  rr_steering_pub_->publish(rr_steering);

  double fl_speed = 0.0;
  double fr_speed = 0.0;
  double rl_speed = 0.0;
  double rr_speed = 0.0;

  if (feedback_valid && transition_ == Transition::DRIVING) {
    fl_speed = fl_speed_cmd_;
    fr_speed = fr_speed_cmd_;
    rl_speed = rl_speed_cmd_;
    rr_speed = rr_speed_cmd_;
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
  std_msgs::msg::Float64 camera_msg;
  camera_msg.data = 1.0;
  camera_vel_pub_->publish(camera_msg);
}

}  // namespace teleop_driver
