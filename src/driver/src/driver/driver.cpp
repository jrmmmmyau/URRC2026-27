#include "driver/driver.hpp"

#include <chrono>
#include <cmath>
#include <functional>

namespace driver {

namespace {
double wheel_radius = 0.09;
double wheel_x = 0.32;
double wheel_y = 0.24;
double steering_tolerance = 0.075;
constexpr double steering_release_tolerance = 0.15;  // hysteresis, radians
constexpr double stopped_speed_tolerance = 0.1;  // wheel rad/s
constexpr auto feedback_timeout = std::chrono::milliseconds(500);
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

void Driver::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {

  // Robot coordinates:
  // +X = forward
  // +Y = left
  // +Z rotation = counterclockwise
  const double vx = msg->linear.x;   // m/s
  const double vy = msg->linear.y;   // m/s
  const double wz = msg->angular.z;  // rad/s

  constexpr double pi = 3.14159265358979323846;
  constexpr double linear_deadband = 0.01;   // m/s
  constexpr double angular_deadband = 0.01;  // rad/s

  // Stop driving without changing steering targets.
  auto stop_wheels = [&]() {
    fl_speed_cmd_ = 0.0;
    fr_speed_cmd_ = 0.0;
    rl_speed_cmd_ = 0.0;
    rr_speed_cmd_ = 0.0;
  };

  // Reject invalid input.
  if (!std::isfinite(vx) ||
      !std::isfinite(vy) ||
      !std::isfinite(wz)) {
    stop_wheels();
    return;
  }

  // Stop command.
  if (std::hypot(vx, vy) < linear_deadband &&
      std::abs(wz) < angular_deadband) {
    stop_wheels();
    return;
  }

  // Steering optimization requires current joint feedback.
  if (!feedback_is_fresh()) {
    stop_wheels();
    return;
  }

  // Wrap an angle to [-pi, pi].
  auto wrap_angle = [](double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
  };

  // Calculate steering angle and rolling speed for one module.
  auto calculate_module =
      [&](double module_x,
          double module_y,
          double current_angle,
          double &target_angle,
          double &speed_cmd) {

    // Translation + velocity caused by chassis rotation.
    const double module_vx = vx - wz * module_y;
    const double module_vy = vy + wz * module_x;

    const double module_speed =
        std::hypot(module_vx, module_vy);

    // Retain steering direction if this module should not move.
    if (module_speed < 1e-6) {
      speed_cmd = 0.0;
      return;
    }

    double desired_angle =
        std::atan2(module_vy, module_vx);

    // Convert ground speed (m/s) to wheel speed (rad/s).
    speed_cmd = module_speed / wheel_radius;

    // Minimize steering travel by reversing wheel direction
    // when the desired steering change exceeds 90 degrees.
    const double steering_error =
        wrap_angle(desired_angle - current_angle);

    if (std::abs(steering_error) > pi / 2.0) {
      desired_angle += pi;
      speed_cmd = -speed_cmd;
    }

    // Choose an equivalent angle near the previous target.
    const double target_change =
        wrap_angle(desired_angle - target_angle);

    // Avoid triggering braking for numerical roundoff alone.
    if (std::abs(target_change) > 1e-9) {
      target_angle += target_change;
    }
  };

  // Module positions relative to the chassis center.
  calculate_module(
      +wheel_x, +wheel_y,
      fl_angle_, fl_target_, fl_speed_cmd_);

  calculate_module(
      +wheel_x, -wheel_y,
      fr_angle_, fr_target_, fr_speed_cmd_);

  calculate_module(
      -wheel_x, +wheel_y,
      rl_angle_, rl_target_, rl_speed_cmd_);

  calculate_module(
      -wheel_x, -wheel_y,
      rr_angle_, rr_target_, rr_speed_cmd_);
}

double angle_error(double target, double current){
  return std::atan2(std::sin(target-current), std::cos(target-current));
}

bool Driver::feedback_is_fresh() const {
  const auto now = std::chrono::steady_clock::now();
  for (const auto &stamp : feedback_times_) {
    if (stamp == std::chrono::steady_clock::time_point{} ||
        now - stamp >= feedback_timeout) {
      return false;
    }
  }
  return true;
}

bool Driver::wheels_are_aligned(double tolerance) const {
  if (!feedback_is_fresh()) {
    return false;
  }

  // RCLCPP_INFO(this->get_logger(),
  //   "Errors: FL=%.3f | FR=%.3f | RL=%.3f | RR=%.3f",
  //   angle_error(fl_target_, fl_angle_),
  //   angle_error(fr_target_, fr_angle_),
  //   angle_error(rl_target_, rl_angle_),
  //   angle_error(rr_target_, rr_angle_));

  return std::abs(angle_error(fl_target_ , fl_angle_)) < tolerance &&
         std::abs(angle_error(fr_target_ , fr_angle_)) < tolerance &&
         std::abs(angle_error(rl_target_ , rl_angle_)) < tolerance &&
         std::abs(angle_error(rr_target_ , rr_angle_)) < tolerance;
}

bool Driver::wheels_are_stopped() const {
  return feedback_is_fresh() &&
         std::abs(fl_speed_actual_) < stopped_speed_tolerance &&
         std::abs(fr_speed_actual_) < stopped_speed_tolerance &&
         std::abs(rl_speed_actual_) < stopped_speed_tolerance &&
         std::abs(rr_speed_actual_) < stopped_speed_tolerance;
}

void Driver::publish_commands() {
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
}

}  // namespace driver
