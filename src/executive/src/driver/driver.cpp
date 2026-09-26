#include "executive/driver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kHalfPi = 1.5707963267948966;
constexpr double kWheelRadius = 0.09;  // metres; matches the URDF wheel geometry
constexpr double kHalfWheelbase = 0.32;
constexpr double kHalfTrack = 0.24;
constexpr double kSteeringTolerance = 0.05;  // radians (~2.9 degrees)
constexpr double kCommandEpsilon = 1e-3;

double angle_error(double target, double current) {
  return std::atan2(std::sin(target - current), std::cos(target - current));
}

double wrap_angle(double angle) {
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

namespace executive {

Driver::Driver() : Node("driver") {
  cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&Driver::cmd_vel_callback, this, std::placeholders::_1));

  joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Driver::joint_state_callback, this, std::placeholders::_1));

  steering_publishers_[0] = create_publisher<std_msgs::msg::Float64>(
      "/cmd_pos/fl_steering", 10);
  steering_publishers_[1] = create_publisher<std_msgs::msg::Float64>(
      "/cmd_pos/fr_steering", 10);
  steering_publishers_[2] = create_publisher<std_msgs::msg::Float64>(
      "/cmd_pos/rl_steering", 10);
  steering_publishers_[3] = create_publisher<std_msgs::msg::Float64>(
      "/cmd_pos/rr_steering", 10);

  wheel_publishers_[0] = create_publisher<std_msgs::msg::Float64>(
      "/cmd_vel/fl_wheel", 10);
  wheel_publishers_[1] = create_publisher<std_msgs::msg::Float64>(
      "/cmd_vel/fr_wheel", 10);
  wheel_publishers_[2] = create_publisher<std_msgs::msg::Float64>(
      "/cmd_vel/rl_wheel", 10);
  wheel_publishers_[3] = create_publisher<std_msgs::msg::Float64>(
      "/cmd_vel/rr_wheel", 10);

  // Keep commands alive while waiting for steering feedback.
  control_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&Driver::control_timer_callback, this));

  RCLCPP_INFO(get_logger(), "Swerve driver started");
}

void Driver::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
    if (msg->name[i] == "front_left_steering_joint") {
      current_steering_angles_[0] = msg->position[i];
    } else if (msg->name[i] == "front_right_steering_joint") {
      current_steering_angles_[1] = msg->position[i];
    } else if (msg->name[i] == "rear_left_steering_joint") {
      current_steering_angles_[2] = msg->position[i];
    } else if (msg->name[i] == "rear_right_steering_joint") {
      current_steering_angles_[3] = msg->position[i];
    }
  }
  have_joint_state_ = true;
}

void Driver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  const double vx = msg->linear.x;
  const double vy = msg->linear.y;
  const double wz = msg->angular.z;

  const std::array<double, 4> x = {
      kHalfWheelbase, kHalfWheelbase, -kHalfWheelbase, -kHalfWheelbase};
  const std::array<double, 4> y = {
      kHalfTrack, -kHalfTrack, kHalfTrack, -kHalfTrack};

  for (std::size_t i = 0; i < 4; ++i) {
    const double module_vx = vx - wz * y[i];
    const double module_vy = vy + wz * x[i];
    double target_angle = std::atan2(module_vy, module_vx);
    double wheel_speed = std::hypot(module_vx, module_vy) / kWheelRadius;

    // Use the equivalent reversed-wheel solution when it avoids a large
    // steering rotation.
    if (std::abs(angle_error(target_angle, current_steering_angles_[i])) >
        kHalfPi) {
      target_angle = wrap_angle(target_angle + kPi);
      wheel_speed = -wheel_speed;
    }

    target_steering_angles_[i] = target_angle;
    target_wheel_speeds_[i] = wheel_speed;
  }

  if (std::abs(vx) < kCommandEpsilon && std::abs(vy) < kCommandEpsilon &&
      std::abs(wz) < kCommandEpsilon) {
    target_steering_angles_.fill(0.0);
    target_wheel_speeds_.fill(0.0);
  }

  publish_steering_commands();
}

void Driver::publish_steering_commands() {
  for (std::size_t i = 0; i < 4; ++i) {
    std_msgs::msg::Float64 command;
    command.data = target_steering_angles_[i];
    steering_publishers_[i]->publish(command);
  }
}

void Driver::publish_wheel_commands(const std::array<double, 4> &speeds) {
  for (std::size_t i = 0; i < 4; ++i) {
    std_msgs::msg::Float64 command;
    command.data = speeds[i];
    wheel_publishers_[i]->publish(command);
  }
}

bool Driver::steering_is_aligned() const {
  if (!have_joint_state_) {
    return false;
  }
  for (std::size_t i = 0; i < 4; ++i) {
    if (std::abs(angle_error(target_steering_angles_[i],
                             current_steering_angles_[i])) >
        kSteeringTolerance) {
      return false;
    }
  }
  return true;
}

void Driver::control_timer_callback() {
  publish_steering_commands();
  if (steering_is_aligned()) {
    publish_wheel_commands(target_wheel_speeds_);
  } else {
    publish_wheel_commands({0.0, 0.0, 0.0, 0.0});
  }
}

}  // namespace executive
