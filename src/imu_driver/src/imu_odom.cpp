#include <array>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

using Vec3 = std::array<double, 3>;

// Rotate vector v by quaternion q.
// Used to turn "acceleration as the IMU sees it" into
// "acceleration in the world's fixed directions".
static Vec3 rotate(const geometry_msgs::msg::Quaternion & q, const Vec3 & v)
{
  // t = 2 * (q.xyz cross v)
  const double tx = 2.0 * (q.y * v[2] - q.z * v[1]);
  const double ty = 2.0 * (q.z * v[0] - q.x * v[2]);
  const double tz = 2.0 * (q.x * v[1] - q.y * v[0]);
  // v' = v + w*t + (q.xyz cross t)
  return {
    v[0] + q.w * tx + (q.y * tz - q.z * ty),
    v[1] + q.w * ty + (q.z * tx - q.x * tz),
    v[2] + q.w * tz + (q.x * ty - q.y * tx)};
}

// Same rotation, but backwards (world -> robot's own frame).
static Vec3 rotate_inverse(geometry_msgs::msg::Quaternion q, const Vec3 & v)
{
  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  return rotate(q, v);
}

class ImuOdom : public rclcpp::Node
{
public:
  ImuOdom() : Node("imu_odom")
  {
    // Input: raw IMU readings
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) { on_imu(msg); });

    // Output: the robot's estimated position and velocity
    pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  }

private:
  static constexpr double kGravity = 9.80665;  // m/s^2

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // ---- Step 0: how much time passed since the last reading? ----
    const rclcpp::Time now = this->now();
    if (!has_last_time_) {
      last_time_ = now;
      has_last_time_ = true;
      return;  // need two readings before we can measure a time gap
    }
    const double dt = (now - last_time_).seconds();
    last_time_ = now;
    if (dt <= 0.0 || dt > 0.5) {
      return;  // skip nonsense or huge gaps (would cause big jumps)
    }

    // ---- Step 1: rotate acceleration into world directions ----
    const Vec3 accel_body = {
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z};
    Vec3 accel_world = rotate(msg->orientation, accel_body);

    // ---- Step 2: remove gravity (IMU always feels ~9.81 m/s^2 up) ----
    accel_world[2] -= kGravity;

    // ---- Step 3: integrate: accel -> velocity -> position ----
    for (int i = 0; i < 3; ++i) {
      vel_[i] += accel_world[i] * dt;
      pos_[i] += vel_[i] * dt;
    }

    // ---- Step 4: package it up and publish on /odom ----
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now;
    odom.header.frame_id = "odom";       // position is measured in this frame
    odom.child_frame_id = "base_link";   // ...and describes this frame (the robot)

    odom.pose.pose.position.x = pos_[0];
    odom.pose.pose.position.y = pos_[1];
    odom.pose.pose.position.z = pos_[2];
    odom.pose.pose.orientation = msg->orientation;  // IMU already gives this

    // Velocity in Odometry is expressed from the robot's point of view
    const Vec3 vel_body = rotate_inverse(msg->orientation, vel_);
    odom.twist.twist.linear.x = vel_body[0];
    odom.twist.twist.linear.y = vel_body[1];
    odom.twist.twist.linear.z = vel_body[2];
    odom.twist.twist.angular = msg->angular_velocity;  // turn rate, straight from IMU

    pub_->publish(odom);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  Vec3 pos_{0.0, 0.0, 0.0};  // meters, world frame
  Vec3 vel_{0.0, 0.0, 0.0};  // m/s, world frame
  rclcpp::Time last_time_;
  bool has_last_time_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuOdom>());
  rclcpp::shutdown();
  return 0;
}