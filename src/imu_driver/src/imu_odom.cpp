#include <array>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using Vec3 = std::array<double, 3>;

//  Decide how you want to transform a vector between the IMU/body
// frame and the world frame.
static Vec3 rotate(const geometry_msgs::msg::Quaternion & q, const Vec3 & v)
{
  const double tx = 2.0 * (q.y * v[2] - q.z * v[1]);
  const double ty = 2.0 * (q.z * v[0] - q.x * v[2]);
  const double tz = 2.0 * (q.x * v[1] - q.y * v[0]);
  // TODO: return the rotated vector.
  return {
    v[0] + q.w * tx + (q.y * tz - q.z * ty),
    v[1] + q.w * ty + (q.z * tx - q.x * tz),
    v[2] + q.w * tz + (q.x * ty - q.y * tx)};
}

// Provide the reverse transformation.  From world to robot
static Vec3 rotate_inverse(geometry_msgs::msg::Quaternion q, const Vec3 & v)
{
  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  //return the inverse-transformed vector.
  return rotate(q,v);
}

class ImuOdom : public rclcpp::Node
{
public:
  ImuOdom() : Node("imu_odom")
  {
    subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu_data",
      rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg){on_imu(msg);}
    );

    // added to give distinction to driving and parked
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        cmd_is_zero_ = std::abs(msg->linear.x) < 0.01 &&
                       std::abs(msg->linear.y) < 0.01 &&
                       std::abs(msg->angular.z) < 0.01;
      }
    );

    // Connect incoming messages to on_imu().

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    //broadcast tf
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // 1. MUST use Gazebo's physics timestamp, NOT this->now()
    const rclcpp::Time current_time = msg->header.stamp;
    if (!has_last_time_) {
      last_time_ = current_time;
      has_last_time_ = true;
      return;  // need two readings before we can measure a time gap
    }
    const double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (dt <= 0.0 || dt > 0.5) {
      accel_primed_ = false;  // last_accel_ is stale after a skipped sample
      return;  // skip nonsense or huge gaps
    }

    const Vec3 accel_body = {
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z};
    Vec3 accel_world = rotate(msg->orientation, accel_body);

    if (calib_count_ < kCalibSamples) {
      bias_[0] += accel_world[0];
      bias_[1] += accel_world[1];

      if (++calib_count_ == kCalibSamples) {
        bias_[0] /= kCalibSamples;
        bias_[1] /= kCalibSamples;

        RCLCPP_INFO(get_logger(), "IMU bias: x=%.5f y=%.5f", bias_[0], bias_[1]);
      }

      return;  // no integration while calibrating
    }

    // added low-pass filter (smooth solver jitter)
    for (int i = 0; i < 2; ++i) {
      const double a = accel_world[i] - bias_[i];
      filter_[i] = kLpfAlpha * a + (1.0 - kLpfAlpha) * filter_[i];
      accel_world[i] = filter_[i];
    }

    // zero-velocity update
    const double a_mag = std::hypot(accel_world[0], accel_world[1]);
    const double w_mag = std::abs(msg->angular_velocity.z);

    if (a_mag < kAccelStill && w_mag < kGyroStill && cmd_is_zero_) {
      ++still_count_;
    } else {
      still_count_ = 0;
    }

    const bool is_still = still_count_ > kStillSamples;

    if (is_still) {
      vel_ = {0.0, 0.0, 0.0};
      last_accel_ = {0.0, 0.0, 0.0};
      accel_primed_ = false;
    } else {
      // prime the trapezoid so the first step has no artifact
      if (!accel_primed_) {
        last_accel_ = accel_world;
        accel_primed_ = true;
      }

      // 2. Integrate X and Y using Trapezoidal rule (no velocity damping!)
      for (int i = 0; i < 2; ++i) {
        // Only filter microscopic floating-point solver noise when nearly idle
        if (std::abs(accel_world[i]) < 1e-4) {
          accel_world[i] = 0.0;
        }

        const double next_vel = vel_[i] + 0.5 * (last_accel_[i] + accel_world[i]) * dt;
        pos_[i] += 0.5 * (vel_[i] + next_vel) * dt;
        vel_[i] = next_vel;
        last_accel_[i] = accel_world[i];
      }
    }

    vel_[2] = 0.0;
    pos_[2] = 0.0;

    Vec3 vel_body = rotate_inverse(msg->orientation, vel_);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = pos_[0];
    odom.pose.pose.position.y = pos_[1];
    odom.pose.pose.position.z = pos_[2];
    odom.pose.pose.orientation = msg->orientation;

    odom.twist.twist.linear.x = vel_body[0];
    odom.twist.twist.linear.y = vel_body[1];
    odom.twist.twist.linear.z = vel_body[2];
    odom.twist.twist.angular = msg->angular_velocity;

    publisher_->publish(odom);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";
    t.transform.translation.x = pos_[0];
    t.transform.translation.y = pos_[1];
    t.transform.translation.z = pos_[2];
    t.transform.rotation = msg->orientation;

    tf_broadcaster_->sendTransform(t);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // These state variables should remain in the world frame.
  Vec3 pos_{0.0, 0.0, 0.0};
  Vec3 vel_{0.0, 0.0, 0.0};
  Vec3 last_accel_{0.0, 0.0, 0.0};
  rclcpp::Time last_time_;
  bool has_last_time_ = false;

  //tuning const
  Vec3 bias_{0.0, 0.0, 0.0};
  Vec3 filter_{0.0, 0.0, 0.0};
  bool accel_primed_ = false;
  bool cmd_is_zero_ = true;
  int calib_count_ = 0;
  int still_count_ = 0;
  static constexpr int kCalibSamples = 200;    // starting samples
  static constexpr int kStillSamples = 20;     // quiet samples before declaring
  static constexpr double kLpfAlpha = 0.2;     // 1.0 = no filter, lower better
  static constexpr double kAccelStill = 0.05;  // m/s^2, tune from idle logs
  static constexpr double kGyroStill = 0.02;   // rad/s, tune from logs
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuOdom>());
  rclcpp::shutdown();

  return 0;
}
