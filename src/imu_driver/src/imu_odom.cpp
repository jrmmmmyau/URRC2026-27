#include <array>
#include <memory>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
      return;  // skip nonsense or huge gaps
    }

    const Vec3 accel_body = {
      msg->linear_acceleration.x,
      msg->linear_acceleration.y,
      msg->linear_acceleration.z};
    Vec3 accel_world = rotate(msg->orientation, accel_body);

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
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // These state variables should remain in the world frame.
  Vec3 pos_{0.0, 0.0, 0.0};
  Vec3 vel_{0.0, 0.0, 0.0};
  Vec3 last_accel_{0.0, 0.0, 0.0};
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
