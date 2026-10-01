#include <array>
#include <memory>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using Vec3 = std::array<double, 3>;

// TODO 1: Decide how you want to transform a vector between the IMU/body
// frame and the world frame.
static Vec3 rotate(const geometry_msgs::msg::Quaternion & q, const Vec3 & v)
{
  const double tx = 2.0 * (q.y * v[2] - q.z * v[1]);
  const double ty = 2.0 * (q.z * v[0] - q.x * v[2]);
  const double tz = 2.0 * (q.x * v[1] - q.y * v[0]);
  // TODO: return the rotated vector.
  return {v[0] + q.w * tx + (q.y * tz - q.z * ty),
    v[1] + q.w * ty + (q.z * tx - q.x * tz),
    v[2] + q.w * tz + (q.x * ty - q.y * tx)};
}

// TODO 2: Provide the reverse transformation.  From world to robot
static Vec3 rotate_inverse(geometry_msgs::msg::Quaternion q, const Vec3 & v)
{
  q.x = -q.x;
  q.y = -q.y;
  q.z = -q.z;
  // TODO: return the inverse-transformed vector.
  return rotate(q,v);
}

class ImuOdom : public rclcpp::Node
{
public:
  ImuOdom() : Node("imu_odom")
  {
    // TODO 3: Choose the input topic and subscription settings for the IMU.
    // Connect incoming messages to on_imu().

    // TODO 4: odometry output topic and publisher settings.
  }

private:
  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // TODO 5: Establish a trustworthy time step between measurements.  Think
    // about message timestamps, simulation time, startup, dropped messages,
    // delayed messages, and unreasonable gaps.

    // TODO 6: Verify IMU data

    // TODO 7: Convert the acceleration into the frame used for integration.
    // Identify the source and destination frames before writing the math.

    // TODO 8: Account for gravity according to the IMU driver's convention
    // and your chosen world frame.  But might want to wait for a spec from IMU

    // TODO 9: Choose an integration method and maintain whatever state it
    // needs.  Consider numerical error, uncertainty propagation, drifting

    // TODO 10: Fill and publish an Odometry message.  Remember to fill out everything
  
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  // These state variables should remain in the world frame.
  Vec3 pos_{0.0, 0.0, 0.0};
  Vec3 vel_{0.0, 0.0, 0.0};
  rclcpp::Time last_time_;
  bool has_last_time_ = false;
};

int main(int argc, char ** argv)
{
  // TODO 11: Complete the ROS 2 program lifecycle.

  return 0;
}
