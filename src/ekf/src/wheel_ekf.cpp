#include <algorithm>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ekf/ekf.hpp"

// Wheel EKF: predicts the robot's location (x, y, theta) from the wheel
// speeds. Prediction only (Table 7.2 lines 2 to 7): the wheels give no
// absolute position, so the uncertainty keeps growing.
//
//   Input : /wheel/odom      (nav_msgs/Odometry, speeds in twist)
//   Output: /odometry/wheel  (nav_msgs/Odometry, location + uncertainty)
//   TF    : none (pose_fusion publishes odom to base_link)

class WheelEkf : public rclcpp::Node
{
public:
  WheelEkf() : Node("wheel_ekf")
  {
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/wheel/odom", 10,
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) { on_wheel(*msg); });

    pub_ = create_publisher<nav_msgs::msg::Odometry>("/odometry/wheel", 10);

    // Start at (0, 0, 0), fairly sure about it (1e-4)
    ekf_.Sigma = Ekf2D::Mat3::Identity() * 1e-4;
  }

private:
  void on_wheel(const nav_msgs::msg::Odometry & msg)
  {
    rclcpp::Time t(msg.header.stamp, RCL_ROS_TIME);

    if (!has_last_) {
      last_ = t;
      has_last_ = true;
      return;                          // need two messages to get dt
    }
    const double dt = (t - last_).seconds();
    last_ = t;
    if (dt <= 0.0 || dt > 0.5) {
      return;                          // skip broken or huge gaps
    }

    // ===== Controls u_t = (v_t, w_t) =====
    const double v = msg.twist.twist.linear.x;    // forward speed (m/s)
    const double w = msg.twist.twist.angular.z;   // turn rate (rad/s)

    // ===== Line 2: theta = mu_{t-1, theta} =====
    const double th = ekf_.mu(2);
    const double c = std::cos(th);
    const double s = std::sin(th);

    // ===== Line 3: G_t = d(new state) / d(old state) =====
    //   [ 1  0  -v dt sin(theta) ]
    //   [ 0  1   v dt cos(theta) ]
    //   [ 0  0   1               ]
    Ekf2D::Mat3 G = Ekf2D::Mat3::Identity();
    G(0, 2) = -v * dt * s;
    G(1, 2) =  v * dt * c;

    // ===== Line 4: V_t = d(new state) / d(controls v, w) =====
    //   [ dt cos(theta)   0  ]
    //   [ dt sin(theta)   0  ]
    //   [ 0               dt ]
    Eigen::Matrix<double, 3, 2> V;
    V << dt * c, 0.0,
         dt * s, 0.0,
         0.0,    dt;

    // ===== Line 5: M_t = how noisy the controls are =====
    // From the wheel message's twist covariance:
    //   index 0  = variance of v (linear.x)
    //   index 35 = variance of w (angular.z)
    // A small minimum keeps it from ever being exactly zero.
    const auto & cov = msg.twist.covariance;
    Eigen::Matrix2d M = Eigen::Matrix2d::Zero();
    M(0, 0) = std::max(cov[0],  1e-4);
    M(1, 1) = std::max(cov[35], 1e-4);

    // ===== Line 6: the change in state (straight step) =====
    Ekf2D::Vec3 motion;
    motion << v * dt * c,
              v * dt * s,
              w * dt;

    // ===== Lines 6 and 7: predict =====
    //   mu_bar    = mu + motion
    //   Sigma_bar = G Sigma G^T + V M V^T
    ekf_.predict(motion, G, V * M * V.transpose());

    publish(t, msg);
  }

  void publish(const rclcpp::Time & t, const nav_msgs::msg::Odometry & in)
  {
    nav_msgs::msg::Odometry out;
    out.header.stamp = t;
    out.header.frame_id = "odom";      // label only, does not publish TF
    out.child_frame_id = "base_link";

    // Location = the EKF's best guess
    out.pose.pose.position.x = ekf_.mu(0);
    out.pose.pose.position.y = ekf_.mu(1);
    out.pose.pose.orientation.z = std::sin(ekf_.mu(2) / 2.0);   // yaw to quaternion
    out.pose.pose.orientation.w = std::cos(ekf_.mu(2) / 2.0);

    // Uncertainty: Sigma (x, y, theta) into ROS's 6x6 grid
    // (6x6 order: x, y, z, roll, pitch, yaw; so x=0, y=1, yaw=5)
    const int idx[3] = {0, 1, 5};
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        out.pose.covariance[idx[i] * 6 + idx[j]] = ekf_.Sigma(i, j);
      }
    }
    out.pose.covariance[2 * 6 + 2] = 1e6;   // z, roll, pitch: not estimated
    out.pose.covariance[3 * 6 + 3] = 1e6;
    out.pose.covariance[4 * 6 + 4] = 1e6;

    // Speeds: pass the wheel speeds through
    out.twist = in.twist;

    pub_->publish(out);
  }

  Ekf2D ekf_;
  rclcpp::Time last_;
  bool has_last_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelEkf>());
  rclcpp::shutdown();
  return 0;
}