#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class WheelOdom : public rclcpp::Node
{
public:
  WheelOdom()
  : Node("wheel_odom")
    {

        fl_wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel/fl_wheel", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { fl_wheel = msg->data; });

        fr_wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel/fr_wheel", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { fr_wheel = msg->data; });

        rl_wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel/rl_wheel", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { rl_wheel = msg->data; });

        rr_wheel_sub_ = create_subscription<std_msgs::msg::Float64>(
        "/cmd_vel/rr_wheel", 10,
        [this](const std_msgs::msg::Float64::SharedPtr msg) { rr_wheel = msg->data; });


        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&compute_vel, this, std::placeholders::_1));

        twist_pub_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
          "/wheel_odom/twist", 10);
    }

private:
    // FL FR RL RR
    float x[4]={0.32, 0.32, -0.32, -0.32};
    float y[4]={0.24, -0.24, 0.24, -0.24};
    float r=0.09; //radius in m


  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fl_wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fr_wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rl_wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rr_wheel_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_pub_;
  float fl_steering=0;
  float fr_steering=0;
  float rl_steering=0;
  float rr_steering=0;
  float fl_wheel=0;
  float fr_wheel=0;
  float rl_wheel=0;
  float rr_wheel=0;

void compute_vel(sensor_msgs::msg::JointState::SharedPtr msg){
    // FL FR RL RR
    const char *steer_names[] = {
    "front_left_steering_joint", "front_right_steering_joint",
    "rear_left_steering_joint", "rear_right_steering_joint"};
  double *angles[4] = {&fl_steering, &fr_steering, &rl_steering, &rr_steering};
  double speeds[4] = {fl_wheel, fr_wheel, rl_wheel, rr_wheel};
  for (size_t i = 0; i < msg->name.size(); ++i) {
      for (size_t wheel = 0; wheel < 4; ++wheel) {
          if (msg->name[i] == steer_names[wheel]) {
              if (i < msg->position.size() && std::isfinite(msg->position[i])) {
              *angles[wheel] = msg->position[i];
              }
          }
      }
  }

  //vel of each wheel
  double vx = 0, vy = 0, ang_vel = 0;
  for(int i=0; i<4;i++){
    double u=speeds[i]*r*std::cos(*angles[i]);
    double w=speeds[i]*r*std::sin(*angles[i]);
    vx+=u;
    vy+=w;
    ang_vel+=(x[i]*w-y[i]*u);
  }

  vx/=4;
  vy/=4;
  ang_vel/=0.64;//this is sum of x^2+y^2
    // uncertainty grows with speed (regolith slip)
  double speed = std::hypot(vx, vy);
  double sv = 0.05 + 0.3 * speed;            // m/s
  double sw = 0.1 + 0.3 * std::abs(ang_vel); // rad/s

  geometry_msgs::msg::TwistWithCovarianceStamped out;
  out.header.stamp = this->now();
  out.header.frame_id = "base_link";
  out.twist.twist.linear.x = vx;
  out.twist.twist.linear.y = vy;
  out.twist.twist.angular.z = ang_vel;

  out.twist.covariance[0]  = sv * sv;   // vx
  out.twist.covariance[7]  = sv * sv;   // vy
  out.twist.covariance[14] = 1e6;       // vz, not measured
  out.twist.covariance[21] = 1e6;       // roll rate, not measured
  out.twist.covariance[28] = 1e6;       // pitch rate, not measured
  out.twist.covariance[35] = sw * sw;   // wz

  twist_pub_->publish(out);
}

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}