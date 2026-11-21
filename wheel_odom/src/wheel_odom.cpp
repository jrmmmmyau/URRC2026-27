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
    }

private:
    // FL FR RL RR
    float x[4]={0.32, 0.32, -0.32, -0.32};
    float y[4]={0.24, -0.24, 0.24, -0.24};


  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fl_wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fr_wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rl_wheel_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr rr_wheel_sub_;
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
  double *angles[] = {&fl_steering, &fr_steering, &rl_steering, &rr_steering};
  double *speeds[4];
  for (size_t i = 0; i < msg->name.size(); ++i) {
      for (size_t wheel = 0; wheel < 4; ++wheel) {
          if (msg->name[i] == steer_names[wheel]) {
              if (i < msg->velocity.size() && std::isfinite(msg->velocity[i])) {
              *angles[wheel] = msg->position[i];
              }
          }
      }
  }

  float u[4];//x velocity
  float w[4];//y velocity
  for(int i=0; i<4;i++){
    

  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}