#pragma once
#include <vector>
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace executive {

class Driver : public rclcpp::Node {
public:
  Driver();

private:
    void cmd_vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg);

    void joint_state_callback(
        const sensor_msgs::msg::JointState::SharedPtr msg);

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_subscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        drive_publisher_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
	    joint_state_publisher_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
        joint_state_subscriber_;

    double current_fl_angle_ = 0.0;
    double current_fr_angle_ = 0.0;
    double current_bl_angle_ = 0.0;
    double current_br_angle_ = 0.0;

   
};
}
