#include "executive/driver.hpp"
#include <cmath>

namespace executive
{
executive::Driver::Driver()
    : Node("driver")
{

    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10, 
        std::bind(
            &Driver::cmd_vel_callback,
            this,
            std::placeholders::_1));

    drive_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/drive_cmd_vel",
        10);

    RCLCPP_INFO(this->get_logger(), "Driver node started");
}


void Driver::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    geometry_msgs::msg::Twist drive_command;

    drive_command.linear.x = msg->linear.x;
    drive_command.linear.y = msg->linear.y;
    drive_command.linear.z = msg->linear.z;

    drive_command.angular.x = msg->angular.x;
    drive_command.angular.y = msg->angular.y;
    drive_command.angular.z = msg->angular.z;

    drive_publisher_->publish(drive_command);
}

}
