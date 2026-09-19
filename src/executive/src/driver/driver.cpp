#include "executive/driver.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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

    join_state_publiisher_ = this->create_publisher< sensor_msgs::msg::JointState >(
		    "joint_states", 1);

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
    
    double vx = msg-> linear.x; 
    double vy = msg->linear.y; 
    double w = msg->angular.z; 

    double X = 1;
    double Y = 2;

    // calculation for front Left (X,Y)
    double fl_vx = vx - (w * Y);
    double fl_vy = vy + (w * X);


    //Front right (X, -Y)
    double fr_vx = vx + (w * Y); 
    double fr_vy = vy + (w *X);

    //Back left (-X, Y)
    double bl_vx = vx - (w * Y);
    double bl_vy = vy - (w * X); 

    //Back right (-X, -Y)
    double br_vx = vx + (w * Y); 
    double br_vy = vy - (w * X);

    //converting the vectors to Drive Speed (hypotenuse) and Steering angle (atan2)
    double fl_speed = std::hypot(fl_vx, fl_vy); 
    double fl_angle = std::atan2(fl_vy, fl_vx);

    double fr_speed = std::hypot(fr_vx, fr_vy); 
    double fr_angle = std::atan2(fr_vy, fr_vx);
    

    double bl_speed = std::hypot(bl_vx, bl_vy); 
    double bl_angle = std::atan2(bl_vy, bl_vx);
    
    double br_speed = std::hypot(br_vx, br_vy); 
    double br_angle = std::atan2(br_vy, br_vx);

    RCLCPP_INFO(this->get_logger(),
    "\nFL: [%.2f m/s, %.2f rad] | FR: [%.2f m/s, %.2f rad]\nRL: [%.2f m/s, %.2f rad] | RR: [%.2f m/s, %.2f rad]",
    fl_speed, fl_angle, fr_speed, fr_angle, bl_speed, bl_angle, br_speed, br_angle);
}

}
