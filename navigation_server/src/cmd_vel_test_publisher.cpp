#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <cmath>
#include <memory>
#include <random>

#include "tf2/LinearMath/Quaternion.h"

std::default_random_engine random_generator_;
std::normal_distribution<double> position_drift_{0.0, 0.01};
std::normal_distribution<double> yaw_drift_{0.0, 0.005};

const int PUBLISH_FREQUENCY = 10;  // Hz

class FakeRobot : public rclcpp::Node
{
public:
    FakeRobot() : Node("fake_robot")
    {
        // -------------------------------------------------
        // Starting robot state
        // -------------------------------------------------

        this->declare_parameter<double>("robot_x", 0.0);
        this->declare_parameter<double>("robot_y", 0.0);
        this->declare_parameter<double>("yaw_degrees", 0.0);

        robot_x_ = this->get_parameter("robot_x").as_double();
        robot_y_ = this->get_parameter("robot_y").as_double();

        double yaw_degrees =
            this->get_parameter("yaw_degrees").as_double();

        robot_yaw_ = yaw_degrees * M_PI / 180.0;

        // -------------------------------------------------
        // /odom publisher
        // -------------------------------------------------

        odom_publisher_ =
            this->create_publisher<nav_msgs::msg::Odometry>(
                "/odom",
                10);

        // -------------------------------------------------
        // /cmd_vel subscriber
        // -------------------------------------------------

        cmd_vel_subscriber_ =
            this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel",
                10,
                std::bind(
                    &FakeRobot::cmd_vel_callback,
                    this,
                    std::placeholders::_1));

        // -------------------------------------------------
        // Timer
        // -------------------------------------------------

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / PUBLISH_FREQUENCY),
            std::bind(
                &FakeRobot::update_and_publish,
                this));

        RCLCPP_INFO(
            this->get_logger(),
            "Fake robot started at x=%.2f, y=%.2f, yaw=%.2f degrees",
            robot_x_,
            robot_y_,
            yaw_degrees);
    }

private:
    // -------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr
        odom_publisher_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
        cmd_vel_subscriber_;

    rclcpp::TimerBase::SharedPtr timer_;

    // -------------------------------------------------
    // Robot state
    // -------------------------------------------------

    double robot_x_;
    double robot_y_;
    double robot_yaw_;

    double linear_velocity_ = 0.0;
    double angular_velocity_ = 0.0;

    // -------------------------------------------------
    // Receive /cmd_vel
    // -------------------------------------------------

    void cmd_vel_callback(
        const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        linear_velocity_ = msg->linear.x;
        angular_velocity_ = msg->angular.z;
    }

    // -------------------------------------------------
    // Update robot position and publish odometry
    // -------------------------------------------------

    void update_and_publish()
    {
        double dt = 1.0 / PUBLISH_FREQUENCY;

        // ---------------------------------------------
        // Update position
        // ---------------------------------------------

        robot_x_ +=
            linear_velocity_ *
            std::cos(robot_yaw_) *
            dt;

        robot_y_ +=
            linear_velocity_ *
            std::sin(robot_yaw_) *
            dt;

        // ---------------------------------------------
        // Update heading
        // ---------------------------------------------

        robot_yaw_ += angular_velocity_ * dt;

        // Keep yaw between -pi and pi
        while (robot_yaw_ > M_PI)
        {
            robot_yaw_ -= 2.0 * M_PI;
        }

        while (robot_yaw_ < -M_PI)
        {
            robot_yaw_ += 2.0 * M_PI;
        }

        robot_x_ += position_drift_(random_generator_);
        robot_y_ += position_drift_(random_generator_);
        robot_yaw_ += yaw_drift_(random_generator_);

        // ---------------------------------------------
        // Create odometry message
        // ---------------------------------------------

        nav_msgs::msg::Odometry odom;

        odom.header.stamp = this->get_clock()->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = robot_x_;
        odom.pose.pose.position.y = robot_y_;
        odom.pose.pose.position.z = 0.0;

        // ---------------------------------------------
        // Convert yaw to quaternion
        // ---------------------------------------------

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, robot_yaw_);

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // ---------------------------------------------
        // Publish odometry
        // ---------------------------------------------

        odom_publisher_->publish(odom);

        // ---------------------------------------------
        // Debug output
        // ---------------------------------------------

        RCLCPP_INFO(
            this->get_logger(),
            "Robot: x=%.2f y=%.2f yaw=%.2f | "
            "cmd_vel: v=%.2f w=%.2f",
            robot_x_,
            robot_y_,
            robot_yaw_,
            linear_velocity_,
            angular_velocity_);
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<FakeRobot>();

    rclcpp::spin(node);

    node.reset();

    rclcpp::shutdown();

    return 0;
}