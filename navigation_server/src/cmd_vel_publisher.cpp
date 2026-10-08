#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <chrono>
#include <limits>
#include <cmath>

const int CONTROL_FREQUENCY = 10;  // Hz
const int DISTANCE_INCREASE_LIMIT = 10;
const int LOOKAHEAD_POINTS = 5;
const float TURN_THRESHOLD = 0.2;  // radians
const float TURN_SPEED = 0.5;  // rad/s
const float DRIVE_SPEED = 5; // m/s
const float MAX_SPEED = 5; // m/s
const float MAX_ACCELERATION = 1.0; // m/s^2
const float MAX_DECELERATION = 1.5; // m/s^2
const float MIN_START_SPEED = 0.5; // m/s
const float WAYPOINT_DISTANCE_THRESHOLD = 1; // meters

class CmdVelPublisher : public rclcpp::Node
{
enum class ControllerState
{
    DRIVING,
    TURNING,
    GOAL_REACHED
};
public:
    CmdVelPublisher() : Node("cmd_vel_publisher")
    {
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&CmdVelPublisher::odom_callback, this, std::placeholders::_1));
        path_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&CmdVelPublisher::path_callback, this, std::placeholders::_1));
        waypoint_subscriber_ = this->create_subscription<nav_msgs::msg::Path>("/waypoints", 10, std::bind(&CmdVelPublisher::waypoint_callback, this, std::placeholders::_1));
        path_planning_subscriber_ = this->create_subscription<std_msgs::msg::Bool>("/path_planning", 10, std::bind(&CmdVelPublisher::path_planning_callback, this, std::placeholders::_1));
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / CONTROL_FREQUENCY), std::bind(&CmdVelPublisher::control_loop, this));
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr path_planning_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Path current_path_;
    nav_msgs::msg::Path current_waypoints_;

    bool path_planning_ = false;
    bool have_odom_ = false;
    bool have_path_ = false;
    bool have_waypoints_ = false;
    bool have_path_planning_ = false;
    bool goal_reached_ = false;
    int previous_closest_index_ = 0;
    size_t current_waypoint_index_ = 1;
    float target_heading_ = 0.0;
    float current_speed_ = 0.0;
    ControllerState state_ = ControllerState::DRIVING;

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        have_odom_ = true;
    }
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        current_path_ = *msg;
        have_path_ = true;
        previous_closest_index_ = 0;
    }
    void waypoint_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        current_waypoints_ = *msg;
        have_waypoints_ = true;
        current_waypoint_index_ = 1;
        goal_reached_ = false;
        state_ = ControllerState::DRIVING;
        RCLCPP_INFO(this->get_logger(),"recieved %zu waypoints", msg->poses.size());
    }
    void path_planning_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        path_planning_ = msg->data;
        have_path_planning_ = true;
    }
    void control_loop()
    {
        float closest_distance = std::numeric_limits<float>::max();
        int closest_index = -1;
        int increasing_count = 0;
        
        if (path_planning_ || 
            !have_odom_ || 
            !have_path_ || 
            !have_path_planning_ || 
            !have_waypoints_ || 
            current_waypoints_.poses.empty() || 
            current_path_.poses.empty() || 
            current_waypoints_.poses.size() < 2){
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_publisher_->publish(cmd_vel);
            return;
        }

        if (state_ == ControllerState::GOAL_REACHED)
        {
            geometry_msgs::msg::Twist cmd_vel;

            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;

            cmd_vel_publisher_->publish(cmd_vel);

            return;
        }

        float robot_x = current_odom_.pose.pose.position.x;
        float robot_y = current_odom_.pose.pose.position.y;
        float waypoint_x = current_waypoints_.poses[current_waypoint_index_].pose.position.x;
        float waypoint_y = current_waypoints_.poses[current_waypoint_index_].pose.position.y;
        
        float waypoint_distance = std::sqrt(std::pow(robot_x-waypoint_x,2)+std::pow(robot_y-waypoint_y,2));

        float previous_waypoint_x = current_waypoints_.poses[current_waypoint_index_ - 1].pose.position.x;

        float previous_waypoint_y = current_waypoints_.poses[current_waypoint_index_ - 1].pose.position.y;

        float direction_x = waypoint_x - previous_waypoint_x;
        float direction_y = waypoint_y - previous_waypoint_y;

        float robot_relative_x = robot_x - waypoint_x;
        float robot_relative_y = robot_y - waypoint_y;

        float passed_distance = robot_relative_x * direction_x + robot_relative_y * direction_y;

        float robot_from_previous_x = robot_x - previous_waypoint_x;
        float robot_from_previous_y = robot_y - previous_waypoint_y;
        float segment_distance = std::sqrt(std::pow(direction_x,2)+std::pow(direction_y,2));
        float projection = robot_from_previous_x * direction_x + robot_from_previous_y * direction_y;
        float distance_from_previous = projection / segment_distance;
        float progress = distance_from_previous / segment_distance;
        progress = std::max(0.0f, std::min(1.0f, progress));

        float desired_speed;

        if (progress < 0.25)
        {
            desired_speed = MIN_START_SPEED + MAX_SPEED * (progress / 0.25);
        }
        else if (progress < 0.75)
        {
            desired_speed = DRIVE_SPEED;
        }
        else
        {
            desired_speed = MAX_SPEED * ((1.0 - progress) / 0.25);
        }

        float dt = 1.0 / CONTROL_FREQUENCY;

        if (current_speed_ < desired_speed)
        {
            float speed_change_a = MAX_ACCELERATION * dt;
            if (speed_change_a < desired_speed - current_speed_) {
                current_speed_ += speed_change_a;
            } else {
                current_speed_ = desired_speed;
            }
        }
        else if (current_speed_ > desired_speed)
        {
            float speed_change_d = MAX_DECELERATION * dt;
            if (speed_change_d < current_speed_ - desired_speed) {
                current_speed_ -= speed_change_d;
            } else {
                current_speed_ = desired_speed;
            }
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Waypoint: %zu | Distance: %.2f",
            current_waypoint_index_,
            waypoint_distance);

        if (waypoint_distance <= WAYPOINT_DISTANCE_THRESHOLD || passed_distance >= 0) {
            if (current_waypoint_index_ == current_waypoints_.poses.size() - 1)
            {
                goal_reached_ = true;
                state_ = ControllerState::GOAL_REACHED;
                geometry_msgs::msg::Twist cmd_vel;

                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;

                cmd_vel_publisher_->publish(cmd_vel);

                RCLCPP_INFO(this->get_logger(), "Final waypoint reached. Stopping.");

                return;
            }
            if (current_waypoint_index_ < current_waypoints_.poses.size() - 1) {

                float current_waypoint_x = current_waypoints_.poses[current_waypoint_index_].pose.position.x;
                float current_waypoint_y = current_waypoints_.poses[current_waypoint_index_].pose.position.y;

                float next_waypoint_x = current_waypoints_.poses[current_waypoint_index_ + 1].pose.position.x;
                float next_waypoint_y = current_waypoints_.poses[current_waypoint_index_ + 1].pose.position.y;

                target_heading_ = std::atan2(next_waypoint_y - current_waypoint_y, next_waypoint_x - current_waypoint_x);

                state_ = ControllerState::TURNING;

                RCLCPP_INFO(this->get_logger(), "reached waypoint moving to waypoint %zu", current_waypoint_index_);
                RCLCPP_INFO(this->get_logger(), "Turning toward heading: %.2f rad (%.1f degrees)", target_heading_, target_heading_ * 180.0 / M_PI);
            }
        }

        for (size_t i = previous_closest_index_; i < current_path_.poses.size(); i++)
        {
            float path_x = current_path_.poses[i].pose.position.x;
            float path_y = current_path_.poses[i].pose.position.y;
            float indexed_distance = std::sqrt(std::pow((robot_x - path_x),2) + std::pow((robot_y - path_y),2));
            if (indexed_distance < closest_distance) {
                closest_distance = indexed_distance;
                closest_index = i;
                increasing_count = 0;
            } else if (indexed_distance > closest_distance) {
                increasing_count++;
            }
            if (increasing_count >= DISTANCE_INCREASE_LIMIT) {
                break;
            }
                
        }

        previous_closest_index_ = closest_index;

        int lookahead_index = closest_index + LOOKAHEAD_POINTS;
        if (lookahead_index >= current_path_.poses.size()) {
            lookahead_index = current_path_.poses.size() - 1;
        }

        float target_x = current_path_.poses[lookahead_index].pose.position.x;
        float target_y = current_path_.poses[lookahead_index].pose.position.y;

        float lookahead_from_previous_x =
            target_x - previous_waypoint_x;

        float lookahead_from_previous_y =
            target_y - previous_waypoint_y;

        float lookahead_projection =
            lookahead_from_previous_x * direction_x +
            lookahead_from_previous_y * direction_y;

        float segment_length_squared =
            direction_x * direction_x +
            direction_y * direction_y;


        if (lookahead_projection > segment_length_squared)
            {
                target_x = waypoint_x;
                target_y = waypoint_y;
            }

        float target_angle = std::atan2(target_y - robot_y, target_x - robot_x);

        

        auto orientation = current_odom_.pose.pose.orientation;

        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);

        double roll;
        double pitch;
        double yaw;

        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        float angle_error = target_angle - yaw;
        while (angle_error > M_PI) {
            angle_error -= 2 * M_PI;
        }

        while (angle_error < -M_PI) {
            angle_error += 2 * M_PI;
        }
        //RCLCPP_INFO(this->get_logger(), "Closest: %d | Lookahead: %d | Target angle: %.2f | Yaw: %.2f | Angle error: %.2f", closest_index, lookahead_index, target_angle, yaw, angle_error);
    
        geometry_msgs::msg::Twist cmd_vel;

        if (state_ == ControllerState::TURNING)
        {
            float angle_error = target_heading_ - yaw;

            current_speed_ = 0.0;

            while (angle_error > M_PI)
                angle_error -= 2 * M_PI;

            while (angle_error < -M_PI)
                angle_error += 2 * M_PI;

            geometry_msgs::msg::Twist cmd_vel;

            if (std::abs(angle_error) <= TURN_THRESHOLD)
            {
                
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = 0.0;

                state_ = ControllerState::DRIVING;
                current_waypoint_index_++;

                RCLCPP_INFO(
                    this->get_logger(),
                    "Turn complete. Driving toward waypoint %zu.",
                    current_waypoint_index_);
            }
            else
            {
                cmd_vel.linear.x = 0.0;

                if (angle_error > 0)
                    cmd_vel.angular.z = TURN_SPEED;
                else
                    cmd_vel.angular.z = -TURN_SPEED;
            }

            cmd_vel_publisher_->publish(cmd_vel);

            return;
        }

        if (std::abs(angle_error) > TURN_THRESHOLD) {
            cmd_vel.linear.x = 0.0;

            if (angle_error > 0) {
                cmd_vel.angular.z = TURN_SPEED;
            } else {
                cmd_vel.angular.z = -TURN_SPEED;
            }
        } else {
            cmd_vel.linear.x = current_speed_;
            cmd_vel.angular.z = 0.0;
        }
        cmd_vel_publisher_->publish(cmd_vel);

    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CmdVelPublisher>();

    RCLCPP_INFO(rclcpp::get_logger("cmd_vel_publisher"), "Ready to cmd_vel");

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}