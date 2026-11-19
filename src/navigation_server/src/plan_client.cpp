#include "rclcpp/rclcpp.hpp"
#include "navigation_server/srv/plan_path.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class PlanClient : public rclcpp::Node
{
public:
  PlanClient(double start_x, double start_y, double goal_x, double goal_y)
  : Node("plan_client"), start_x_(start_x), start_y_(start_y), goal_x_(goal_x), goal_y_(goal_y)
  {
    // Subscribe to the map
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {map_callback(msg);});

    // Service client
    client_ = create_client<navigation_server::srv::PlanPath>("/plan_path");

    // Publish the resulting path
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("/plan", 10);

    // Publish start and goal
    start_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/start", 10);
    goal_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal", 10);

    timer_ = create_wall_timer(1s, [this]() {send_request();});
  }

private:
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    map_ = msg;
  }

  void send_request()
  {
    if (request_sent_) {
      return;
    }

    if (!map_) {
      RCLCPP_INFO(get_logger(), "Waiting for map...");
      return;
    }

    if (!client_->wait_for_service(0s)) {
      RCLCPP_INFO(get_logger(), "Waiting for /plan_path...");
      return;
    }

    // Create the service request
    auto request = std::make_shared<navigation_server::srv::PlanPath::Request>();

    // Start
    request->start = geometry_msgs::msg::Pose2D();
    request->start.x = start_x_;
    request->start.y = start_y_;
    request->start.theta = 0.0;

    // Goal
    request->goal = geometry_msgs::msg::Pose2D();
    request->goal.x = goal_x_;
    request->goal.y = goal_y_;
    request->goal.theta = 0.0;

    // Send the map directly to A*
    request->map = *map_;

    RCLCPP_INFO(get_logger(), "Start: (%f, %f)", start_x_, start_y_);
    RCLCPP_INFO(get_logger(), "Goal: (%f, %f)", goal_x_, goal_y_);
    RCLCPP_INFO(get_logger(), "Sending map to A*...");

    auto future = client_->async_send_request(
      request,
      [this](rclcpp::Client<navigation_server::srv::PlanPath>::SharedFuture result) {
        response_callback(result);
      });
    (void)future;
    request_sent_ = true;
  }

  void response_callback(
    rclcpp::Client<navigation_server::srv::PlanPath>::SharedFuture future)
  {
    try {
      auto response = future.get();
      const auto number_of_poses = response->plan.poses.size();
      RCLCPP_INFO(get_logger(), "Received path with %zu poses", number_of_poses);

      if (number_of_poses == 0) {
        RCLCPP_WARN(get_logger(), "A* did not find a path.");
        return;
      }

      // Publish path
      path_publisher_->publish(response->plan);

      // Publish start
      geometry_msgs::msg::PoseStamped start;
      start.header = response->plan.header;
      start.pose.position.x = start_x_;
      start.pose.position.y = start_y_;
      start.pose.orientation.w = 1.0;
      start_publisher_->publish(start);

      // Publish goal
      geometry_msgs::msg::PoseStamped goal;
      goal.header = response->plan.header;
      goal.pose.position.x = goal_x_;
      goal.pose.position.y = goal_y_;
      goal.pose.orientation.w = 1.0;
      goal_publisher_->publish(goal);

      RCLCPP_INFO(get_logger(), "Published path, start, and goal.");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Service call failed: %s", e.what());
    }
  }

  double start_x_;
  double start_y_;
  double goal_x_;
  double goal_y_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  bool request_sent_ = false;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Client<navigation_server::srv::PlanPath>::SharedPtr client_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr start_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Require four coordinates
  if (argc != 5) {
    std::cout << "Usage: python3 plan_client.py START_X START_Y GOAL_X GOAL_Y" << std::endl;
    return 0;
  }

  const double start_x = std::stod(argv[1]);
  const double start_y = std::stod(argv[2]);
  const double goal_x = std::stod(argv[3]);
  const double goal_y = std::stod(argv[4]);

  auto node = std::make_shared<PlanClient>(start_x, start_y, goal_x, goal_y);
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
