#include "rclcpp/rclcpp.hpp"
#include "navigation_server/srv/plan_path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

#include <chrono>
#include <cmath>
#include <memory>

#include "costmap.hpp"
#include "costmap_params.hpp"
#include "astar.hpp"

static costmap::Params g_cm_params;
static astar::Params g_astar_params;

rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr path_planning_publisher;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_publisher;
rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_publisher;

static void world_to_grid(const nav_msgs::msg::OccupancyGrid &map, double x, double y, int &grid_x, int &grid_y)
{
    grid_x = static_cast<int>(std::floor((x - map.info.origin.position.x) / map.info.resolution));
    grid_y = static_cast<int>(std::floor((y - map.info.origin.position.y) / map.info.resolution));
}

static void grid_to_world(const nav_msgs::msg::OccupancyGrid &map, int x, int y, double &world_x, double &world_y)
{
    world_x = (x + 0.5) * map.info.resolution + map.info.origin.position.x;
    world_y = (y + 0.5) * map.info.resolution + map.info.origin.position.y;
}

static bool map_validation(const nav_msgs::msg::OccupancyGrid &map)
{
    return !(map.data.empty() ||
             map.data.size() != static_cast<size_t>(map.info.width) * static_cast<size_t>(map.info.height) ||
             map.info.resolution <= 0.0f);
}

static void publish_planning(bool value)
{
    std_msgs::msg::Bool msg;
    msg.data = value;
    path_planning_publisher->publish(msg);
}

void pathfind(const std::shared_ptr<navigation_server::srv::PlanPath::Request> request,
              std::shared_ptr<navigation_server::srv::PlanPath::Response> response)
{
    const auto logger = rclcpp::get_logger("navigation_server");
    publish_planning(true);

    // On any failure: publish EMPTY plan/waypoints so the controller stops instead of
    // continuing along a stale path that may now be blocked.
    auto fail = [&](const std::string &why) {
        RCLCPP_WARN(logger, "Planning failed: %s", why.c_str());
        nav_msgs::msg::Path empty;
        empty.header = request->map.header;
        plan_publisher->publish(empty);
        waypoints_publisher->publish(empty);
        publish_planning(false);
    };

    const auto &map = request->map;
    if (!map_validation(map)) { fail("invalid map"); return; }

    RCLCPP_INFO(logger, "Start: %f, %f Goal: %f, %f", request->start.x, request->start.y, request->goal.x, request->goal.y);

    int start_x, start_y, goal_x, goal_y;
    world_to_grid(map, request->start.x, request->start.y, start_x, start_y);
    world_to_grid(map, request->goal.x, request->goal.y, goal_x, goal_y);

    auto t0 = std::chrono::steady_clock::now();
    costmap::Costmap cm = costmap::build(map.data, static_cast<int>(map.info.width),
                                         static_cast<int>(map.info.height), map.info.resolution, g_cm_params);
    auto t1 = std::chrono::steady_clock::now();
    astar::Result result = astar::plan(cm, {start_x, start_y}, {goal_x, goal_y}, g_astar_params);
    auto t2 = std::chrono::steady_clock::now();

    RCLCPP_INFO(logger, "costmap: %.1f ms, A*: %.1f ms",
                std::chrono::duration<double, std::milli>(t1 - t0).count(),
                std::chrono::duration<double, std::milli>(t2 - t1).count());

    if (!result.found) { fail(result.message); return; }
    if (result.start_snapped) {
        RCLCPP_WARN(logger, "Robot is inside the clearance zone; planning from nearest free cell (%d, %d)",
                    result.start_used.x, result.start_used.y);
    }

    const auto &path = result.path;
    int unknown_cells = 0;
    for (const auto &c : path) if (cm.is_unknown(c.x, c.y)) unknown_cells++;
    RCLCPP_INFO(logger, "Path found: %zu cells (%d through unknown), %zu waypoints",
                path.size(), unknown_cells, result.waypoint_indices.size());

    nav_msgs::msg::Path plan;
    plan.header = map.header;
    for (size_t i = 0; i < path.size(); i++) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = plan.header;
        double wx, wy;
        grid_to_world(map, path[i].x, path[i].y, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;

        double yaw;
        if (path.size() == 1) {
            yaw = request->start.theta;
        } else if (i < path.size() - 1) {
            yaw = std::atan2(double(path[i + 1].y - path[i].y), double(path[i + 1].x - path[i].x));
        } else {
            yaw = request->goal.theta;
        }
        pose.pose.orientation.z = std::sin(yaw / 2.0);
        pose.pose.orientation.w = std::cos(yaw / 2.0);
        plan.poses.push_back(pose);
    }

    nav_msgs::msg::Path waypoints;
    waypoints.header = map.header;
    for (size_t idx : result.waypoint_indices) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = waypoints.header;
        double wx, wy;
        grid_to_world(map, path[idx].x, path[idx].y, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.orientation.w = 1.0;
        waypoints.poses.push_back(pose);
        RCLCPP_INFO(logger, "Waypoint: (%d, %d)", path[idx].x, path[idx].y);
    }

    plan_publisher->publish(plan);
    waypoints_publisher->publish(waypoints);
    response->plan = plan;
    response->waypoints = waypoints;

    publish_planning(false);
    RCLCPP_INFO(logger, "finished");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("navigation_server");

    g_cm_params = load_costmap_params(*node);
    g_astar_params.turn_cost = static_cast<float>(node->declare_parameter<double>("turn_cost", g_astar_params.turn_cost));
    g_astar_params.max_start_snap_cells = static_cast<int>(
        node->declare_parameter<int64_t>("max_start_snap_cells", g_astar_params.max_start_snap_cells));

    path_planning_publisher = node->create_publisher<std_msgs::msg::Bool>("/path_planning", 10);
    plan_publisher = node->create_publisher<nav_msgs::msg::Path>("/plan", 10);
    waypoints_publisher = node->create_publisher<nav_msgs::msg::Path>("/waypoints", 10);
    rclcpp::Service<navigation_server::srv::PlanPath>::SharedPtr service =
        node->create_service<navigation_server::srv::PlanPath>("plan_path", &pathfind);

    RCLCPP_INFO(node->get_logger(), "Ready to map");
    RCLCPP_INFO(node->get_logger(), "robot_radius %.2f + margin %.2f m, inflation %.2f m, occupied >= %d, unknown %s",
                g_cm_params.robot_radius, g_cm_params.safety_margin, g_cm_params.inflation_radius,
                g_cm_params.occupied_threshold, g_cm_params.allow_unknown ? "traversable" : "blocked");
    RCLCPP_INFO(node->get_logger(), "Turn cost: %f", g_astar_params.turn_cost);

    rclcpp::spin(node);
    path_planning_publisher.reset();
    plan_publisher.reset();
    waypoints_publisher.reset();
    node.reset();
    rclcpp::shutdown();
    return 0;
}
