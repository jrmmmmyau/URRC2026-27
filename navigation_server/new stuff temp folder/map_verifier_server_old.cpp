#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <cmath>
#include <limits>
#include <utility>

#include "navigation_server/srv/plan_path.hpp"

const float MAP_CHANGE_THRESHOLD = 1000.0f;
const float ROBOT_RADIUS = 0.5f;
const float ROBOT_BERTH = 0.5f;
const float EFFECTIVE_RADIUS = ROBOT_RADIUS + ROBOT_BERTH;
const float GOAL_TOLERANCE = 0.001f;

class MapVerifierServer : public rclcpp::Node
{
public:
    MapVerifierServer()
        : Node("map_verifier_server")
    {
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map",10, std::bind(&MapVerifierServer::map_callback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("/goal", 10, std::bind(&MapVerifierServer::goal_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MapVerifierServer::odom_callback, this, std::placeholders::_1));

        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&MapVerifierServer::path_callback, this, std::placeholders::_1));

        // A* service client
        plan_client_ = this->create_client<navigation_server::srv::PlanPath>("/plan_path"); RCLCPP_INFO(this->get_logger(), "Map verifier server started");
    }

private:

    void map_callback(
        const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = *msg;
        have_map_ = true;
        RCLCPP_INFO(this->get_logger(), "Received map");
        

        if (!have_last_verified_map_)
        {
            last_verified_map_ = current_map_;
            have_last_verified_map_ = true;

            if (have_goal_ && have_odom_)
            {
                request_new_plan();
            }

            return;
        }

        float change = calculate_map_change(last_verified_map_, current_map_);
        if (change < MAP_CHANGE_THRESHOLD) {
            return;
        }

        if (map_change_affects_path()) {
            request_new_plan();
        } else {
            last_verified_map_ = current_map_;
        }

    }

    void goal_callback(
        const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        current_goal_ = *msg;
        have_goal_ = true;
        RCLCPP_INFO(this->get_logger(), "Received goal");

        if (!have_last_planned_goal_) {
            if (have_map_ && have_odom_) {
                request_new_plan();
            }

            return;
        }

        if (goal_changed(last_planned_goal_, current_goal_))
        {
            request_new_plan();
        }
    }

    void odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        have_odom_ = true;
    }

    void path_callback(
        const nav_msgs::msg::Path::SharedPtr msg)
    {
        current_path_ = *msg;
        have_path_ = true;
    }

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    // A* service client
    rclcpp::Client<navigation_server::srv::PlanPath>::SharedPtr plan_client_;

    // Stored data
    nav_msgs::msg::OccupancyGrid current_map_;
    nav_msgs::msg::OccupancyGrid last_verified_map_;
    geometry_msgs::msg::Pose2D current_goal_;
    geometry_msgs::msg::Pose2D last_planned_goal_;
    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Path current_path_;

    // Data availability
    bool have_map_ = false;
    bool have_last_verified_map_ = false;
    bool have_goal_ = false;
    bool have_last_planned_goal_ = false;
    bool have_odom_ = false;
    bool have_path_ = false;

    std::pair<int, int> world_to_grid(float world_x, float world_y, const nav_msgs::msg::OccupancyGrid &map) {
    int grid_x = static_cast<int>(std::floor((world_x - map.info.origin.position.x) / map.info.resolution));
    int grid_y = static_cast<int>(std::floor((world_y - map.info.origin.position.y) / map.info.resolution));
    return {grid_x, grid_y};
    }

    bool map_change_affects_path() {
        if (!have_path_) {
            return true;
        }
        for (const auto &pose : current_path_.poses) {
            auto [grid_x, grid_y] = world_to_grid(pose.pose.position.x, pose.pose.position.y, current_map_);
            int radius_cells = static_cast<int>(std::ceil(EFFECTIVE_RADIUS / current_map_.info.resolution));
            for (int dx = -radius_cells; dx <= radius_cells; dx++) {
                for (int dy = -radius_cells; dy <= radius_cells; dy++) {
                    if (dx * dx + dy * dy > radius_cells * radius_cells) {
                            continue;
                        }
                    int check_x = grid_x + dx;
                    int check_y = grid_y + dy;
                    if (check_x < 0 || check_x >= static_cast<int>(current_map_.info.width) || check_y < 0 || check_y >= static_cast<int>(current_map_.info.height)) {
                        return true;
                    }
                    size_t index = check_x + current_map_.info.width * check_y;
                    if (current_map_.data[index] != 0) {
                        continue;
                    }
                    return true;
                }
            }
        }
        return false;
    }
    
    float calculate_map_change(const nav_msgs::msg::OccupancyGrid &old_map, const nav_msgs::msg::OccupancyGrid &new_map) {
        if (old_map.data.size() != new_map.data.size()) {
            return std::numeric_limits<float>::infinity();
        }
        float change = 0.0f;
        for (size_t i = 0; i < old_map.data.size(); i++) {
            int old_value = static_cast<unsigned char>(old_map.data[i]);
            int new_value = static_cast<unsigned char>(new_map.data[i]);
            change += std::abs(new_value - old_value);
        }
        return change;
    }

    bool goal_changed(const geometry_msgs::msg::Pose2D &old_goal, const geometry_msgs::msg::Pose2D &new_goal) {
        return std::abs(old_goal.x - new_goal.x) > GOAL_TOLERANCE || std::abs(old_goal.y - new_goal.y) > GOAL_TOLERANCE;
    }

    void request_new_plan() {
        RCLCPP_INFO(this->get_logger(), "REQUEST NEW PLAN CALLED");
        if (!have_map_ || !have_goal_ || !have_odom_) {
            return;
        }
        if (!plan_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "A* service is not available");
            return;
        }
        
        auto request = std::make_shared<navigation_server::srv::PlanPath::Request>();

        request->start.x = current_odom_.pose.pose.position.x;
        request->start.y = current_odom_.pose.pose.position.y;
        request->start.theta = 0.0;

        request->goal = current_goal_;
        request->map = current_map_;

        plan_client_->async_send_request(
            request,
            [this, request](
                rclcpp::Client<navigation_server::srv::PlanPath>::SharedFuture future)
            {
                auto response = future.get();

                if (!response)
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "A* service returned no response");

                    return;
                }

                last_planned_goal_ = request->goal;
                have_last_planned_goal_ = true;

                last_verified_map_ = request->map;
                have_last_verified_map_ = true;

                current_path_ = response->plan;
                have_path_ = true;

                RCLCPP_INFO(
                    this->get_logger(),
                    "New path received from A*");
            });
    }

};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MapVerifierServer>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}