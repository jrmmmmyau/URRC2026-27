#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

#include "navigation_server/srv/plan_path.hpp"
#include "costmap.hpp"
#include "costmap_params.hpp"

// A cell counts as "changed" if its decoded value moved by at least this much (0..254 scale)
// or it flipped between unknown and known.
const int CELL_CHANGE_MIN = 30;
// Only run the (more expensive) path check once this many cells changed since the last verified map.
const size_t MAP_CHANGE_MIN_CELLS = 20;
const float GOAL_TOLERANCE = 0.001f;

class MapVerifierServer : public rclcpp::Node
{
public:
    MapVerifierServer()
        : Node("map_verifier_server")
    {
        cm_params_ = load_costmap_params(*this);

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 10, std::bind(&MapVerifierServer::map_callback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("/goal", 10, std::bind(&MapVerifierServer::goal_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&MapVerifierServer::odom_callback, this, std::placeholders::_1));
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>("/plan", 10, std::bind(&MapVerifierServer::path_callback, this, std::placeholders::_1));

        plan_client_ = this->create_client<navigation_server::srv::PlanPath>("/plan_path");
        RCLCPP_INFO(this->get_logger(), "Map verifier server started");
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_map_ = *msg;
        have_map_ = true;

        if (!have_last_verified_map_)
        {
            last_verified_map_ = current_map_;
            have_last_verified_map_ = true;
            if (have_goal_ && have_odom_) {
                request_new_plan();
            }
            return;
        }

        size_t changed = count_changed_cells(last_verified_map_, current_map_);
        if (changed < MAP_CHANGE_MIN_CELLS) {
            return;
        }

        if (map_change_affects_path()) {
            RCLCPP_INFO(this->get_logger(), "%zu cells changed and the path is no longer clear, replanning", changed);
            request_new_plan();
        } else {
            last_verified_map_ = current_map_;
        }
    }

    void goal_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
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

        if (goal_changed(last_planned_goal_, current_goal_)) {
            request_new_plan();
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_odom_ = *msg;
        have_odom_ = true;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        current_path_ = *msg;
        // The planner publishes an empty path when planning fails; that means "no valid path".
        have_path_ = !msg->poses.empty();
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Client<navigation_server::srv::PlanPath>::SharedPtr plan_client_;

    costmap::Params cm_params_;

    nav_msgs::msg::OccupancyGrid current_map_;
    nav_msgs::msg::OccupancyGrid last_verified_map_;
    geometry_msgs::msg::Pose2D current_goal_;
    geometry_msgs::msg::Pose2D last_planned_goal_;
    nav_msgs::msg::Odometry current_odom_;
    nav_msgs::msg::Path current_path_;

    bool have_map_ = false;
    bool have_last_verified_map_ = false;
    bool have_goal_ = false;
    bool have_last_planned_goal_ = false;
    bool have_odom_ = false;
    bool have_path_ = false;
    bool plan_in_flight_ = false;
    bool replan_queued_ = false;

    std::pair<int, int> world_to_grid(double world_x, double world_y, const nav_msgs::msg::OccupancyGrid &map)
    {
        int grid_x = static_cast<int>(std::floor((world_x - map.info.origin.position.x) / map.info.resolution));
        int grid_y = static_cast<int>(std::floor((world_y - map.info.origin.position.y) / map.info.resolution));
        return {grid_x, grid_y};
    }

    // True if the remaining part of the path (from the pose nearest the robot onward) now
    // crosses a blocked cell of the inflated costmap, or leaves the map.
    bool map_change_affects_path()
    {
        if (!have_path_ || current_path_.poses.empty()) {
            return true;
        }

        costmap::Costmap cm = costmap::build(current_map_.data,
                                             static_cast<int>(current_map_.info.width),
                                             static_cast<int>(current_map_.info.height),
                                             current_map_.info.resolution, cm_params_);

        const double rx = current_odom_.pose.pose.position.x;
        const double ry = current_odom_.pose.pose.position.y;
        size_t first = 0;
        double best = std::numeric_limits<double>::max();
        for (size_t i = 0; i < current_path_.poses.size(); i++) {
            double dx = current_path_.poses[i].pose.position.x - rx;
            double dy = current_path_.poses[i].pose.position.y - ry;
            double d = dx * dx + dy * dy;
            if (d < best) { best = d; first = i; }
        }

        for (size_t i = first; i < current_path_.poses.size(); i++) {
            const auto &pos = current_path_.poses[i].pose.position;
            auto [gx, gy] = world_to_grid(pos.x, pos.y, current_map_);
            if (!cm.traversable(gx, gy)) {
                return true;
            }
        }
        return false;
    }

    size_t count_changed_cells(const nav_msgs::msg::OccupancyGrid &old_map, const nav_msgs::msg::OccupancyGrid &new_map)
    {
        if (old_map.data.size() != new_map.data.size() ||
            old_map.info.width != new_map.info.width ||
            old_map.info.resolution != new_map.info.resolution ||
            old_map.info.origin.position.x != new_map.info.origin.position.x ||
            old_map.info.origin.position.y != new_map.info.origin.position.y) {
            return std::numeric_limits<size_t>::max();  // geometry changed: always check
        }
        size_t changed = 0;
        for (size_t i = 0; i < old_map.data.size(); i++) {
            int o = costmap::decode_cell(old_map.data[i]);
            int n = costmap::decode_cell(new_map.data[i]);
            if ((o == costmap::UNKNOWN) != (n == costmap::UNKNOWN)) {
                changed++;
            } else if (o != costmap::UNKNOWN && std::abs(n - o) >= CELL_CHANGE_MIN) {
                changed++;
            }
        }
        return changed;
    }

    bool goal_changed(const geometry_msgs::msg::Pose2D &old_goal, const geometry_msgs::msg::Pose2D &new_goal)
    {
        return std::abs(old_goal.x - new_goal.x) > GOAL_TOLERANCE || std::abs(old_goal.y - new_goal.y) > GOAL_TOLERANCE;
    }

    void request_new_plan()
    {
        if (!have_map_ || !have_goal_ || !have_odom_) {
            return;
        }
        if (!plan_client_->service_is_ready()) {
            RCLCPP_WARN(this->get_logger(), "A* service is not available");
            return;
        }
        if (plan_in_flight_) {
            replan_queued_ = true;  // run once more with the freshest data when this one returns
            return;
        }

        auto request = std::make_shared<navigation_server::srv::PlanPath::Request>();
        request->start.x = current_odom_.pose.pose.position.x;
        request->start.y = current_odom_.pose.pose.position.y;
        request->start.theta = 0.0;
        request->goal = current_goal_;
        request->map = current_map_;

        plan_in_flight_ = true;
        plan_client_->async_send_request(
            request,
            [this, request](rclcpp::Client<navigation_server::srv::PlanPath>::SharedFuture future)
            {
                plan_in_flight_ = false;
                auto response = future.get();

                if (!response || response->plan.poses.empty())
                {
                    // Do NOT record the goal/map as handled, so a later map update or goal retries.
                    RCLCPP_WARN(this->get_logger(), "A* found no path; will retry when the map changes");
                    current_path_ = nav_msgs::msg::Path();
                    have_path_ = false;
                } else {
                    last_planned_goal_ = request->goal;
                    have_last_planned_goal_ = true;
                    last_verified_map_ = request->map;
                    have_last_verified_map_ = true;
                    current_path_ = response->plan;
                    have_path_ = true;
                    RCLCPP_INFO(this->get_logger(), "New path received from A* (%zu poses)", response->plan.poses.size());
                }

                if (replan_queued_) {
                    replan_queued_ = false;
                    request_new_plan();
                }
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
