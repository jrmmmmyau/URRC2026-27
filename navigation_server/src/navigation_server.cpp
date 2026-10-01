#include "rclcpp/rclcpp.hpp"
#include "navigation_server/srv/plan_path.hpp"

#include <memory>
#include <cmath>
#include <vector>
#include <algorithm>

bool is_free(const nav_msgs::msg::OccupancyGrid &map, int x, int y) 
{
    if (x < 0 or x >= static_cast<int>(map.info.width)) {
    return false;
    }
    if (y < 0 or y >= static_cast<int>(map.info.height)) {
    return false;
    }
    
    int index = x + (map.info.width * y);
    
    if (map.data[index] != 0) {
    return false;
    }

    return true;
}

void world_to_grid(const nav_msgs::msg::OccupancyGrid &map, float x, float y, int &grid_x, int &grid_y)
{
    grid_x = floor((x - map.info.origin.position.x) / map.info.resolution);
    grid_y = floor((y - map.info.origin.position.y) / map.info.resolution);
}

void grid_to_world(const nav_msgs::msg::OccupancyGrid &map, float x, float y, double &world_x, double &world_y)
{
    world_x = (x + 0.5) * map.info.resolution + map.info.origin.position.x;
    world_y = (y + 0.5) * map.info.resolution + map.info.origin.position.y;
}

bool map_validation(const nav_msgs::msg::OccupancyGrid &map)
{
    if ((map.data.size() != static_cast<size_t>(map.info.width) * static_cast<size_t>(map.info.height)) or map.data.size() == 0 or map.info.resolution <= 0.0f) {
        return false;
    }
    return true;
}

struct Node
{
    int x;
    int y;
    float g;
    float h;
    float f;
    int parent_x;
    int parent_y;
    int dx;
    int dy;
};

void pathfind(const std::shared_ptr<navigation_server::srv::PlanPath::Request> request,
          std::shared_ptr<navigation_server::srv::PlanPath::Response> response)
{
    if (!map_validation(request->map)){
        RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "Invalid Map");
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "Start: %f, %f Goal: %f, %f", request->start.x, request->start.y, request->goal.x, request->goal.y);
  //a* beginnings
    // initialize start node and variables
    
    const float TURN_COST = 10.0f;
    std::vector<Node> open_list;
    std::vector<Node> closed_list;
    struct Node start_node;
    int start_x;
    int start_y;
    int goal_x;
    int goal_y;
    double path_world_x;
    double path_world_y;
    int best_index = 0;
    int open_index;
    float lowest_f;
    bool in_closed;
    bool in_open = false;
    world_to_grid(request->map, request->start.x, request->start.y, start_x, start_y);
    world_to_grid(request->map, request->goal.x, request->goal.y, goal_x, goal_y);
    
    RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "Start: (%d, %d)", start_x, start_y);
    RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "goal: (%d, %d)", goal_x, goal_y);
    if (!(is_free(request->map, start_x, start_y) and is_free(request->map, goal_x, goal_y))) {
        RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "start or goal is invalid");
        return;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "start and goal are valid");
    }
    start_node.x = start_x;
    start_node.y = start_y;
    start_node.g = 0;
    start_node.dx = 0;
    start_node.dy = 0;
    start_node.h = std::sqrt(std::pow((goal_x - start_node.x),2) + std::pow((goal_y - start_node.y),2));
    start_node.f = start_node.g + start_node.h;
    lowest_f = start_node.f;
    open_list.push_back(start_node);
    for (size_t i = 0; i < open_list.size(); i++)
        {
            if (open_list[i].f < lowest_f){
                lowest_f = open_list[i].f;
                best_index = i;
            }
        };

    Node current = open_list[best_index];

    while (true)
    {
        if (!(current.x == goal_x && current.y == goal_y))
        {
            if (!open_list.empty())
            {
                // loop that can find the index with the lowest f
                lowest_f = open_list[0].f;
                best_index = 0;
                for (size_t i = 0; i < open_list.size(); i++)
                    {
                        if (open_list[i].f < lowest_f){
                            lowest_f = open_list[i].f;
                            best_index = i;
                        }
                    };

                current = open_list[best_index];
                open_list.erase(open_list.begin() + best_index);
                closed_list.push_back(current);
                //RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "current: (%d, %d)", current.x, current.y);
                // loop that creates all the new nodes
                for (int dx = -1; dx <= 1; dx++)
                    {
                        for (int dy = -1; dy <= 1; dy++)
                            {
                                if (dx == 0 and dy ==0){
                                    continue;
                                }
                                if ((is_free(request->map, current.x + dx, current.y + dy)) and 
                                ((is_free(request->map, current.x + dx, current.y)) and ((is_free(request->map, current.x, current.y + dy)) or 
                                dx == 0 or 
                                dy == 0)))
                                {
                                    struct Node new_node;
                                    new_node.x = current.x + dx;
                                    new_node.y = current.y + dy;
                                    new_node.dx = dx;
                                    new_node.dy = dy;
                                    new_node.parent_x = current.x;
                                    new_node.parent_y = current.y;
                                    in_closed = false;
                                    in_open = false;
                                    open_index = -1;
                                    for (size_t i = 0; i < closed_list.size(); i++)
                                        {
                                            if (new_node.x == closed_list[i].x and new_node.y == closed_list[i].y){
                                                in_closed = true;
                                            }
                                        };
                                    for (size_t i = 0; i < open_list.size(); i++)
                                        {
                                            if (new_node.x == open_list[i].x && new_node.y == open_list[i].y){
                                                in_open = true;
                                                open_index = i;
                                            }
                                        }
                                    if (!in_closed){
                                        if (dx != 0 and dy !=0){
                                                new_node.g = (std::sqrt(2.0f) + current.g);
                                            } else {
                                                new_node.g = (1 + current.g);
                                            }
                                        if (current.dx != 0 || current.dy != 0){
                                                if (dx != current.dx || dy != current.dy)
                                                {
                                                    new_node.g += TURN_COST;
                                                }
                                            }
                                        new_node.h = std::sqrt(std::pow((goal_x - new_node.x),2) + std::pow((goal_y - new_node.y),2));
                                        new_node.f = new_node.g + new_node.h;   
                                        if (!in_open){ 
                                            open_list.push_back(new_node);
                                            //RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "Neghibor: (%d, %d)", new_node.x, new_node.y);
                                            //RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "Node (%d, %d) came from (%d, %d)", new_node.x, new_node.y, new_node.parent_x, new_node.parent_y);
                                        } else {
                                            if (new_node.g < open_list[open_index].g) {
                                                open_list[open_index].g = new_node.g;
                                                open_list[open_index].f = new_node.f;
                                                open_list[open_index].parent_x = new_node.parent_x;
                                                open_list[open_index].parent_y = new_node.parent_y;
                                                open_list[open_index].dx = new_node.dx;
                                                open_list[open_index].dy = new_node.dy;
                                            }
                                        }
                                    }
                                }
                            }
                    }
            } else {
                RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "no nodes left to search, no path found");
                break;
            }
        } else {
            RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "Goal reached");
            RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "end location: (%d, %d)", current.x, current.y);
            std::vector<Node> path;
            Node path_node = current;
            while (!(path_node.x == start_x and path_node.y == start_y)) {
                //RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "path node: (%d, %d)", path_node.x, path_node.y);
                path.push_back(path_node);
                for (size_t i = 0; i < closed_list.size(); i++)
                    {
                        if (path_node.parent_x == closed_list[i].x and path_node.parent_y == closed_list[i].y){
                            path_node = closed_list[i];
                            break;
                        }
                    };
            }
            path.push_back(path_node);
            std::reverse(path.begin(), path.end());
            //for (size_t i = 0; i < path.size(); i++) {RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "Path node: (%d, %d)", path[i].x, path[i].y);}
            nav_msgs::msg::Path plan;
            plan.header = request->map.header;
            for (size_t i = 0; i < path.size(); i++)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header = plan.header;
                    grid_to_world(request->map, path[i].x, path[i].y, path_world_x, path_world_y);
                    pose.pose.position.x = path_world_x;
                    pose.pose.position.y = path_world_y;
                    double yaw;
                    if (path.size() == 1) {
                        yaw = request->start.theta;
                    } else if (i < path.size() - 1)
                    {
                        double dx = path[i + 1].x - path[i].x;
                        double dy = path[i + 1].y - path[i].y;
                        yaw = std::atan2(dy, dx);
                    } else
                    {
                        double dx = path[i].x - path[i - 1].x;
                        double dy = path[i].y - path[i - 1].y;
                        yaw = std::atan2(dy, dx);
                    }
                    pose.pose.orientation.z = std::sin(yaw / 2.0);
                    pose.pose.orientation.w = std::cos(yaw / 2.0);
                    plan.poses.push_back(pose);
                }
            response->plan = plan;
            break;
        }
    }
    
    
  RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "finished");  
}



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("navigation_server");

  rclcpp::Service<navigation_server::srv::PlanPath>::SharedPtr service =
    node->create_service<navigation_server::srv::PlanPath>("plan_path", &pathfind);

  RCLCPP_INFO(rclcpp::get_logger("navigation_server"), "Ready to map, ts is working");

  rclcpp::spin(node);
  rclcpp::shutdown();
}