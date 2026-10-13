#pragma once
// Declares/reads the costmap ROS parameters. Both the planner and the verifier call this
// so they always build the same costmap (set the values once in a shared params YAML).
#include <rclcpp/rclcpp.hpp>
#include "costmap.hpp"

inline costmap::Params load_costmap_params(rclcpp::Node &node)
{
    costmap::Params p;
    p.robot_radius      = static_cast<float>(node.declare_parameter<double>("robot_radius", p.robot_radius));
    p.safety_margin     = static_cast<float>(node.declare_parameter<double>("safety_margin", p.safety_margin));
    p.inflation_radius  = static_cast<float>(node.declare_parameter<double>("inflation_radius", p.inflation_radius));
    p.inflation_scale   = static_cast<float>(node.declare_parameter<double>("inflation_scale", p.inflation_scale));
    p.cost_decay        = static_cast<float>(node.declare_parameter<double>("cost_decay", p.cost_decay));
    p.occupied_threshold = static_cast<int>(node.declare_parameter<int64_t>("occupied_threshold", p.occupied_threshold));
    p.gray_weight       = static_cast<float>(node.declare_parameter<double>("gray_weight", p.gray_weight));
    p.allow_unknown     = node.declare_parameter<bool>("allow_unknown", p.allow_unknown);
    p.unknown_cost      = static_cast<float>(node.declare_parameter<double>("unknown_cost", p.unknown_cost));
    return p;
}
