#ifndef GUI__MAIN_PANEL_HPP_
#define GUI__MAIN_PANEL_HPP_

#include <memory>

#include <rviz_common/panel.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <QPlainTextEdit>
#include <rcl_interfaces/msg/log.hpp>

#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <QLabel>
#include <QWidget>

namespace gui
{

class MainPanel : public rviz_common::Panel
{
  Q_OBJECT

public:
  explicit MainPanel(QWidget * parent = nullptr);
  ~MainPanel() override;

  void onInitialize() override;

private:
    // ros
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> node_ptr_;
    
     // Separate ROS node for the embedded RViz instance
    std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> rviz_node_ptr;

    //embedded rviz
    rviz_common::RenderPanel * map_view_;
    std::unique_ptr<rviz_common::VisualizationManager> rviz_manager_;

    // camera
    QLabel * camera_view_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

    // pose view
    QLabel * pose_view_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

    // Current stage
    QLabel * stage_view_;

    // Robot status / ROS logs
    QPlainTextEdit * status_view_;
    rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;
    void logCallback(const rcl_interfaces::msg::Log::ConstSharedPtr msg);

};
}  // namespace gui

#endif  // GUI__MAIN_PANEL_HPP_
