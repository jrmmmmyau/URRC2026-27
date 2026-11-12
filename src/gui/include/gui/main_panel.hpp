#ifndef GUI__MAIN_PANEL_HPP_
#define GUI__MAIN_PANEL_HPP_

#include <memory>
#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <QLabel>

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

    // camera
    QLabel * camera_view_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
};
}  // namespace gui

#endif  // GUI__MAIN_PANEL_HPP_