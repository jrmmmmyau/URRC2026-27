#include "gui/main_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <opencv2/imgproc.hpp>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSizePolicy>
#include <QWidget>
#include <QImage>
#include <QPixmap>
#include <QLabel>
#include <QMetaObject>
#include <Qt>

#include <functional>
#include <pluginlib/class_list_macros.hpp>

namespace gui
{
// Constructor
MainPanel::MainPanel(QWidget * parent)
: rviz_common::Panel(parent), camera_view_(nullptr)
{
    // overall layour
    auto* main_layout = new QHBoxLayout(this);

    main_layout->setContentsMargins(8, 8, 8, 8);
    main_layout->setSpacing(8);

    // left column (has camera at top)
    auto* left_layout = new QVBoxLayout();

    // camera view
    camera_view_ = new QLabel("No camera feed");
    camera_view_->setAlignment(Qt::AlignCenter);
    camera_view_->setMinimumSize(160, 120);
    camera_view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    camera_view_->setStyleSheet("background-color: black; color: white; border: 1px solid gray;");

    left_layout->addWidget(camera_view_, 1);
    left_layout->addStretch(2);

    // center column (only the map whatever form we end up going with - im gonna just do occupancy grid for now)
    auto* center_layout = new QVBoxLayout();

    // right column (more buttons idk what yet)
    auto* right_layout = new QVBoxLayout();

    // add columns to main layout
    main_layout->addLayout(left_layout, 1);
    main_layout->addLayout(center_layout, 2);
    main_layout->addLayout(right_layout, 1);
}

MainPanel::~MainPanel() = default;

void MainPanel::onInitialize()
{
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    if (!node_ptr_) {
        RCLCPP_ERROR(rclcpp::get_logger("MainPanel"), "Failed to get ROS node abstraction");
        return;
    }

    auto node = node_ptr_->get_raw_node();

    // Subscribe to the camera image topic
    image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw", 
        rclcpp::SensorDataQoS(),
        std::bind(&MainPanel::imageCallback, this, std::placeholders::_1));
}

// Callback function for image subscription
void MainPanel::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    try
    {
        // convert ros image to open cv bgr image
        auto cv_image = cv_bridge::toCvCopy(
            msg, sensor_msgs::image_encodings::BGR8
        );

        // convert bgr to rgb
        cv::Mat rgb_image;

        cv::cvtColor(cv_image->image, rgb_image, cv::COLOR_BGR2RGB);

        // convert open cv image to qt image
        QImage qt_image(
            rgb_image.data,
            rgb_image.cols,
            rgb_image.rows,
            static_cast<int>(rgb_image.step),
            QImage::Format_RGB888
        );

        QImage image_copy = qt_image.copy();

        // update gui in the main thread
        QMetaObject::invokeMethod(
            camera_view_,
            [this, image_copy]() {
                camera_view_->setPixmap(QPixmap::fromImage(image_copy).scaled(
                    camera_view_->size(),
                    Qt::KeepAspectRatio,
                    Qt::SmoothTransformation
                ));
            },
            Qt::QueuedConnection
        );
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(node_ptr_->get_raw_node()->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

} // namespace gui


PLUGINLIB_EXPORT_CLASS(gui::MainPanel, rviz_common::Panel)