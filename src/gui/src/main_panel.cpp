#include "gui/main_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>

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
#include <cmath>
#include <QFrame>
#include <QMetaObject>
#include <Qt>

#include <functional>
#include <pluginlib/class_list_macros.hpp>

namespace gui
{
// Constructor
MainPanel::MainPanel(QWidget * parent)
: rviz_common::Panel(parent), map_view_(nullptr),  camera_view_(nullptr)
{
    // overall layout
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
    

    // pose odom 
    auto* pose_frame = new QFrame(this);
    pose_frame->setFixedHeight(70);
    pose_frame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    pose_frame->setFrameShape(QFrame::StyledPanel);
    pose_frame->setStyleSheet("QFrame { background-color: #333; border: 1px solid gray; }");

    auto* pose_layout = new QVBoxLayout(pose_frame);
    auto* pose_title = new QLabel("Robot Pose", pose_frame);
    pose_title->setStyleSheet("font-weight: bold; color: white; border: none; font-size: 11px;");
    pose_view_ = new QLabel("X: --\nY: --\nHeading: --", pose_frame);
    pose_view_->setStyleSheet("color: white; border: none; font-size: 10px;");
    pose_view_->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

    pose_layout->addWidget(pose_title);
    pose_layout->addWidget(pose_view_);

    left_layout->addWidget(pose_frame);
    left_layout->addStretch(2);

    // center column (only the map)
    auto* center_layout = new QVBoxLayout();

    // rviz render panel
   /* map_view_ = new rviz_common::RenderPanel(this);
    
    map_view_->setMinimumSize(300, 300);
    map_view_->setSizePolicy(
        QSizePolicy::Expanding, QSizePolicy::Expanding);

    center_layout->addWidget(map_view_); */

    // right column (more buttons idk what yet, probably some click to nav)
    auto* right_layout = new QVBoxLayout();
    right_layout->addStretch();

    // add columns to main layout
    main_layout->addLayout(left_layout, 1);
    main_layout->addLayout(center_layout, 2);
    main_layout->addLayout(right_layout, 1);


}

MainPanel::~MainPanel() = default;

void MainPanel::onInitialize()
{
    auto logger = rclcpp::get_logger("MainPanel");

    RCLCPP_INFO(logger, "1. onInitialize started");

    // Get the parent RViz ROS node
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();

    RCLCPP_INFO(logger, "2. Got ROS node abstraction");

    if (!node_ptr_) {
        RCLCPP_ERROR(logger, "Failed to get ROS node abstraction");
        return;
    }

    auto node = node_ptr_->get_raw_node();

    RCLCPP_INFO(logger, "3. Got raw node");

    // Camera subscription uses the ORIGINAL RViz node
    image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera/camera/color/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(
            &MainPanel::imageCallback,
            this,
            std::placeholders::_1));

    // odom sub
    odom_sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(
            &MainPanel::odomCallback,
            this,
            std::placeholders::_1
        )
    );

    // Initialize the embedded render window
    /* map_view_->getRenderWindow()->initialize();

    RCLCPP_INFO(logger, "4. Render window initialized");

    // Create a SEPARATE ROS node for the embedded RViz instance
    rviz_node_ptr =
        std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
            "embedded_map_rviz");

    RCLCPP_INFO(logger, "5. Created separate ROS node");

    // Create the embedded VisualizationManager
    rviz_manager_ =
        std::make_unique<rviz_common::VisualizationManager>(
            map_view_,
            rviz_node_ptr,
            nullptr,
            rviz_node_ptr->get_raw_node()->get_clock());

    RCLCPP_INFO(logger, "6. Created visualization manager");

    // Initialize the render panel
    map_view_->initialize(rviz_manager_.get());

    RCLCPP_INFO(logger, "7. Initialized render panel");

    // Initialize and start the manager
    rviz_manager_->initialize();

    RCLCPP_INFO(logger, "8. Initialized visualization manager");

    rviz_manager_->startUpdate();

    RCLCPP_INFO(logger, "9. Started visualization manager");


    // Set map frame
    rviz_manager_->setFixedFrame("camera_depth_optical_frame");

    RCLCPP_INFO(logger, "10. Set fixed frame");

    // Create map display
    rviz_common::Display * map_display =
        rviz_manager_->createDisplay(
            "rviz_default_plugins/Map",
            "Occupancy Grid",
            true);

    RCLCPP_INFO(logger, "11. Created map display");

    if (map_display) {
        auto* topic_property = map_display->subProp("Topic");

        if (topic_property) {
            topic_property->setValue("/occupancy_grid");
        }
    } else {
        RCLCPP_ERROR(
            node->get_logger(),
            "Failed to create occupancy grid");
    }

    RCLCPP_INFO(logger, "12. Finished onInitialize"); */
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

// callback for odom
void MainPanel::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;

    auto q = msg->pose.pose.orientation;
    double yaw = std::atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    );

    double heading = yaw * 180.0 / M_PI;
    QMetaObject::invokeMethod(
        pose_view_,
        [this, x, y, heading]() {
            pose_view_->setText(
                QString("X: %1 m\nY: %2 m\nHeading: %3")
                    .arg(x, 0, 'f', 2)
                    .arg(y, 0, 'f', 2)
                    .arg(heading, 0, 'f', 1)
            );
        },
        Qt::QueuedConnection
    );
}

} // namespace gui


PLUGINLIB_EXPORT_CLASS(gui::MainPanel, rviz_common::Panel)
