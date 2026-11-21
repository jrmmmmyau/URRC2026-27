#include "gui/main_panel.hpp"

#include <rviz_common/display_context.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_rendering/render_window.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <QApplication>

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

    // Current stage frame
    auto* stage_frame = new QFrame(this);
    stage_frame->setFixedHeight(65);
    stage_frame->setFrameShape(QFrame::StyledPanel);
    stage_frame->setStyleSheet(
        "QFrame { background-color: #333; border: 1px solid gray; }"
    );

    auto* stage_layout = new QVBoxLayout(stage_frame);

    auto* stage_title = new QLabel("Current Stage", stage_frame);
    stage_title->setStyleSheet(
        "font-weight: bold; color: white; border: none; font-size: 11px;"
    );

    stage_view_ = new QLabel("", stage_frame);
    stage_view_->setStyleSheet(
        "color: white; border: none; font-size: 11px;"
    );

    stage_layout->addWidget(stage_title);
    stage_layout->addWidget(stage_view_);

    left_layout->addWidget(stage_frame);

    // Robot status frame
    auto* status_frame = new QFrame(this);
    status_frame->setFrameShape(QFrame::StyledPanel);
    status_frame->setStyleSheet(
        "QFrame { background-color: #333; border: 1px solid gray; }"
    );

    auto* status_layout = new QVBoxLayout(status_frame);

    auto* status_title = new QLabel("Robot Status", status_frame);
    status_title->setStyleSheet(
        "font-weight: bold; color: white; border: none; font-size: 11px;"
    );

    status_view_ = new QPlainTextEdit(status_frame);
    status_view_->setReadOnly(true);
    status_view_->setMaximumBlockCount(100);
    status_view_->setPlaceholderText("Waiting for robot messages...");

    status_view_->setStyleSheet(
        "QPlainTextEdit {"
        "background-color: #222;"
        "color: white;"
        "border: none;"
        "font-size: 10px;"
        "}"
    );

    status_layout->addWidget(status_title);
    status_layout->addWidget(status_view_);

    left_layout->addWidget(status_frame, 1);

    left_layout->addStretch(2);

    // center column (only the map)
    auto* center_layout = new QVBoxLayout();

    // rviz render panel
    map_view_ = new rviz_common::RenderPanel(this);
    
    map_view_->setMinimumSize(300, 300);
    map_view_->setSizePolicy(
        QSizePolicy::Expanding, QSizePolicy::Expanding);

    center_layout->addWidget(map_view_);

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

    // Get the parent RViz ROS node
    node_ptr_ = getDisplayContext()->getRosNodeAbstraction().lock();


    if (!node_ptr_) {
        RCLCPP_ERROR(logger, "Failed to get ROS node abstraction");
        return;
    }

    auto node = node_ptr_->get_raw_node();


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

    // Subscribe to ROS logs
    log_sub_ = node->create_subscription<rcl_interfaces::msg::Log>(
        "/rosout",
        rclcpp::QoS(100),
        std::bind(
            &MainPanel::logCallback,
            this,
            std::placeholders::_1
        )
    );

    // initialize the embedded render window
    QApplication::processEvents();
    RCLCPP_INFO(logger, "A. Initializing embedded RViz render window");
    map_view_->getRenderWindow()->initialize();
    RCLCPP_INFO(logger, "B. Embedded RViz render window initialized");

    // new node bc keep getting seg faults
    rviz_node_ptr_ = std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>(
        "embedded_map_rviz"
    );

    rviz_manager_ = std::make_unique<rviz_common::VisualizationManager>(
        map_view_,
        rviz_node_ptr_,
        nullptr,
        rviz_node_ptr_->get_raw_node()->get_clock()
    );

    RCLCPP_INFO(logger, "C. Embedded RViz VisualizationManager initializing");
    map_view_->initialize(rviz_manager_.get());
    rviz_manager_->initialize();
    RCLCPP_INFO(logger, "D. Embedded RViz VisualizationManager initialized");

    // test 5004334 
    rviz_manager_->setFixedFrame("camera_depth_optical_frame");
    rviz_common::Display* grid_display = rviz_manager_->createDisplay(
        "rviz_default_plugins/Grid",
        "Reference Grid",
        true
    );

    if (!grid_display) {
        RCLCPP_ERROR(logger, "Failed to create grid display");
        return;
    }

    rviz_common::Display* map_display = rviz_manager_->createDisplay(
        "rviz_default_plugins/Map",
        "Occupancy Grid",
        true
    );

    if (!map_display) {
        RCLCPP_ERROR(logger, "Failed to create occupancy grid display");
        return;
    } 

    rviz_manager_->startUpdate();

}

// Callback function for image subscription
void MainPanel::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
    
    try
    {
        // convert ros image to open cv bgr image
        auto cv_image = cv_bridge::toCvCopy(
            msg, sensor_msgs::image_encodings::RGB8
        );

        // convert bgr to rgb
        cv::Mat rgb_image = cv_image->image;

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
        RCLCPP_ERROR(node_ptr_->get_raw_node()->get_logger(), 
        "main panel image callback error: %s", e.what());
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
                QString("X: %1 m\nY: %2 m\nHeading: %3 deg")
                    .arg(x, 0, 'f', 2)
                    .arg(y, 0, 'f', 2)
                    .arg(heading, 0, 'f', 1)
            );
        },
        Qt::QueuedConnection
    );
}

void MainPanel::logCallback(
    const rcl_interfaces::msg::Log::ConstSharedPtr msg)
{
    QString level;

    switch (msg->level)
    {
        case rcl_interfaces::msg::Log::DEBUG:
            level = "DEBUG";
            break;

        case rcl_interfaces::msg::Log::INFO:
            level = "INFO";
            break;

        case rcl_interfaces::msg::Log::WARN:
            level = "WARN";
            break;

        case rcl_interfaces::msg::Log::ERROR:
            level = "ERROR";
            break;

        case rcl_interfaces::msg::Log::FATAL:
            level = "FATAL";
            break;

        default:
            level = "UNKNOWN";
            break;
    }

    QString log_message = QString("[%1] %2: %3")
        .arg(level)
        .arg(QString::fromStdString(msg->name))
        .arg(QString::fromStdString(msg->msg));

    QMetaObject::invokeMethod(
        status_view_,
        [this, log_message]() {
            status_view_->appendPlainText(log_message);
        },
        Qt::QueuedConnection
    );
}


} // namespace gui


PLUGINLIB_EXPORT_CLASS(gui::MainPanel, rviz_common::Panel)
