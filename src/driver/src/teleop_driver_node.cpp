#include "driver/teleop_driver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto teleop_driver = std::make_shared<teleop_driver::Teleop_driver>();
    rclcpp::spin(teleop_driver);
    rclcpp::shutdown();
    return 0;
}
