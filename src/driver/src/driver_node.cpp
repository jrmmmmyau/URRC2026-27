#include "driver/driver.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto driver = std::make_shared<driver::Driver>();
    rclcpp::spin(driver);
    rclcpp::shutdown();
    return 0;
}
