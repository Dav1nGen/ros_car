#include "jubot_driver/jubot_driver.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto driver_node = std::make_shared<JubotDriver>();
    rclcpp::spin(driver_node);
    driver_node->SetVelocity(0, 0, 0);
    rclcpp::shutdown();
    return 0;
}
