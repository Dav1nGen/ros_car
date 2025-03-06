#include "obstacle_avoidance.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto avoidance_node = std::make_shared<Avoidancer>("avoidancer");
    rclcpp::spin(avoidance_node);
    rclcpp::shutdown();
    return 0;
}
