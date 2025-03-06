// Copyright[2024] <davi>
#include "walk_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto avoidance_node = std::make_shared<WalkController>("walk_controller");
    rclcpp::spin(avoidance_node);
    rclcpp::shutdown();
    return 0;
}
