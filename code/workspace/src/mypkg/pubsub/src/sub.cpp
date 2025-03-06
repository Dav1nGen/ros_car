// ros2
#include <rclcpp/rclcpp.hpp>
// Pub
#include "pubsub/sub.hpp"

int main(int argc, char* argv[])
{
    // init
    rclcpp::init(argc, argv);
    auto sub_node = std::make_shared<Sub>("sub1");
    // loop
    rclcpp::spin(sub_node);
    // shutdown
    rclcpp::shutdown();
    return 0;
}