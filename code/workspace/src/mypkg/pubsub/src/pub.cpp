// ros2
#include <rclcpp/rclcpp.hpp>
// pub
#include "pubsub/pub.hpp"

int main(int argc, const char *argv[])
{
    // init
    rclcpp::init(argc, argv);
    auto pub_node = std::make_shared<Pub>("pub1");
    // loop
    rclcpp::spin(pub_node);
    // shutdown
    rclcpp::shutdown();
    return 0;
}
