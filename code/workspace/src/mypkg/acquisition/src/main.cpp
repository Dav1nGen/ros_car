/**
 * @file main.cpp
 * @author luoyebai (2112216825@qq.com)
 * @brief acquisition.hpp must be included before register_node_macro.hpp
 * @version 0.1
 * @date 2023-04-28
 *
 * @copyright Copyright (c) 2023
 *
 */

// acquisition
#include "acquisition/acquisition.hpp"

int main(int argc, const char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AcquisitionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
