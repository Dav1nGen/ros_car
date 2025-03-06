/**
 * @file follow.cpp
 * @author davi (davicheng1114@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-05-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "line_follow/follow.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Follow>("line_follow");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
