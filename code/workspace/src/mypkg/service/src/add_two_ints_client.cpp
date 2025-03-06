// ros2
#include <example_interfaces/srv/add_two_ints.hpp>
#include <rclcpp/rclcpp.hpp>
// std
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;  // NOLINT

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    if (argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                    "需要输入参数: 两个相加的整数 X Y");
        return 1;
    }
    auto node   = rclcpp::Node::make_shared("add_two_ints_client");
    auto client = node->create_client<example_interfaces::srv::AddTwoInts>(
        "add_two_ints");
    auto request =
        std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    while (!client->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(node->get_logger(), "服务已经被中断，退出客户端");
            return 0;
        }
        RCLCPP_INFO(node->get_logger(), "服务暂时不可用，请耐心等待");
    }
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(node->get_logger(), "服务返回结果: %ld", result.get()->sum);
    else
        RCLCPP_ERROR(node->get_logger(), "两整数相加服务出错");
    rclcpp::shutdown();
    return 0;
}
