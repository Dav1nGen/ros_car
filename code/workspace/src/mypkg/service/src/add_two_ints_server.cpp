// ros2
#include <rclcpp/rclcpp.hpp>
// std
#include <memory>  //NOLINT
// insterfaces
#include <example_interfaces/srv/add_two_ints.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("add_two_ints_service");
    auto add =
        [&node](
            const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>
                request,
            std::shared_ptr<example_interfaces::srv::AddTwoInts::Response>
                response) {
            response->sum = request->a + request->b;
            RCLCPP_INFO(node->get_logger(), "正在获取数据请求\na: %ld b: %ld",
                        request->a, request->b);
            RCLCPP_INFO(node->get_logger(), "返回响应: [%ld]", response->sum);
        };
    auto service = node->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints", add);
    RCLCPP_INFO(node->get_logger(), "服务端初始化完毕，可以开始相加");
    rclcpp::spin(node);
    rclcpp::shutdown();
}  // NOLINT