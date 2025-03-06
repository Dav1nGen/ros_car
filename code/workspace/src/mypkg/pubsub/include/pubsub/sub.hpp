#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::placeholders;

class Sub : public rclcpp::Node
{
public:
    explicit Sub(const std::string &name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s开始运行", name.c_str());
        auto callback_f = std::bind(&Sub::listenHelloCallBack, this, _1);
        sub_ptr_        = this->create_subscription<std_msgs::msg::String>(
            "/topic1", 1, callback_f);
    }
    ~Sub() = default;

private:
    void listenHelloCallBack(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "接收消息:'%s' ", msg->data.c_str());
        return;
    }
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_ptr_;
};