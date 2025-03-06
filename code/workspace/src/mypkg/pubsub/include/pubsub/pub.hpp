#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class Pub : public rclcpp::Node
{
public:
    explicit Pub(const std::string& name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "%s开始运行", name.c_str());

        pub_ptr_ = this->create_publisher<std_msgs::msg::String>("/topic1", 1);

        auto callback_f = std::bind(&Pub::sayHelloCallBack, this);
        timer_ptr_      = this->create_wall_timer(500ms, callback_f);
    }

    ~Pub() = default;

private:
    void sayHelloCallBack()
    {
        std_msgs::msg::String msg;
        msg.data = "hello " + std::to_string(++count_);
        if (count_ == 10)
        {
            msg.data = "kill";
            pub_ptr_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "发布消息:'%s' ", msg.data.c_str());
            exit(0);
        }
        pub_ptr_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "发布消息:'%s' ", msg.data.c_str());
        return;
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_ptr_;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    size_t count_ = 0;
};