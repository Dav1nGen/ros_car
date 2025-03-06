/**
 * @file acquistion_node.hpp
 * @author luoyebai (2112216825@qq.com)
 * @brief camera captures image node
 * @version 0.1
 * @date 2023-04-27
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

/**
 * @brief acquire image and publish it
 *
 */
class NavigationNode : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Navigation Node object
     *
     * @param options
     */
    NavigationNode() : Node("Navigation")
    {
        // set log level
        this->declare_parameter("rcl_log_level", 0);
        this->declare_parameter("is_debug", false);
        this->get_parameter("rcl_log_level", log_level_);
        this->get_logger().set_level((rclcpp::Logger::Level)log_level_);
        param_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        param_cb_handle_ = param_sub_->add_parameter_callback(
            "is_debug", [this](const rclcpp::Parameter& p) {});

        // publisher init
        RCLCPP_INFO(this->get_logger(), "acquisition node start");
        publisher_ =
            this->create_publisher<std_msgs::msg::String>("publish_image", 1);

        // timer binding callback function
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            std::bind(&NavigationNode::TimerCallBack, this));
    }

private:
    int log_level_;
    std::shared_ptr<rclcpp::ParameterEventHandler> param_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> param_cb_handle_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    /**
     * @brief callback function for publishing message
     *
     */
    void TimerCallBack();
};

void NavigationNode::TimerCallBack() {}
