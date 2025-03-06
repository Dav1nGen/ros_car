// Copyright[2024] <davi>
#ifndef MYPKG_OBSTACLE_AVOIDANCE_INCLUDE_OBSTACLE_AVOIDANCE_HPP_
#    define MYPKG_OBSTACLE_AVOIDANCE_INCLUDE_BOSTACLE_AVOIDANCE_HPP_
#    include <cmath>
#    include <geometry_msgs/msg/twist.hpp>
#    include <iostream>
#    include <mypkg_interfaces/msg/move_data.hpp>
#    include <rclcpp/rclcpp.hpp>
#    include <std_msgs/msg/string.hpp>
#    include <string>

using MoveData = mypkg_interfaces::msg::MoveData;

class WalkController : public rclcpp::Node
{
public:
    explicit WalkController(const std::string &name) : Node(name)
    {
        // 接收移动信息
        auto avoidancer_sub_callback =
            std::bind(&WalkController::walkControllerSubCallback, this,
                      std::placeholders::_1);

        // 创建订阅者
        walk_controller_sub_ptr_ = this->create_subscription<MoveData>(
            "move", 1, avoidancer_sub_callback);

        // 创建发布者
        walk_controller_pub_ptr_ =
            this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

        // 定时器定时发布消息
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&WalkController::walkControllerPubCallback, this));
    }

    ~WalkController()
    {
        this->cmd_vel_.linear.x  = 0;
        this->cmd_vel_.linear.y  = 0;
        this->cmd_vel_.linear.z  = 0;
        this->cmd_vel_.angular.x = 0;
        this->cmd_vel_.angular.y = 0;
        this->cmd_vel_.angular.z = 0;
        this->walk_controller_pub_ptr_->publish(this->cmd_vel_);
    }

private:
    uint8_t direction;
    int64_t parameter;
    geometry_msgs::msg::Twist cmd_vel_{};

    rclcpp::Subscription<MoveData>::SharedPtr walk_controller_sub_ptr_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        walk_controller_pub_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;  // 发布者发布的定时器

    // 避障器订阅者回调函数
    void walkControllerSubCallback(const MoveData msg);
    // 避障器发布者回调函数
    void walkControllerPubCallback();

public:
    // 发布信息函数
    void setPubMessage();
};

void WalkController::walkControllerSubCallback(const MoveData msg)
{
    this->direction = msg.direction;
    this->parameter = msg.parameter;
}

void WalkController::walkControllerPubCallback()
{
    this->setPubMessage();
    this->walk_controller_pub_ptr_->publish(this->cmd_vel_);
}

void WalkController::setPubMessage()
{
    // 初始化发布的信息, 默认状态为停止
    this->cmd_vel_.linear.x  = 0;
    this->cmd_vel_.linear.y  = 0;
    this->cmd_vel_.linear.z  = 0;
    this->cmd_vel_.angular.x = 0;
    this->cmd_vel_.angular.y = 0;
    this->cmd_vel_.angular.z = 0;

    if (this->direction == 'w')
    {
        this->cmd_vel_.linear.x = this->parameter;
    }

    if (this->direction == 'a')
    {
        this->cmd_vel_.angular.z = this->parameter;
    }

    if (this->direction == 'd')
    {
        this->cmd_vel_.angular.z = this->parameter;
    }

    if (this->direction == 's')
    {
        this->cmd_vel_.linear.x = this->parameter;
    }
}

#endif  // MYPKG_OBSTACLE_AVOIDANCE_INCLUDE_OBSTACLE_AVOIDANCE_HPP_
