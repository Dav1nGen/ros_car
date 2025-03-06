/**
 * @file obstacle_avoidance.hpp
 * @author Dav1nGen (davicheng1114@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-05-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef _ROOT_WORKSPACE_SRC_MYPKG_OBSTACLE_AVOIDANCE_INCLUDE_OBSTACLE_AVOIDANCE_HPP_
#define _ROOT_WORKSPACE_SRC_MYPKG_OBSTACLE_AVOIDANCE_INCLUDE_OBSTACLE_AVOIDANCE_HPP_
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <mypkg_interfaces/msg/move_data.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using MoveData = mypkg_interfaces::msg::MoveData;

class Avoidancer : public rclcpp::Node
{
public:
    explicit Avoidancer(const std::string &name) : Node(name)
    {
        // 避障器接收雷达信息
        auto avoidancer_sub_callback = std::bind(
            &Avoidancer::avoidancerSubCallback, this, std::placeholders::_1);

        // 创建雷达订阅者
        avoidancer_sub_ptr_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "scan", 1, avoidancer_sub_callback);

        // 收集qr识别信息
        auto qr_sub_callback =
            std::bind(&Avoidancer::qrSubCallback, this, std::placeholders::_1);

        // 创建qr订阅者
        qr_sub_ptr_ = this->create_subscription<std_msgs::msg::Bool>(
            "qr", 1, qr_sub_callback);

        // 创建发布者
        avoidancer_pub_ptr_ = this->create_publisher<MoveData>("move", 1);

        // 定时器定时发布消息
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Avoidancer::avoidancerPubCallback, this));
    }

    ~Avoidancer()
    {
        this->move_data_.direction = 'p';

        this->avoidancer_pub_ptr_->publish(this->move_data_);
    }

private:
    MoveData move_data_{};

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        avoidancer_sub_ptr_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr qr_sub_ptr_;
    rclcpp::Publisher<mypkg_interfaces::msg::MoveData>::SharedPtr
        avoidancer_pub_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;  // 发布者发布的定时器

    float front_distance_;  // 车前方障碍物的距离
    float back_distance_;   // 车后方障碍物的距离
    float min_front_distance_ = 16.0f;
    float is_front_know_      = false;
    bool turn_direction;  // 车避障时旋转的方向

    bool qr_state = false;  // 是否识别到qr码

    // 避障器订阅者回调函数
    void avoidancerSubCallback(
        const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
    // qr订阅者回调函数
    void qrSubCallback(const std_msgs::msg::Bool msg);
    // 避障器发布者回调函数
    void avoidancerPubCallback();

public:
    // 设置移动函数
    void setMoveData();
    // 获取避障方向
    void getDirection(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);

    // 获取前后平均距离
    void getDistance(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg);
};

void Avoidancer::avoidancerSubCallback(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    this->getDistance(msg);
    // this->front_distance_ = msg->ranges[0];
    // this->back_distance_ = msg->ranges[359];
    this->getDirection(msg);
}

void Avoidancer::qrSubCallback(const std_msgs::msg::Bool msg)
{
    this->qr_state = msg.data;
}

void Avoidancer::getDirection(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    uint count_left             = 0;
    uint count_right            = 0;
    float total_distance_left   = 0;
    float total_distance_right  = 0;
    float average_distnce_left  = 0;
    float average_distnce_right = 0;

    for (int i = 0; i < 90; i++)
    {
        if (!std::isinf(msg->ranges.data()[i]))
        {
            count_left++;
            total_distance_left += msg->ranges.data()[i];
        }
        if (!std::isinf(msg->ranges.data()[719 - i]))
        {
            count_right++;
            total_distance_right += msg->ranges.data()[719 - i];
        }
    }

    average_distnce_left  = total_distance_left / count_left;
    average_distnce_right = total_distance_right / count_right;

    this->turn_direction = average_distnce_left > average_distnce_right;
}

void Avoidancer::getDistance(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
    float total_front_distance = 0;
    float total_back_distance  = 0;
    float count_front          = 0;
    float count_back           = 0;

    for (int i = 0; i < 25; i++)
    {
        if (!std::isinf(msg->ranges.data()[i]))
        {
            if (min_front_distance_ > msg->ranges.data()[i])
                min_front_distance_ = msg->ranges.data()[i];
            is_front_know_ = true;
            total_front_distance += msg->ranges.data()[i];
            count_front++;
        }
        if (!std::isinf(msg->ranges.data()[719 - i]))
        {
            total_front_distance += msg->ranges.data()[719 - i];
            count_front++;
        }
        if (!std::isinf(msg->ranges.data()[359 + i]))
        {
            total_back_distance += msg->ranges.data()[359 + i];
            count_back++;
        }
        if (!std::isinf(msg->ranges.data()[359 - i]))
        {
            total_back_distance += msg->ranges.data()[359 - i];
            count_back++;
        }
    }

    this->front_distance_ = total_front_distance / count_front;
    this->back_distance_  = total_back_distance / count_back;
}

void Avoidancer::avoidancerPubCallback()
{
    this->setMoveData();
    // 当没有识别到qr码时发送避障移动消息
    if (!qr_state.data) this->avoidancer_pub_ptr_->publish(this->move_data_);
}

void Avoidancer::setMoveData()
{
    // 如果前方小于0.5，且后方大于0.5则后退
    //    if (this->front_distance_ < 0.5 && this->back_distance_ > 0.5) {
    //        this->mode_msg_.data = "back";
    //        RCLCPP_INFO(this->get_logger(), "mode: %s",
    //                    this->mode_msg_.data.c_str());
    //        return;
    //    }

    // 如果前方小于0.5，且后方小于0.5则停止
    //    if (this->front_distance_ < 0.5 && this->back_distance_ < 0.5) {
    //        this->mode_msg_.data = "stop";
    //        RCLCPP_INFO(this->get_logger(), "mode: %s",
    //                    this->mode_msg_.data.c_str());
    //        return;
    //    }

    //    if (this->front_distance_ < 0.5) {

    // 初始化移动信息
    this->move_data_.direction = 't';
    this->move_data_.parameter = 0;

    // 如果前方小于0.6则根据两边的障碍物距离转向
    if (this->min_front_distance_ < 0.6 || !is_front_know_)
        this->move_data_.direction = this->turn_direction ? 'a' : 'd';
    // 都不满足则前进
    else
        this->move_data_.direction = 'w';
    min_front_distance_ = 16.0f;
    is_front_know_      = false;
    RCLCPP_INFO(this->get_logger(), "mode: %c", this->move_data_.direction);

    // 根据移动的方向设置移动参数
    if (this->move_data_.direction == 'w') this->move_data_.parameter = 0.3;

    if (this->move_data_.direction == 'a') this->move_data_.parameter = 1;

    if (this->move_data_.direction == 'd') this->move_data_.parameter = -1;

    if (this->move_data_.direction == 's') this->move_data_.parameter = -0.3;
}

// void Avoidancer::setPubMessage()
// {
//     this->move_data_.parameter = 0;

// 初始化发布的信息, 默认状态为停止
// this->move_data_.linear.x = 0;
// this->move_data_.linear.y = 0;
// this->move_data_.linear.z = 0;
// this->move_data_.angular.x = 0;
// this->move_data_.angular.y = 0;
// this->move_data_.angular.z = 0;

// if (this->mode_msg_.data == "forward")
// {
//     this->move_data_.linear.x = 0.3;
// }

// if (this->mode_msg_.data == "left")
// {
//     this->move_data_.angular.z = 1;
// }

// if (this->mode_msg_.data == "right")
// {
//     this->move_data_.angular.z = -1;
// }

// if (this->mode_msg_.data == "back")
// {
//     this->move_data_.linear.x = -0.3;
// }
// }

#endif  // _ROOT_WORKSPACE_SRC_MYPKG_OBSTACLE_AVOIDANCE_INCLUDE_OBSTACLE_AVOIDANCE_HPP_
