/**
 * @file follow.hpp
 * @author Dav1nGen (davicheng1114@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-05-26
 *
 * @copyright Copyright (c) 2024
 *
 */
#ifndef _ROOT_WORKSPACE_SRC_MYPKG_LINE_FOLLOW_INCLUDE_LINE_FOLLOW_FOLLOW_HPP_
#define _ROOT_WORKSPACE_SRC_MYPKG_LINE_FOLLOW_INCLUDE_LINE_FOLLOW_FOLLOW_HPP_

#include <cv_bridge/cv_bridge.h>

#include <geometry_msgs/msg/twist.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "detector/detector.hpp"

class Follow : public rclcpp::Node
{
public:
    explicit Follow(std::string name) : Node(name)
    {
        // 初始化图片为纯黑
        this->image_ = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));

        // 创建发布者
        this->publisher_ = this->create_publisher<Twist>("cmd_vel", 1);
        this->debug_publisher_ =
            this->create_publisher<Image>("debug_image", 1);
        this->result_publisher_ =
            this->create_publisher<Image>("result_image", 1);

        // 创建订阅者
        this->subscription_ = this->create_subscription<Image>(
            "publish_image", 1,
            std::bind(&Follow::subscriptionCallback, this,
                      std::placeholders::_1));

        //     创建定时器，定时发布消息
        //     this->timer_ = this->create_wall_timer(
        //         std::chrono::milliseconds(200),
        //         std::bind(&Follow::publisherCallback, this));
    }

    ~Follow()
    {
        this->cmdvel_.linear.x  = 0;
        this->cmdvel_.linear.y  = 0;
        this->cmdvel_.linear.z  = 0;
        this->cmdvel_.angular.x = 0;
        this->cmdvel_.angular.y = 0;
        this->cmdvel_.angular.z = 0;
        this->publisher_->publish(this->cmdvel_);
    }

private:
    using Image          = sensor_msgs::msg::Image;
    using Twist          = geometry_msgs::msg::Twist;
    using Publisher      = rclcpp::Publisher<Twist>;
    using ImagePublisher = rclcpp::Publisher<Image>;
    using Subscription   = rclcpp::Subscription<Image>;

    Image toImageMsg(const cv::Mat &src);

    Publisher::SharedPtr publisher_;
    ImagePublisher::SharedPtr debug_publisher_;
    ImagePublisher::SharedPtr result_publisher_;
    Subscription::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    Twist cmdvel_;
    cv::Mat image_;
    float front_speed_ = 0.6;
    float spin_speed_  = 0.8;
    float alpha        = 0.8;

public:
    void publisherCallback();
    void subscriptionCallback(const Image::SharedPtr msg);
    void initPublisher();
};

Follow::Image Follow::toImageMsg(const cv::Mat &src)
{
    Image img_msg;
    std_msgs::msg::Header img_header;
    cv_bridge::CvImage cv_bridge;
    img_header.stamp = this->get_clock()->now();
    cv_bridge =
        cv_bridge::CvImage(img_header, sensor_msgs::image_encodings::BGR8, src);
    cv_bridge.toImageMsg(img_msg);
    return img_msg;
}

// void Follow::publisherCallback() {
//     this->initPublisher();

//     LineDetector dector = LineDetector(this->image_);
//     uint result = dector.getResult();
//     cv::Mat debug_src = dector.getDebugImage();
//     cv::imwrite("debug.jpg", debug_src);

//     if (result == 0) {
//         this->cmdvel_.linear.x = 0.3;
//         RCLCPP_INFO(this->get_logger(), "go straight");
//     }

//     if (result == 1) {
//         this->cmdvel_.angular.z = 0.5;
//         this->cmdvel_.linear.x = 0.3;
//         RCLCPP_INFO(this->get_logger(), "turn left");
//     }

//     if (result == 2) {
//         this->cmdvel_.angular.z = -0.5;
//         this->cmdvel_.linear.x = 0.3;
//         RCLCPP_INFO(this->get_logger(), "turn right");
//     }
// subscription_cmdvel_);

//     publish message
//     Image debug_img_msg;
//     std_msgs::msg::Header debug_img_header;
//     cv_bridge::CvImage cv_bridge;
//     debug_img_header.stamp = this->get_clock()->now();
//     cv_bridge = cv_bridge::CvImage(
//         debug_img_header, sensor_msgs::image_encodings::BGR8, debug_src);
//     cv_bridge.toImageMsg(debug_img_msg);
//     // publish
//     this->debug_publisher_->publish(debug_img_msg);
// }

void Follow::subscriptionCallback(const Image::SharedPtr msg)
{
    this->image_ = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());
    cv::imwrite("image.jpg", this->image_);
    this->initPublisher();

    LineDetector dector = LineDetector(this->image_);
    uint result         = dector.getResult();

    if (result == 0)
    {
        this->cmdvel_.linear.x = this->front_speed_;
        RCLCPP_INFO(this->get_logger(), "go straight");
    }

    if (result == 1)
    {
        this->cmdvel_.angular.z = alpha * dector.spin_coef * this->spin_speed_;
        this->cmdvel_.linear.x  = this->front_speed_;
        RCLCPP_INFO(this->get_logger(), "turn left");
    }

    if (result == 2)
    {
        this->cmdvel_.angular.z = -alpha * dector.spin_coef * this->spin_speed_;
        this->cmdvel_.linear.x  = this->front_speed_;
        RCLCPP_INFO(this->get_logger(), "turn right");
    }

    if (result == 3)
    {
        this->cmdvel_.angular.z = spin_speed_;
        RCLCPP_INFO(this->get_logger(), "spin");
    }

    this->publisher_->publish(this->cmdvel_);
    auto debug_img_msg  = toImageMsg(dector.debug_img);
    auto result_img_msg = toImageMsg(dector.result_img);
    // publish
    this->debug_publisher_->publish(debug_img_msg);
    this->result_publisher_->publish(result_img_msg);
}

void Follow::initPublisher()
{
    this->cmdvel_.linear.x  = 0;
    this->cmdvel_.linear.y  = 0;
    this->cmdvel_.linear.z  = 0;
    this->cmdvel_.angular.x = 0;
    this->cmdvel_.angular.y = 0;
    this->cmdvel_.angular.z = 0;
}

#endif  // _ROOT_WORKSPACE_SRC_MYPKG_LINE_FOLLOW_INCLUDE_LINE_FOLLOW_FOLLOW_HPP_
