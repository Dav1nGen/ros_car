#ifndef QRCODE_DETECTOR_INCLUDE_QRCODE_DETECTOR_HPP_
#define QRCODE_DETECTOR_INCLUDE_QRCODE_DETECTOR_HPP_

#include <cassert>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <mypkg_interfaces/msg/move_data.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <vector>

class QrCodeDetector : public rclcpp::Node
{
public:
    explicit QrCodeDetector(std::string name) : Node(name)
    {
        this->getParameter();
        this->initMsgs();

        // this->image_ =
        // cv::imread("/home/made4/code/workspace/src/mypkg/qrcode_detector/img/1.jpg");
        // assert(!image_.empty() && "image empty!");
        // this->pnpSolveQrcode(this->image_);
        // this->image_center_point_ = cv::Point2f(this->image_.cols / 2,
        // this->image_.rows / 2);

        //  创建发布者
        move_data_publisher_ptr_ = this->create_publisher<
            mypkg_interfaces::msg::MoveData>(
            "move",
            1);  //  发布者对象指针,用于在定时器中发布基于二维码位置的移动指令
        this->move_data_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&QrCodeDetector::pubMoveDataCallback, this));

        found_qrcode_publisher_ptr_ =
            this->create_publisher<std_msgs::msg::Bool>(
                "qr",
                1);  //  发布者对象指针,用于在定时器中发布是否识别到二维码bool值
        this->found_qrcode_publisher_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&QrCodeDetector::pubFoundQrcodeCallback, this));

        // 创建订阅者
        this->subscription_ptr_ =
            this->create_subscription<sensor_msgs::msg::Image>(
                "publish_image", 1,
                std::bind(&QrCodeDetector::subscriptionCallback, this,
                          std::placeholders::_1));
    }

    ~QrCodeDetector()
    {
        this->found_qrcode_.data   = false;
        this->move_data_.direction = 0;  //  0直行 1左转 2右转
        this->move_data_.parameter = 0;
    };

private:
    std_msgs::msg::Bool found_qrcode_;
    mypkg_interfaces::msg::MoveData move_data_;
    cv::Mat image_;
    cv::Mat rvec_, tvec_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_ptr_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
        found_qrcode_publisher_ptr_;
    rclcpp::Publisher<mypkg_interfaces::msg::MoveData>::SharedPtr
        move_data_publisher_ptr_;
    rclcpp::TimerBase::SharedPtr found_qrcode_publisher_timer_;
    rclcpp::TimerBase::SharedPtr move_data_publisher_timer_;

    //  二维码中心坐标
    cv::Point2f qrcode_center_point_;
    //  图像中心坐标
    cv::Point2f image_center_point_;
    //  小车与二维码距离
    double distance_to_qrcode_;  //  mm

    //  相机参数
    cv::Mat camera_matrix_data_;
    cv::Mat dist_coeffs_data_;
    std::vector<cv::Point3f> object_points_;
    std::vector<cv::Point2f> image_points_;

    //  初始化消息
    void getParameter();
    // pnp解算Qrcode
    void pnpSolveQrcode(cv::Mat image);

    void initMsgs();
    void subscriptionCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void pubFoundQrcodeCallback();
    void pubMoveDataCallback();

public:
};

void QrCodeDetector::initMsgs()
{
    this->distance_to_qrcode_  = 0;
    this->found_qrcode_.data   = false;
    this->move_data_.direction = 'w';
    this->move_data_.parameter = 0;
}

void QrCodeDetector::getParameter()
{
    auto qrcode_size = this->declare_parameter("qrcode_size", 150);
    this->camera_matrix_data_ =
        cv::Mat(
            3, 3, CV_64F,
            const_cast<double *>(
                this->declare_parameter("camera_matrix", std::vector<double>())
                    .data()))
            .clone();
    this->dist_coeffs_data_ =
        cv::Mat(
            1, 5, CV_64F,
            const_cast<double *>(
                this->declare_parameter("dist_coeffs", std::vector<double>())
                    .data()))
            .clone();
    //  设置棋盘格角点
    this->object_points_.push_back(
        cv::Point3f(0, qrcode_size / 2, -qrcode_size / 2));
    this->object_points_.push_back(
        cv::Point3f(0, qrcode_size / 2, qrcode_size / 2));
    this->object_points_.push_back(
        cv::Point3f(0, -qrcode_size / 2, qrcode_size / 2));
    this->object_points_.push_back(
        cv::Point3f(0, -qrcode_size / 2, -qrcode_size / 2));

    std::stringstream ss;
    ss << "qrcode size:" << qrcode_size << "mm\n"
       << "Camera matrix: " << this->camera_matrix_data_
       << "\nDist coeffs: " << this->dist_coeffs_data_;
    RCLCPP_DEBUG(this->get_logger(), ss.str().c_str());
}

void QrCodeDetector::pnpSolveQrcode(cv::Mat image)
{
    cv::QRCodeDetector detector;
    bool detect_success = detector.detect(image, this->image_points_);
    if (!detect_success)
    {
        this->initMsgs();
        RCLCPP_INFO(this->get_logger(), "qrcode not found!");
        return;
    }
    this->qrcode_center_point_ =
        cv::Point2f((this->image_points_[0].x + this->image_points_[2].x) / 2,
                    (this->image_points_[0].y + this->image_points_[2].y) / 2);
    this->image_center_point_ =
        cv::Point2f(this->image_.cols / 2, this->image_.rows / 2);
    // cv::drawChessboardCorners(image, cv::Size(2, 2), this->image_points_,
    // true); cv::imshow("image", image); cv::waitKey(0);
    //  EPnP解算
    bool solve =
        cv::solvePnP(this->object_points_, this->image_points_,
                     this->camera_matrix_data_, this->dist_coeffs_data_,
                     this->rvec_, this->tvec_, false, cv::SOLVEPNP_EPNP);
    //  打印解算结果
    if (solve)
    {
        RCLCPP_INFO(this->get_logger(), "Solve success");
        this->found_qrcode_.data =
            true;  //  解算成功才是真成功！！然后继续进行下一步
        // 计算相机与目标的距离
        double distance           = cv::norm(this->tvec_);
        this->distance_to_qrcode_ = distance;
        //  打印解算结果
        std::stringstream ss;
        ss << "Rvec: " << this->rvec_ << "\nTvec: " << this->tvec_
           << "\nDistance: " << distance << "mm"
           << "\npoint1:" << image_points_[0] << "\npoint2:" << image_points_[1]
           << "\npoint3:" << image_points_[2] << "\npoint4:" << image_points_[3]
           << "\nqrcode_center_point_:" << this->qrcode_center_point_;
        RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    }
}

void QrCodeDetector::subscriptionCallback(
    const sensor_msgs::msg::Image::SharedPtr msg)
{
    //  将订阅者中的数据赋值给image_
    this->image_ = cv::Mat(msg->height, msg->width, CV_8UC3, msg->data.data());
    assert(!image_.empty() && "image empty!");
    this->pnpSolveQrcode(this->image_);
}

void QrCodeDetector::pubFoundQrcodeCallback()
{
    this->found_qrcode_publisher_ptr_->publish(this->found_qrcode_);
}

void QrCodeDetector::pubMoveDataCallback()
{
    //  0直行 1左转 2右转 t停止
    if (!this->found_qrcode_.data) return;

    //  与二维码距离小于100mm，停止
    if (this->distance_to_qrcode_ < 100)
    {
        this->move_data_.direction = 't';
        this->move_data_.parameter = 0;
        this->move_data_publisher_ptr_->publish(this->move_data_);
        return;
    }

    //  二维码中心x坐标小于图像中心坐标，左转
    if (this->qrcode_center_point_.x < this->image_center_point_.x &&
        std::abs(this->qrcode_center_point_.x - this->image_center_point_.x) >
            50)
    {
        this->move_data_.direction = 'a';
        this->move_data_.parameter = 1;
        this->move_data_publisher_ptr_->publish(this->move_data_);
        return;
    }

    //  二维码中心x坐标大于图像中心坐标，右转
    if (this->qrcode_center_point_.x > this->image_center_point_.x &&
        std::abs(this->qrcode_center_point_.x - this->image_center_point_.x) >
            50)
    {
        this->move_data_.direction = 'd';
        this->move_data_.parameter = 1;
        this->move_data_publisher_ptr_->publish(this->move_data_);
        return;
    }

    // 二维码中心坐标与图像中心坐标差值小于50个像素点，直行
    if (this->qrcode_center_point_.x < this->image_center_point_.x &&
        std::abs(this->qrcode_center_point_.x - this->image_center_point_.x) <=
            50)
    {
        this->move_data_.direction = 'w';
        this->move_data_.parameter = 1;
        this->move_data_publisher_ptr_->publish(this->move_data_);
        return;
    }
}

#endif  // QRCODE_DETECTOR_INCLUDE_QRCODE_DETECTOR_HPP_