/**
 * @file detector.hpp
 * @author Dav1nGen (davicheng1114@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-05-26
 *
 * @copyright Copyright (c) 2024
 *
 */

#ifndef _ROOT_WORKSPACE_SRC_MYPKG_LINE_FOLLOW_INCLUDE_DETECTOR_HPP_
#define _ROOT_WORKSPACE_SRC_MYPKG_LINE_FOLLOW_INCLUDE_DETECTOR_HPP_

#include <algorithm>
#include <opencv4/opencv2/opencv.hpp>
#include <vector>

// 比较函数，用于按照轮廓的面积大小进行排序
bool compareContourAreas(const std::vector<cv::Point> &contour1,
                         const std::vector<cv::Point> &contour2)
{
    double area1 = cv::contourArea(contour1);
    double area2 = cv::contourArea(contour2);
    return area1 > area2;  // 降序排列
}
class LineDetector
{
public:
    LineDetector(cv::Mat src)
    {
        cv::Mat img =
            src(cv::Range(src.rows / 4, src.rows), cv::Range(0, src.cols));
        this->src_ = img.clone();
    }

    ~LineDetector() = default;

private:
    void parameterInit();       // 参数初始化函数
    cv::Mat imageProcessing();  // 图像处理函数
    cv::Point2f getTarget();    // 获取目标方向函数
    uint directControl();       // 巡线方向控制函数

    uint direction_correction_;  // 0方向正确 1左 2右 3无识别
    cv::Point2f car_direction_;  // 小车前进的方向
    cv::Point2f target_point_;   // 前进的目标点
    cv::Mat src_;
    cv::Mat binary_img_;

public:
    uint getResult();  // 获取结果函数
    cv::Mat debug_img  = cv::Mat::zeros(100, 100, CV_8UC3);
    cv::Mat result_img = cv::Mat::zeros(100, 100, CV_8UC3);
    float spin_coef    = 0.3;
};

void LineDetector::parameterInit()
{
    this->direction_correction_ = 3;
    this->target_point_.x       = 0;
    this->target_point_.y       = 0;
    this->car_direction_ =
        cv::Point2f(this->src_.cols / 2, this->src_.rows / 2);
}

cv::Mat LineDetector::imageProcessing()
{
    int src_w = src_.cols;
    int src_h = src_.rows;
    cv::Mat img =
        (this->src_.clone())(cv::Rect(0, src_h / 2, src_w, src_h / 2));
    // cv::Mat hsvImage;
    // cvtColor(img, hsvImage, cv::COLOR_BGR2HSV);

    // 定义黄色在HSV空间的范围
    // cv::Scalar lower_yellow = cv::Scalar(20, 100, 100);
    // cv::Scalar upper_yellow = cv::Scalar(100, 255, 255);

    //                                   B    G    R
    cv::Scalar lower_yellow = cv::Scalar(0, 100, 100);
    cv::Scalar upper_yellow = cv::Scalar(60, 200, 200);
    // 通过阈值化，提取黄色物体的二值图
    cv::Mat yellow_mask;
    inRange(img, lower_yellow, upper_yellow, yellow_mask);

    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    // 闭操作
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_DILATE, kernel);
    cv::cvtColor(yellow_mask, debug_img, cv::COLOR_GRAY2BGR);
    return yellow_mask;
}
cv::Point2f LineDetector::getTarget()
{
    std::vector<std::vector<cv::Point>> contours;
    std::vector<std::vector<cv::Point>> result_contours;
    cv::Mat binary_img  = this->binary_img_.clone();
    this->target_point_ = cv::Point2f(0, 0);

    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE, cv::Point(0, src_.rows / 2));

    // cv::Mat black_img = cv::Mat::zeros(this->src_.size(), CV_8UC3);
    cv::drawContours(src_, contours, -1, cv::Scalar(100, 100, 100), 1, 8);

    if (contours.empty()) return cv::Point2f(0, 0);

    for (const auto &contour : contours)
    {
        auto rect = minAreaRect(contour);

        // 计算矩形的长宽比
        float aspectRatio = std::max(rect.size.width / rect.size.height,
                                     rect.size.height / rect.size.width);
        float area        = rect.size.area();
        float angle       = rect.angle;
        // 设置长宽比的范围
        float minAspectRatio = 2;
        float maxAspectRatio = 10;

        bool ratio       = false;
        bool area_judge  = false;
        bool angle_judge = false;

        if (aspectRatio >= minAspectRatio && aspectRatio <= maxAspectRatio)
            ratio = true;
        if (area > 50) area_judge = true;

        if (angle >= 60 && angle <= 80) angle_judge = true;

        //	if (ratio && area_judge && angle_judge)
        if (area_judge && ratio) result_contours.push_back(contour);
    }
    if (result_contours.empty()) return cv::Point2f(0, 0);
    std::sort(result_contours.begin(), result_contours.end(),
              compareContourAreas);
    // 绘制轮廓
    cv::drawContours(src_, result_contours, 0, cv::Scalar(255, 255, 255), 1, 8);
    // 获取最小外接矩形的四个顶点
    auto rect = minAreaRect(result_contours[0]);
    cv::Point2f vertices[4];
    rect.points(vertices);
    // std::cout << rect.angle << "\n";

    // 绘制最小外接矩形
    for (int i = 0; i < 4; i++)
    {
        line(src_, vertices[i], vertices[(i + 1) % 4],
             cv::Scalar(255, 255, 255), 2);
    }
    this->result_img = src_;
    // 获取最小外接矩形的中心点坐标
    cv::Point2f center = rect.center;
    return center;
}
uint LineDetector::directControl()
{
    auto x_error = abs(this->car_direction_.x - this->target_point_.x);
    if (this->target_point_.x == 0 && this->target_point_.y == 0)
    {
        return 3;
    }

    // 向右修正
    if (this->car_direction_.x < this->target_point_.x && x_error > 5)
    {
        spin_coef = x_error / (src_.cols / 2.0);
        return 2;
        // std::cout << "right fix!\n";
    }

    // 向左修正
    if (this->car_direction_.x > this->target_point_.x && x_error > 5)
    {
        spin_coef = x_error / (src_.cols / 2.0);
        return 1;
        // std::cout << "left fix!\n";
    }
    return 0;
}

uint LineDetector::getResult()
{
    this->parameterInit();
    this->binary_img_           = this->imageProcessing();
    this->target_point_         = this->getTarget();
    this->direction_correction_ = this->directControl();
    return this->direction_correction_;
}
#endif  // _ROOT_WORKSPACE_SRC_MYPKG_LINE_FOLLOW_INCLUDE_DETECTOR_HPP_
