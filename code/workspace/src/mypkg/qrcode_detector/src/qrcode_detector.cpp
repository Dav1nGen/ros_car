#include "qrcode_detector.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<QrCodeDetector>("qrcode_detector");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}