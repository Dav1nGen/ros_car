#pragma once
#define TEST 1
#ifdef TEST
// std
#    include <string>
#    include <vector>
#    include <cmath>
// ros
#    include <rclcpp/rclcpp.hpp>
#    include <tf2/LinearMath/Quaternion.h>
#    include <tf2_ros/transform_broadcaster.h>
#    include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#    include <nav_msgs/msg/odometry.hpp>
#    include <std_msgs/msg/int32.hpp>
#    include <std_msgs/msg/int16.hpp>
#    include <std_msgs/msg/u_int16.hpp>
#    include <std_msgs/msg/float32.hpp>
#    include <sensor_msgs/msg/imu.hpp>
#    include <sensor_msgs/msg/range.hpp>
#    include <geometry_msgs/msg/twist.hpp>
#    include <geometry_msgs/msg/transform_stamped.hpp>
// boost
#    include <boost/asio.hpp>
#    include <boost/asio/serial_port.hpp>
#    include <boost/system/error_code.hpp>
#    include <boost/system/system_error.hpp>
#    include <boost/bind.hpp>
#    include <boost/thread.hpp>
// interfaces
#    include "jubot_driver_interfaces/msg/pid_reconfig.hpp"

#    define G 9.8
#    define head1 0xAA
#    define head2 0x55
#    define sendType_velocity 0x11
#    define sendType_pid 0x12
#    define sendType_params 0x13
#    define sendType_wheelspeed 0x14

#    define foundType_Packages 0x06

enum packetFinderState {
    waitingForHead1,
    waitingForHead2,
    waitingForPayloadSize,
    waitingForPayloadType,
    waitingForPayload,
    waitingForCheckSum,
    handlePayload
};

struct pid_param {
    int kp;
    int ki;
    int kd;
};

struct imu_data {
    float angle_x;
    float angle_y;
    float angle_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float q0;
    float q1;
    float q2;
    float q3;
};

typedef boost::shared_ptr<boost::asio::serial_port> serialp_ptr;

class JubotDriver : public rclcpp::Node {
 public:
    JubotDriver();
    ~JubotDriver();
    void SetVelocity(double x, double y, double yaw);
    void loop();

 private:
    bool initRobot();

    void recv_msg();

    void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstPtr& msg);
    // void send_speed_callback(const rclcpp::Time&);
    void send_speed_callback();
    // void dynamic_reconfig_callback(
    //     jubot_driver_interfaces::msg::PIDReconfig& config, uint32_t level);

    void handle_base_data(const uint8_t* buffer_data);
    void SetPID(int p, int i, int d);
    void SetParams(double linear_correction, double angular_correction);

    void check_sum(uint8_t* data, size_t len, uint8_t& dest);
    void distribute_data(uint8_t msg_type, uint8_t* buffer_data);
    void upload_pid_param();

    uint8_t msg_seq_;
    packetFinderState state_;
    bool first_init_;

    std_msgs::msg::Float32 battery_pub_data_;

    boost::mutex cmd_vel_mutex_;
    boost::system::error_code ec_;
    boost::asio::io_service io_service_;
    boost::mutex mutex_;
    serialp_ptr sp_;

    bool recv_flag_;
    bool publish_odom_transform_;
    bool start_flag_;

    geometry_msgs::msg::Twist current_twist_;
    nav_msgs::msg::Odometry odom_;
    geometry_msgs::msg::TransformStamped transformStamped_;
    tf2_ros::TransformBroadcaster br_;
    tf2::Quaternion quaternion_;

    rclcpp::Clock clock_;
    rclcpp::Time now_;
    rclcpp::Time last_time_;
    rclcpp::Time last_twist_time_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr avel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cvel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dvel_pub_;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr aset_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bset_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr cset_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr dset_pub_;

    std_msgs::msg::Int32 avel_pub_data_;
    std_msgs::msg::Int32 bvel_pub_data_;
    std_msgs::msg::Int32 cvel_pub_data_;
    std_msgs::msg::Int32 dvel_pub_data_;

    std_msgs::msg::Int32 aset_pub_data_;
    std_msgs::msg::Int32 bset_pub_data_;
    std_msgs::msg::Int32 cset_pub_data_;
    std_msgs::msg::Int32 dset_pub_data_;
    sensor_msgs::msg::Imu imu_pub_data_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

    std::string port_name_;

    int baud_rate_;

    std::string odom_frame_;
    std::string imu_frame_;
    std::string base_frame_;
    std::string code_version_;
    int control_rate_;

    std::vector<double> imu_list_;
    std::vector<double> odom_list_;
    std::vector<int> wheelspeedSet_list_;
    std::vector<int> wheelspeedGet_list_;

    double wheel_track_;
    double wheel_diameter_;

    double linear_correction_factor_;
    double angular_correction_factor_;

    int kp_;
    int ki_;
    int kd_;
};

#else
// ros
#    include <rclcpp/rclcpp.hpp>
#    include <tf2/LinearMath/Quaternion.h>
#    include <tf2_ros/transform_broadcaster.h>
#    include <nav_msgs/msg/odometry.hpp>
#    include <std_msgs/msg/int32.hpp>
#    include <std_msgs/msg/int16.hpp>
#    include <std_msgs/msg/u_int16.hpp>
#    include <std_msgs/msg/float32.hpp>
#    include <sensor_msgs/msg/imu.hpp>
#    include <sensor_msgs/msg/range.hpp>
#    include <geometry_msgs/msg/twist.hpp>
#    include <geometry_msgs/msg/transform_stamped.hpp>

#    include <dynamic_reconfigure/server.h>
#    include <jubot_driver/PID_reconfigConfig.h>
// std
#    include <string>
#    include <vector>
#    include <math.h>
// boost
#    include <boost/asio.hpp>
#    include <boost/asio/serial_port.hpp>
#    include <boost/system/error_code.hpp>
#    include <boost/system/system_error.hpp>
#    include <boost/bind.hpp>
#    include <boost/thread.hpp>

#    define G 9.8
#    define head1 0xAA
#    define head2 0x55
#    define sendType_velocity 0x11
#    define sendType_pid 0x12
#    define sendType_params 0x13
#    define sendType_wheelspeed 0x14

#    define foundType_Packages 0x06

//#define MAX_STEERING_ANGLE    0.87
//#define M_PI 3.1415926535

enum packetFinderState {
    waitingForHead1,
    waitingForHead2,
    waitingForPayloadSize,
    waitingForPayloadType,
    waitingForPayload,
    waitingForCheckSum,
    handlePayload
};

struct pid_param {
    int kp;
    int ki;
    int kd;
};

struct imu_data {
    float angle_x;
    float angle_y;
    float angle_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float q0;
    float q1;
    float q2;
    float q3;
};

typedef boost::shared_ptr<boost::asio::serial_port> serialp_ptr;

class JubotDriver {
 public:
    JubotDriver();
    ~JubotDriver();
    void loop();

 private:
    bool initRobot();

    void recv_msg();

    void cmd_vel_callback(const geometry_msgs::msg::Twist::ConstPtr& msg);
    void send_speed_callback(const rclcpp::TimerBase&);
    void dynamic_reconfig_callback(jubot_driver::PID_reconfigConfig& config,
                                   uint32_t level);

    void handle_base_data(const uint8_t* buffer_data);
    void SetPID(int p, int i, int d);
    void SetParams(double linear_correction, double angular_correction);
    void SetVelocity(double x, double y, double yaw);

    void check_sum(uint8_t* data, size_t len, uint8_t& dest);
    void distribute_data(uint8_t msg_type, uint8_t* buffer_data);
    void upload_pid_param();

    packetFinderState state_;

    std_msgs::Float32 battery_pub_data_;

    boost::mutex cmd_vel_mutex_;
    boost::system::error_code ec_;
    boost::asio::io_service io_service_;
    boost::mutex mutex_;
    serialp_ptr sp_;

    bool recv_flag_;
    bool publish_odom_transform_;
    bool start_flag_;
    bool first_init_;
    uint8_t msg_seq_;

    geometry_msgs::Twist current_twist_;
    nav_msgs::Odometry odom_;
    geometry_msgs::TransformStamped transformStamped_;
    tf2_ros::TransformBroadcaster br_;

    ros::Time now_;
    ros::Time last_time_;
    ros::Time last_twist_time_;

    ros::Publisher odom_pub_;
    ros::Publisher battery_pub_;
    ros::Publisher imu_pub_;

    ros::Publisher avel_pub_;
    ros::Publisher bvel_pub_;
    ros::Publisher cvel_pub_;
    ros::Publisher dvel_pub_;

    ros::Publisher aset_pub_;
    ros::Publisher bset_pub_;
    ros::Publisher cset_pub_;
    ros::Publisher dset_pub_;

    std_msgs::msg::Int32 avel_pub_data_;
    std_msgs::msg::Int32 bvel_pub_data_;
    std_msgs::msg::Int32 cvel_pub_data_;
    std_msgs::msg::Int32 dvel_pub_data_;

    std_msgs::msg::Int32 aset_pub_data_;
    std_msgs::msg::Int32 bset_pub_data_;
    std_msgs::msg::Int32 cset_pub_data_;
    std_msgs::msg::Int32 dset_pub_data_;
    sensor_msgs::msg::Imu imu_pub_data_;

    ros::Subscriber cmd_sub_;

    std::string port_name_;

    int baud_rate_;

    std::string odom_frame_;
    std::string imu_frame_;
    std::string base_frame_;
    std::string code_version_;
    int control_rate_;

    std::vector<double> imu_list_;
    std::vector<double> odom_list_;
    std::vector<int> wheelspeedSet_list_;
    std::vector<int> wheelspeedGet_list_;

    double wheel_track_;
    double wheel_diameter_;

    double linear_correction_factor_;
    double angular_correction_factor_;

    int kp_;
    int ki_;
    int kd_;
};

#endif  // DEBUG