
#ifndef MOTORNODE_H
#define MOTORNODE_H

#include <rclcpp/rclcpp.hpp>
// #include <ubiquity_motor/PIDConfig.h>
#include <std_msgs/msg/string.hpp>
#include <ubiquity_motor_ros2/motor_parameters.h>
#include <ubiquity_motor_ros2/motor_hardware.h>
#include <ubiquity_motor_ros2/motor_message.h>
// #include <mutex>

using namespace ubiquity_motor_ros2;

static const double BILLION = 1000000000.0;


class MotorNode : public rclcpp::Node{
public:
    MotorNode();
    
    // void PID_update_callback(const ubiquity_motor::PIDConfig& config, uint32_t level);
    // void SystemControlCallback(const std_msgs::msg::String::SharedPtr msg);
    // void initMcbParameters();
    hardware_interface::HardwareInfo getHwInfo();
    void run();

    int g_wheel_slip_nulling;

    std::unique_ptr<MotorHardware> robot;

    // std::mutex node_mutex;
};

#endif

