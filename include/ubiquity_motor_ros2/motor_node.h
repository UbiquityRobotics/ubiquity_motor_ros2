
#ifndef MOTORNODE_H
#define MOTORNODE_H

#include <rclcpp/rclcpp.hpp>
// #include <ubiquity_motor/PIDConfig.h>
#include <std_msgs/msg/string.hpp>
#include <ubiquity_motor_ros2/motor_parameters.h>
#include <ubiquity_motor_ros2/motor_hardware.h>
#include <ubiquity_motor_ros2/motor_message.h>



static const double BILLION = 1000000000.0;


class MotorNode : public rclcpp::Node{
public:
    MotorNode();
    
    // void PID_update_callback(const ubiquity_motor::PIDConfig& config, uint32_t level);
    void SystemControlCallback(const std_msgs::msg::String::SharedPtr msg);
    // void initMcbParameters();
    hardware_interface::HardwareInfo getHwInfo();
    void run();

    int g_wheel_slip_nulling;

    std::unique_ptr<MotorHardware> robot;

    // FirmwareParams g_firmware_params;
    // CommsParams    g_serial_params;
    // NodeParams     g_node_params;

    std::shared_ptr<NodeParams> node_params;
    std::shared_ptr<CommsParams> serial_params;
    std::shared_ptr<FirmwareParams> firmware_params;
};

#endif

