
#ifndef MOTORNODE_H
#define MOTORNODE_H

#include <rclcpp/rclcpp.hpp>

// TODO: Enhancement - Make WHEEL_SLIP_THRESHOLD be a ROS param
#define WHEEL_SLIP_THRESHOLD  (0.08)   // Rotation below which we null excess wheel torque in 4wd drive_type

static const double BILLION = 1000000000.0;


class MotorNode : public rclcpp::Node{
public:
    MotorNode();
    
    void PID_update_callback(const ubiquity_motor::PIDConfig& config, uint32_t level);
    void SystemControlCallback(const std_msgs::String::ConstPtr& msg);
    void initMcbParameters();
    void run();

    int g_wheel_slip_nulling;

    std::unique_ptr<MotorHardware> robot;

    FirmwareParams g_firmware_params;
    CommsParams    g_serial_params;
    NodeParams     g_node_params;
};

#endif

