#ifndef MOTOR_HARDWARE_H
#define MOTOR_HARDWARE_H

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "motor_parameters.h"

class MotorHardware : public hardware_interface::SystemInterface, public rclcpp_lifecycle::LifecycleNode {
public:
    MotorHardware(const rclcpp::NodeOptions &options);
    virtual ~MotorHardware();

    hardware_interface::return_type configure(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type start() override;
    hardware_interface::return_type stop() override;
    hardware_interface::return_type read() override;
    hardware_interface::return_type write() override;

    void closePort();
    bool openPort();
    void clearCommands();
    void readInputs(uint32_t index);
    void writeSpeeds();
    void writeSpeedsInRadians(double left_radians, double right_radians);
    void publishFirmwareInfo();
    float calculateBatteryPercentage(float voltage, int cells, const float* type);
    int areWheelSpeedsLower(double wheelSpeedRadPerSec);
    void requestFirmwareVersion();
    void requestFirmwareDate();
    void setParams(FirmwareParams firmware_params);
    void sendParams();
    void forcePidParamUpdates();
    float getBatteryVoltage();
    void setDeadmanTimer(int32_t deadman);
    void setDeadzoneEnable(int32_t deadzone_enable);
    void setDebugLeds(bool led1, bool led2);
    void setHardwareVersion(int32_t hardware_version);
    void setEstopPidThreshold(int32_t estop_pid_threshold);
    void setEstopDetection(int32_t estop_detection);
    bool getEstopState();
    void setMaxFwdSpeed(int32_t max_speed_fwd);
    void setMaxRevSpeed(int32_t max_speed_rev);
    void setMaxPwm(int32_t max_pwm);
    void setWheelType(int32_t wheel_type);
    void setWheelGearRatio(double wheel_gear_ratio);
    double getWheelGearRatio();
    double getWheelTicksPerRadian();
    void setDriveType(int32_t drive_type);
    void setPidControl(int32_t pid_control);
    void nullWheelErrors();
    void setWheelDirection(int32_t wheel_direction);
    void getMotorCurrents(double &currentLeft, double &currentRight);
    int getOptionSwitch();
    int getPidControlWord();
    void setOptionSwitchReg(int32_t option_switch);
    void requestSystemEvents();
    void setSystemEvents(int32_t system_events);
    void getWheelJointPositions(double &leftWheelPosition, double &rightWheelPosition);
    void setWheelJointVelocities(double leftWheelVelocity, double rightWheelVelocity);
    void publishMotorState();

private:
    void _addOdometryRequest(std::vector<MotorMessage>& commands) const;
    void _addVelocityRequest(std::vector<MotorMessage>& commands) const;

    int16_t calculateSpeedFromRadians(double radians);
    double calculateRadiansFromTicks(int16_t ticks);

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    FirmwareParams fw_params;
    FirmwareParams prev_fw_params;

    int32_t deadman_timer;
    double ticks_per_radian;
    int32_t sendPid_count;
    bool estop_motor_power_off;

    struct Joint {
        double position;
        double velocity;
        double effort;
        double velocity_command;

        Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
    } joints_[2];

    enum MotorOrWheelNumber {
        Motor_M1 = 1,
        Motor_M2 = 2
    };

    enum WheelJointLocation {
        Left  = 0,
        Right = 1
    };

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr leftError;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rightError;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr leftCurrent;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rightCurrent;

    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr leftTickInterval;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr rightTickInterval;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr firmware_state;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_state;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_power_active;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_state;

    MotorSerial* motor_serial_;
    MotorDiagnostics motor_diag_;

    diagnostic_updater::Updater diag_updater_;
};

#endif // MOTOR_HARDWARE_HPP
