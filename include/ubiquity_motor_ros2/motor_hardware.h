/**
Copyright (c) 2016, Ubiquity Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef MOTORHARDWARE_H
#define MOTORHARDWARE_H

#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "ubiquity_motor_ros2_msgs/msg/motor_state.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <ubiquity_motor_ros2/motor_parameters.h>
#include <ubiquity_motor_ros2/motor_message.h>
#include <ubiquity_motor_ros2/motor_serial.h>

// #include "controller_manager/controller_manager.hpp"
// #include "hardware_interface/resource_manager.hpp"

// TODO: Enhancement - Make WHEEL_SLIP_THRESHOLD be a ROS param
#define WHEEL_SLIP_THRESHOLD  (0.08)   // Rotation below which we null excess wheel torque in 4wd drive_type

// Mininum hardware versions required for various features
#define MIN_HW_OPTION_SWITCH 50
// Until we have a holdoff for MCB message overruns we do this delay to be cautious
// Twice the period for status reports from MCB
// auto mcbStatusSleepPeriodNs = rclcpp::Duration::from_seconds(0.02).to_chrono<std::chrono::nanoseconds>();

struct MotorDiagnostics {
    MotorDiagnostics()
        : odom_update_status(
              diagnostic_updater::FrequencyStatusParam(&odom_min_freq, &odom_max_freq)) {}
    // Communication Statuses
    int firmware_version = 0;
    int firmware_date    = 0;
    int firmware_options = 0;

    // These are for diagnostic topic output 
    int fw_pid_proportional = 0;
    int fw_pid_integral = 0;
    int fw_pid_derivative = 0;
    int fw_pid_control = 0;
    int fw_pid_velocity = 0;
    int fw_pid_denominator = 0;
    int fw_pid_moving_buffer_size = 0;
    int fw_max_pwm = 0;
   
    double odom_max_freq = 1000;
    double odom_min_freq = 50;
    diagnostic_updater::FrequencyStatus odom_update_status;

    // Limits
    bool left_pwm_limit = false;
    bool right_pwm_limit = false;
    bool left_integral_limit = false;
    bool right_integral_limit = false;
    bool left_max_speed_limit = false;
    bool right_max_speed_limit = false;
    bool param_limit_in_firmware = false;

    // Power supply statuses
    float battery_voltage = 0.0;
    float battery_voltage_low_level = 22.5;
    float battery_voltage_critical = 21.0;

    // Wheel current states
    double motorCurrentLeft  = 0.0;
    double motorCurrentRight = 0.0;

    // ADC count for zero current. We could calibrate this if required. 
    // Nominally 1024 and goes up from there this lower value is used. 
    double motorAmpsZeroAdcCount = 1015;    

    int    motorPwmDriveLeft  = 0;
    int    motorPwmDriveRight = 0;

    /* For later implementation (firmware support)
    bool  main_5V_error = false;
    bool  main_5V_ol = false;
    bool  main_12V_error = false;
    bool  main_12V_ol = false;
    bool  aux_5V_error = false;
    bool  aux_5V_ol = false;
    bool  aux_12V_error = false;
    bool  aux_12V_ol = false;
    */

    bool  estop_motor_power_off = false;  // for Diagnostic reporting of ESTOP switch

    void firmware_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void limit_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void battery_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_power_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_p_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_i_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_d_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_pid_v_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void motor_max_pwm_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void firmware_options_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
    void firmware_date_status(diagnostic_updater::DiagnosticStatusWrapper &stat);
};

class MotorHardware : public hardware_interface::ActuatorInterface {
public:
    // MotorHardware(const std::shared_ptr<rclcpp::Node>& n, NodeParams& node_params, CommsParams& serial_params, FirmwareParams& firmware_params);
    MotorHardware();
    void init(const std::shared_ptr<rclcpp::Node>& n, const std::shared_ptr<NodeParams>& node_params, const std::shared_ptr<CommsParams>& serial_params, const std::shared_ptr<FirmwareParams>& firmware_params);

    virtual ~MotorHardware();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    // hardware_interface::return_type start() override;
    // hardware_interface::return_type stop() override;
    hardware_interface::return_type read(const rclcpp::Time& current_time, const rclcpp::Duration& elapsed_loop_time) override;
    hardware_interface::return_type write(const rclcpp::Time& current_time, const rclcpp::Duration& elapsed_loop_time) override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;


    // hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override
    // {

    //     return hardware_interface::CallbackReturn::SUCCESS;
    // }

    // hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override
    // {
    //     // Stop hardware operation (e.g., stop motors, stop sensor polling)
    //     return hardware_interface::CallbackReturn::SUCCESS;
    // }

    // hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override
    // {
    //     // Cleanup resources (e.g., close connections, free memory)
    //     return hardware_interface::CallbackReturn::SUCCESS;
    // }

    // hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override
    // {
    //     // Final shutdown and release all resources
    //     return hardware_interface::CallbackReturn::SUCCESS;
    // }


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
    float getBatteryVoltage(void);
    void setDeadmanTimer(int32_t deadman);
    void setDeadzoneEnable(int32_t deadzone_enable);
    void setDebugLeds(bool led1, bool led2);
    void setHardwareVersion(int32_t hardware_version);
    void setEstopPidThreshold(int32_t estop_pid_threshold);
    void setEstopDetection(int32_t estop_detection);
    bool getEstopState(void);
    void setMaxFwdSpeed(int32_t max_speed_fwd);
    void setMaxRevSpeed(int32_t max_speed_rev);
    void setMaxPwm(int32_t max_pwm);
    void setWheelType(int32_t wheel_type);
    void setWheelGearRatio(double wheel_gear_ratio);
    double getWheelGearRatio(void);
    double getWheelTicksPerRadian(void);
    void setDriveType(int32_t drive_type);
    void setPidControl(int32_t pid_control);
    void nullWheelErrors(void);
    void setWheelDirection(int32_t wheel_direction);
    void getMotorCurrents(double &currentLeft, double &currentRight);
    int  getOptionSwitch(void);
    int  getPidControlWord(void);
    void setOptionSwitchReg(int32_t option_switch);
    void requestSystemEvents();
    void setSystemEvents(int32_t system_events);
    void getWheelJointPositions(double &leftWheelPosition, double &rightWheelPosition);
    void setWheelJointVelocities(double leftWheelVelocity, double rightWheelVelocity);
    void publishMotorState(void);
    void initMcbParameters();

    void manageMotorControllerState();
    void setWheelVelocities(const rclcpp::Time& current_time, const rclcpp::Duration & elapsed_loop_time);
    // hardware_interface::HardwareInfo getHwInfo();
    void checkMcbReset();
    void writeMotorSpeeds();


    int firmware_version;
    int firmware_date;
    int firmware_options;
    int num_fw_params;  // This is used for sendParams as modulo count
    int hardware_version;
    int estop_pid_threshold;
    int max_speed_fwd;
    int max_speed_rev;
    int max_pwm;
    int pid_control;
    int deadman_enable;
    int system_events;
    int wheel_type;
    double wheel_gear_ratio;
    int drive_type;
    int wheel_slip_nulling;

    std::unique_ptr<diagnostic_updater::Updater> diag_updater;
    // std::shared_ptr<controller_manager::ControllerManager> controller_manager = nullptr;

private:
    void _addOdometryRequest(std::vector<MotorMessage>& commands) const;
    void _addVelocityRequest(std::vector<MotorMessage>& commands) const;

    int16_t calculateSpeedFromRadians(double radians);
    double calculateRadiansFromTicks(int16_t ticks);
    
    // This utility opens and reads 1 or more bytes from a device on an I2C bus
    // This method was taken on it's own from a big I2C class we may choose to use later
    int i2c_BufferRead(const char *i2cDevFile, uint8_t i2c8bitAddr, uint8_t *pBuffer, int16_t chipRegAddr, uint16_t NumBytesToRead);

    std::vector<hardware_interface::StateInterface> state_interfaces_;
    std::vector<hardware_interface::CommandInterface> command_interfaces_;

    rclcpp::Node::SharedPtr node;

    std::shared_ptr<NodeParams> node_params;
    std::shared_ptr<CommsParams> serial_params;
    std::shared_ptr<FirmwareParams> fw_params;

    FirmwareParams prev_fw_params;

    rclcpp::Logger logger;

    int lastMcbEnabled;
    int wheelSlipEvents;

    // rclcpp::Time current_time;
    // rclcpp::Time last_loop_time;
    // rclcpp::Duration elapsed_loop_time;
    rclcpp::Time last_sys_maint_time;
    rclcpp::Time last_joint_time;
    uint32_t loopIdx;
    // rclcpp::Rate ctrlLoopDelay;
    rclcpp::Duration zeroVelocityTime;
    std::chrono::nanoseconds mcbStatusSleepPeriodNs;

    rclcpp::Duration sysMaintPeriod;       // A periodic MCB maintenance operation
    rclcpp::Duration jointUpdatePeriod;    // A periodic time to update joint velocity
    rclcpp::Duration wheelSlipNullingPeriod;

    double leftLastWheelPos;
    double rightLastWheelPos;
    double leftWheelPos;
    double rightWheelPos;

    double estopReleaseDeadtime;
    double estopReleaseDelay;

    int32_t deadman_timer;

    double  ticks_per_radian;       // Odom ticks per radian for wheel encoders in use

    int32_t sendPid_count;

    bool estop_motor_power_off;    // Motor power inactive, most likely from ESTOP switch


    struct Joint {
        double position;
        double velocity;
        double effort;
        double velocity_command;

        Joint() : position(0), velocity(0), effort(0), velocity_command(0) {}
    } joints_[2];

    // MessageTypes enum for refering to motor or wheel number
    enum MotorOrWheelNumber {
        Motor_M1 = 1,
        Motor_M2 = 2
    };

    // MessageTypes enum in class to avoid global namespace pollution
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
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_state;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motor_power_active;
    rclcpp::Publisher<ubiquity_motor_ros2_msgs::msg::MotorState>::SharedPtr motor_state;

    MotorSerial* motor_serial_;

    MotorDiagnostics motor_diag_;

    FRIEND_TEST(MotorHardwareTests, nonZeroWriteSpeedsOutputs);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPosition);
    FRIEND_TEST(MotorHardwareTests, odomUpdatesPositionMax);
};

#endif
