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

#ifndef UBIQUITY_MOTOR_MOTOR_PARMETERS_H
#define UBIQUITY_MOTOR_MOTOR_PARMETERS_H

#include <rclcpp/rclcpp.hpp>
#include <iostream>

// These defines are for a high level topic for control of the motor node at a system level
#define ROS_TOPIC_SYSTEM_CONTROL  "system_control"    // A topic for system level control commands
#define MOTOR_CONTROL_CMD         "motor_control"     // A mnumonic for a motor control system command
#define MOTOR_CONTROL_ENABLE      "enable"            // Parameter for MOTOR_CONTROL_CMD to enable control
#define MOTOR_CONTROL_DISABLE     "disable"           // Parameter for MOTOR_CONTROL_CMD to enable control
#define MOTOR_SPEED_CONTROL_CMD   "speed_control"     // A mnumonic for disable of speed to avoid colision

// The gear ratio defaults for wheels shipped with Magni
#define WHEEL_GEAR_RATIO_1        ((double)(4.294))   // Default original motor gear ratio for Magni
#define WHEEL_GEAR_RATIO_2        ((double)(5.170))   // 2nd version standard Magni wheels gear ratio

template <typename T>
T getParamOrDefault(const std::shared_ptr<rclcpp::Node>& n, std::string parameter_name,
                    T default_val) {

    rclcpp::Parameter param;
        
    n->declare_parameter(parameter_name, default_val);

    n->get_parameter(parameter_name, param);

    // std::cout << "param " << parameter_name << " val: " << param.get_value<T>() << std::endl;

    return param.get_value<T>();

}

struct FirmwareParams {
    int32_t pid_proportional;
    int32_t pid_integral;
    int32_t pid_derivative;
    int32_t pid_velocity;
    int32_t pid_denominator;
    int32_t pid_moving_buffer_size;
    int32_t pid_control;
    int32_t controller_board_version;
    int32_t estop_detection;
    int32_t estop_pid_threshold;
    int32_t max_speed_fwd;
    int32_t max_speed_rev;
    int32_t max_pwm;
    int32_t deadman_timer;
    int32_t deadzone_enable;
    int32_t hw_options;
    int32_t option_switch;
    int32_t system_events;
    float battery_voltage_multiplier;
    float battery_voltage_offset;
    float battery_voltage_low_level;
    float battery_voltage_critical;

    FirmwareParams()
        : pid_proportional(5000),
          pid_integral(5),
          pid_derivative(-110),
          pid_velocity(0),
          pid_denominator(1000),
          pid_moving_buffer_size(70),
	      pid_control(0),
          controller_board_version(51),
          estop_detection(1),
          estop_pid_threshold(1500),
          max_speed_fwd(104),
          max_speed_rev(-104),
          max_pwm(325),
          deadman_timer(2400000),
          deadzone_enable(0),
          hw_options(0),
          option_switch(0),
          system_events(0),

          // ADC uses Vcc/2 for 2048 counts. We feed in battery with a 1/21 ratio
          // So for 5.00V mult=0.05127  When Vcc=5.16V (Pi4 mod) mult = 0.0529
          battery_voltage_multiplier(0.05127),
          battery_voltage_offset(0.0),
          battery_voltage_low_level(23.2),
          battery_voltage_critical(22.5){};

    FirmwareParams(const std::shared_ptr<rclcpp::Node>& n)
        : pid_proportional(5000),
          pid_integral(5),
          pid_derivative(-110),
          pid_velocity(0),
          pid_denominator(1000),
          pid_moving_buffer_size(70),
	      pid_control(0),
          controller_board_version(51),
          estop_detection(1),
          estop_pid_threshold(1500),
          max_speed_fwd(104),
          max_speed_rev(-104),
          max_pwm(325),
          deadman_timer(2400000),
          deadzone_enable(0),
          hw_options(0),
          option_switch(0),
          system_events(0),

          // ADC uses Vcc/2 for 2048 counts. We feed in battery with a 1/21 ratio
          // So for 5.00V mult=0.05127  When Vcc=5.16V (Pi4 mod) mult = 0.0529
          battery_voltage_multiplier(0.05127),
          battery_voltage_offset(0.0),
          battery_voltage_low_level(23.2),
          battery_voltage_critical(22.5)
        {
        // clang-format off
        pid_proportional = getParamOrDefault(
            n, "pid_proportional", pid_proportional);
        pid_integral = getParamOrDefault(
            n, "pid_integral", pid_integral);
        pid_derivative = getParamOrDefault(
            n, "pid_derivative", pid_derivative);
        pid_velocity = getParamOrDefault(
            n, "pid_velocity", pid_velocity);
        pid_denominator = getParamOrDefault(
            n, "pid_denominator", pid_denominator);
	pid_control = getParamOrDefault(
            n, "pid_control", pid_control);
        pid_moving_buffer_size = getParamOrDefault(
            n, "window_size", pid_moving_buffer_size);
        controller_board_version = getParamOrDefault(
            n, "controller_board_version", controller_board_version);
        estop_detection = getParamOrDefault(
            n, "fw_estop_detection", estop_detection);
        estop_pid_threshold = getParamOrDefault(
            n, "fw_estop_pid_threshold", estop_pid_threshold);
        max_speed_fwd = getParamOrDefault(
            n, "fw_max_speed_fwd", max_speed_fwd);
        max_speed_rev = getParamOrDefault(
            n, "fw_max_speed_rev", max_speed_rev);
        max_pwm = getParamOrDefault(
            n, "fw_max_pwm", max_pwm);
        deadman_timer = getParamOrDefault(
            n, "deadman_timer", deadman_timer);
        deadzone_enable = getParamOrDefault(
            n, "deadzone_enable", deadzone_enable);
        battery_voltage_offset = getParamOrDefault(
            n, "battery_voltage_offset", battery_voltage_offset);
        battery_voltage_multiplier = getParamOrDefault(
            n, "battery_voltage_multiplier", battery_voltage_multiplier);
        battery_voltage_low_level = getParamOrDefault(
            n, "battery_voltage_low_level", battery_voltage_low_level);
        battery_voltage_critical = getParamOrDefault(
            n, "battery_voltage_critical", battery_voltage_critical);
        // clang-format on
    };
};

struct CommsParams {
    std::string serial_port;
    int32_t baud_rate;

    CommsParams()
        : serial_port("/dev/ttyS0"), baud_rate(38400) {};

    CommsParams(const std::shared_ptr<rclcpp::Node>& n)
        : serial_port("/dev/ttyS0"), baud_rate(38400) {
        // clang-format off
        serial_port = getParamOrDefault(
            n, "serial_port", serial_port);
        baud_rate = getParamOrDefault(
            n, "serial_baud", baud_rate);
        // clang-format on
    };
};

struct NodeParams {
    int controller_loop_rate;
    std::string wheel_type;
    std::string wheel_direction;
    double wheel_gear_ratio;
    std::string drive_type;

    int mcbControlEnabled;    // State to allow suspension of MCB serial control for diagnostic purposes
    int mcbSpeedEnabled;      // State to allow zero speed override for safety reasons

    NodeParams() : controller_loop_rate(10),
        wheel_type("firmware_default"), 
        wheel_direction("firmware_default"),
        wheel_gear_ratio(WHEEL_GEAR_RATIO_1),
        drive_type("firmware_default"),
        mcbControlEnabled(1),
        mcbSpeedEnabled(1){};

    NodeParams(const std::shared_ptr<rclcpp::Node>& n) : controller_loop_rate(10),
        wheel_type("firmware_default"),
        wheel_direction("firmware_default"),
	wheel_gear_ratio(WHEEL_GEAR_RATIO_1),
        drive_type("firmware_default"),
        mcbControlEnabled(1),
        mcbSpeedEnabled(1) {
        // clang-format off
        controller_loop_rate = getParamOrDefault(
            n, "controller_loop_rate", controller_loop_rate);
        wheel_type = getParamOrDefault(
            n, "wheel_type", wheel_type);
        wheel_direction = getParamOrDefault(
            n, "wheel_direction", wheel_direction);
        wheel_gear_ratio = getParamOrDefault(
            n, "wheel_gear_ratio", wheel_gear_ratio);
        drive_type = getParamOrDefault(
            n, "drive_type", drive_type);
        // clang-format on
    };
};

#endif  // UBIQUITY_MOTOR_MOTOR_PARMETERS_H
