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

//#include <dynamic_reconfigure/server.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <time.h>
#include <ubiquity_motor_ros2/motor_node.h>

//#include <ubiquity_motor/PIDConfig.h>
//#include <ubiquity_motor/motor_hardware.h>
//#include <ubiquity_motor/motor_message.h>
//#include <ubiquity_motor/motor_parameters.h>
//#include <boost/thread.hpp>
#include <iostream>
#include <string>
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"


// Until we have a holdoff for MCB message overruns we do this delay to be cautious
// Twice the period for status reports from MCB
auto mcbStatusSleepPeriodNs = rclcpp::Duration::from_seconds(0.02).to_chrono<std::chrono::nanoseconds>();

MotorNode::MotorNode() 
    : Node("motor_node"), g_wheel_slip_nulling(0), robot(nullptr) {

    RCLCPP_INFO(get_logger(), "Motor node is starting.");

}


// // Dynamic reconfiguration callback for setting ROS parameters dynamically
// void MotorNode::PID_update_callback(const ubiquity_motor::PIDConfig& config,
//                          uint32_t level) {
//     if (level == 0xFFFFFFFF) {
//         return;
//     }

//     if (level == 1) {
//         g_firmware_params.pid_proportional = config.PID_P;
//     } else if (level == 2) {
//         g_firmware_params.pid_integral = config.PID_I;
//     } else if (level == 4) {
//         g_firmware_params.pid_derivative = config.PID_D;
//     } else if (level == 8) {
//         g_firmware_params.pid_denominator = config.PID_C;
//     } else if (level == 16) {
//         g_firmware_params.pid_moving_buffer_size = config.PID_W;
//     } else if (level == 32) {
//         g_firmware_params.pid_velocity = config.PID_V;
//     } else if (level == 64) {
//         g_firmware_params.max_pwm = config.MAX_PWM;
//     } else {
//         RCLCPP_ERROR(get_logger(), "Unsupported dynamic_reconfigure level %u", level);
//     }
// }

// The system_control topic is used to be able to stop or start communications from the MCB
// and thus allow live firmware updates or other direct MCB serial communications to happen
// without the main control code trying to talk to the MCB
void MotorNode::SystemControlCallback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_DEBUG(get_logger(), "System control msg with content: '%s']", msg->data.c_str());

    // Manage the complete cut-off of all control to the MCB 
    // Typically used for firmware upgrade, but could be used for other diagnostics
    if (msg->data.find(MOTOR_CONTROL_CMD) != std::string::npos) {
        if (msg->data.find(MOTOR_CONTROL_ENABLE) != std::string::npos) {;
            if (g_node_params.mcbControlEnabled == 0) {  // Only show message if state changes
                RCLCPP_INFO(get_logger(), "Received System control msg to ENABLE control of the MCB");
            }
            g_node_params.mcbControlEnabled = 1;
        } else if (msg->data.find(MOTOR_CONTROL_DISABLE) != std::string::npos) {
            if (g_node_params.mcbControlEnabled != 0) {  // Only show message if state changes
                RCLCPP_INFO(get_logger(), "Received System control msg to DISABLE control of the MCB");
            }
            g_node_params.mcbControlEnabled = 0;
        }
    }

    // Manage a motor speed override used to allow collision detect motor stopping
    if (msg->data.find(MOTOR_SPEED_CONTROL_CMD) != std::string::npos) {
        if (msg->data.find(MOTOR_CONTROL_ENABLE) != std::string::npos) {;
            if (g_node_params.mcbSpeedEnabled == 0) {  // Only show message if state changes
                RCLCPP_INFO(get_logger(), "Received System control msg to ENABLE control of motor speed");
            }
            g_node_params.mcbSpeedEnabled = 1;
        } else if (msg->data.find(MOTOR_CONTROL_DISABLE) != std::string::npos) {
            if (g_node_params.mcbSpeedEnabled != 0) {  // Only show message if state changes
                RCLCPP_INFO(get_logger(), "Received System control msg to DISABLE control of motor speed");
            }
            g_node_params.mcbSpeedEnabled = 0;
        }
     }
}

//  initMcbParameters()
//
//  Setup MCB parameters that are from host level options or settings
//
void MotorNode::initMcbParameters()
{
    // A full mcb initialization requires high level system overrides to be disabled
    g_node_params.mcbControlEnabled = 1;
    g_node_params.mcbSpeedEnabled   = 1;

    // Force future calls to sendParams() to update current pid parametes on the MCB
    robot->forcePidParamUpdates();

    // Determine the wheel type to be used by the robot base 
    int32_t wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
    if (g_node_params.wheel_type == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        RCLCPP_INFO(get_logger(), "Default wheel_type of 'standard' will be used.");
        wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
    } else {
        // Any other setting leads to host setting the wheel type
        if (g_node_params.wheel_type == "standard") {
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
            RCLCPP_INFO(get_logger(), "Host is specifying wheel_type of '%s'", "standard");
        } else if (g_node_params.wheel_type == "thin"){
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_THIN;
            RCLCPP_INFO(get_logger(), "Host is specifying wheel_type of '%s'", "thin");

            // If thin wheels and no drive_type is in yaml file we will use 4wd
            if (g_node_params.drive_type == "firmware_default") {
                RCLCPP_INFO(get_logger(), "Default to drive_type of 4wd when THIN wheels unless option drive_type is set");
                g_node_params.drive_type = "4wd";
            }
        } else {
            RCLCPP_WARN(get_logger(), "Invalid wheel_type of '%s' specified! Using wheel type of standard", 
                g_node_params.wheel_type.c_str());
            g_node_params.wheel_type = "standard";
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
        }
    }
    // Write out the wheel type setting to hardware layer
    robot->setWheelType(wheel_type);
    robot->wheel_type = wheel_type;
    rclcpp::sleep_for(mcbStatusSleepPeriodNs);


    // Determine the wheel gear ratio to be used by the robot base 
    // Firmware does not use this setting so no message to firmware is required
    // This gear ratio is contained in the hardware layer so if this node got new setting update hardware layer
    robot->setWheelGearRatio(g_node_params.wheel_gear_ratio);
    RCLCPP_INFO(get_logger(), "Wheel gear ratio of %5.3f will be used.", g_node_params.wheel_gear_ratio);

    // Determine the drive type to be used by the robot base
    int32_t drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
    if (g_node_params.drive_type == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        RCLCPP_INFO(get_logger(), "Default drive_type of 'standard' will be used.");
        drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
    } else {
        // Any other setting leads to host setting the drive type
        if (g_node_params.drive_type == "standard") {
            drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
            RCLCPP_INFO(get_logger(), "Host is specifying drive_type of '%s'", "standard");
        } else if (g_node_params.drive_type == "4wd"){
            drive_type = MotorMessage::OPT_DRIVE_TYPE_4WD;
            RCLCPP_INFO(get_logger(), "Host is specifying drive_type of '%s'", "4wd");
        } else {
            RCLCPP_WARN(get_logger(), "Invalid drive_type of '%s' specified! Using drive type of standard",
                g_node_params.drive_type.c_str());
            g_node_params.drive_type = "standard";
            drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
        }
    }
    // Write out the drive type setting to hardware layer
    robot->setDriveType(drive_type);
    robot->drive_type = drive_type;
    rclcpp::sleep_for(mcbStatusSleepPeriodNs);

    int32_t wheel_direction = 0;
    if (g_node_params.wheel_direction == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        RCLCPP_INFO(get_logger(), "Firmware default wheel_direction will be used.");
    } else {
        // Any other setting leads to host setting the wheel type
        if (g_node_params.wheel_direction == "standard") {
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
            RCLCPP_INFO(get_logger(), "Host is specifying wheel_direction of '%s'", "standard");
        } else if (g_node_params.wheel_direction == "reverse"){
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_REVERSE;
            RCLCPP_INFO(get_logger(), "Host is specifying wheel_direction of '%s'", "reverse");
        } else {
            RCLCPP_WARN(get_logger(), "Invalid wheel_direction of '%s' specified! Using wheel direction of standard", 
                g_node_params.wheel_direction.c_str());
            g_node_params.wheel_direction = "standard";
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
        }
        // Write out the wheel direction setting
        robot->setWheelDirection(wheel_direction);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Tell the controller board firmware what version the hardware is at this time.
    // TODO: Read from I2C.   At this time we only allow setting the version from ros parameters
    if (robot->firmware_version >= MIN_FW_HW_VERSION_SET) {
        RCLCPP_INFO_ONCE(get_logger(), "Firmware is version %d. Setting Controller board version to %d", 
            robot->firmware_version, g_firmware_params.controller_board_version);
        robot->setHardwareVersion(g_firmware_params.controller_board_version);
        RCLCPP_DEBUG(get_logger(), "Controller board version has been set to %d", 
            g_firmware_params.controller_board_version);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Suggest to customer to have current firmware version
    if (robot->firmware_version < MIN_FW_SUGGESTED) {
        RCLCPP_ERROR_ONCE(get_logger(), "Firmware is version V%d. We strongly recommend minimum firmware version of at least V%d", 
            robot->firmware_version, MIN_FW_SUGGESTED);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    } else {
        RCLCPP_INFO_ONCE(get_logger(), "Firmware is version V%d. This meets the recommend minimum firmware versionof V%d", 
            robot->firmware_version, MIN_FW_SUGGESTED);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Certain 4WD robots rely on wheels to skid to reach final positions.
    // For such robots when loaded down the wheels can get in a state where they cannot skid.
    // This leads to motor overheating.  This code below sacrifices accurate odometry which
    // is not achievable in such robots anyway to relieve high wattage drive power when zero velocity.
    g_wheel_slip_nulling = 0;
    if ((robot->firmware_version >= MIN_FW_WHEEL_NULL_ERROR) && (g_node_params.drive_type == "4wd")) {
        g_wheel_slip_nulling = 1;
        RCLCPP_INFO(get_logger(), "Wheel slip nulling will be enabled for this 4wd system when velocity remains at zero.");
    }

    // Tell the MCB board what the port that is on the Pi I2c says on it (the mcb cannot read it's own switchs!)
    // We could re-read periodically but perhaps only every 5-10 sec but should do it from main loop
    if (robot->firmware_version >= MIN_FW_OPTION_SWITCH && robot->hardware_version >= MIN_HW_OPTION_SWITCH) {
        g_firmware_params.option_switch = robot->getOptionSwitch();
        RCLCPP_INFO(get_logger(), "Setting firmware option register to 0x%x.", g_firmware_params.option_switch);
        robot->setOptionSwitchReg(g_firmware_params.option_switch);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }
    
    if (robot->firmware_version >= MIN_FW_SYSTEM_EVENTS) {
        // Start out with zero for system events
        robot->setSystemEvents(0);  // Clear entire system events register
        robot->system_events = 0;
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Setup other firmware parameters that could come from ROS parameters
    if (robot->firmware_version >= MIN_FW_ESTOP_SUPPORT) {
        robot->setEstopPidThreshold(g_firmware_params.estop_pid_threshold);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
        robot->setEstopDetection(g_firmware_params.estop_detection);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    if (robot->firmware_version >= MIN_FW_MAX_SPEED_AND_PWM) {
        robot->setMaxFwdSpeed(g_firmware_params.max_speed_fwd);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
        robot->setMaxRevSpeed(g_firmware_params.max_speed_rev);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }
        
    return;
}

hardware_interface::HardwareInfo MotorNode::getHwInfo() {
    hardware_interface::HardwareInfo hardware_info;
    hardware_info.name = "MotorHardware";
    hardware_info.type = "actuator";  // or "actuator" or "sensor" depending on your case

    // Populate joint information
    hardware_interface::ComponentInfo joint_info;
    joint_info.name = "left_wheel_joint";

    // Initialize command_interfaces
    hardware_interface::InterfaceInfo cmd_interface_info;
    cmd_interface_info.name = "velocity";
    joint_info.command_interfaces.push_back(cmd_interface_info);

    // Initialize state_interfaces
    hardware_interface::InterfaceInfo state_interface_info;
    state_interface_info.name = "position";
    joint_info.state_interfaces.push_back(state_interface_info);

    state_interface_info.name = "velocity";
    joint_info.state_interfaces.push_back(state_interface_info);
    
    state_interface_info.name = "effort";
    joint_info.state_interfaces.push_back(state_interface_info);

    hardware_info.joints.push_back(joint_info);

    // Repeat for the right wheel joint
    joint_info.name = "right_wheel_joint";

    // Clear previous interfaces and add new ones
    joint_info.command_interfaces.clear();
    joint_info.state_interfaces.clear();

    cmd_interface_info.name = "velocity";
    joint_info.command_interfaces.push_back(cmd_interface_info);

    state_interface_info.name = "position";
    joint_info.state_interfaces.push_back(state_interface_info);

    state_interface_info.name = "velocity";
    joint_info.state_interfaces.push_back(state_interface_info);

    state_interface_info.name = "effort";
    joint_info.state_interfaces.push_back(state_interface_info);

    hardware_info.joints.push_back(joint_info);

    return hardware_info;
}

void MotorNode::run() {

    g_firmware_params = FirmwareParams(shared_from_this());
    g_serial_params   = CommsParams(shared_from_this());
    g_node_params     = NodeParams(shared_from_this());

    rclcpp::Rate ctrlLoopDelay(g_node_params.controller_loop_rate);

    int lastMcbEnabled = 1;

    // Until we have a holdoff for MCB message overruns we do this delay to be cautious
    // Twice the period for status reports from MCB
    // rclcpp::Duration mcbStatusPeriodSec(0.02);

    // std::unique_ptr<MotorHardware> robot = nullptr;
    // Keep trying to open serial
    {
        int times = 0;
        while (rclcpp::ok() && robot.get() == nullptr) {
            try {
                robot.reset(new MotorHardware(shared_from_this(), g_node_params, g_serial_params, g_firmware_params));
            }
            catch (const serial::IOException& e) {
                if (times % 30 == 0)
                    RCLCPP_FATAL(get_logger(), "Error opening serial port %s, trying again", g_serial_params.serial_port.c_str());
            }
            ctrlLoopDelay.sleep();
            times++;
        }
    }

    // Create a MultiThreadedExecutor
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto options = controller_manager::get_cm_node_options();

    options.arguments({
        "--ros-args",
        // "--remap", "_target_node_name:__node:=dst_node_name",
        "--log-level", "info"});


    // Create the ResourceManager and register the actuator interface
    auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(get_node_clock_interface(), get_node_logging_interface());
    resource_manager->import_component(std::move(robot), getHwInfo());

    controller_manager::ControllerManager cm(std::move(resource_manager), executor, "controller_manager", get_namespace(), options);

    // Subscribe to the topic with overall system control ability
    auto sub = this->create_subscription<std_msgs::msg::String>(
            ROS_TOPIC_SYSTEM_CONTROL, 1000, std::bind(&MotorNode::SystemControlCallback, this, std::placeholders::_1));
    // }

    // ros::AsyncSpinner spinner(1);
    // spinner.start();
    std::thread executor_thread(std::bind(&rclcpp::executors::MultiThreadedExecutor::spin, executor)); // instead of AsyncSpinner

    // dynamic_reconfigure::Server<ubiquity_motor::PIDConfig> server;
    // dynamic_reconfigure::Server<ubiquity_motor::PIDConfig>::CallbackType f;

    // f = boost::bind(&PID_update_callback, _1, _2);
    // server.setCallback(f);





    robot->setParams(g_firmware_params);
    robot->requestFirmwareVersion();

    // wait for reply then we know firmware version
    rclcpp::sleep_for(mcbStatusSleepPeriodNs);

    // Make sure firmware is listening
    {
        robot->diag_updater.broadcast(0, "Establishing communication with motors");
        // Start times counter at 1 to prevent false error print (0 % n = 0)
        int times = 1;
        while (rclcpp::ok() && robot->firmware_version == 0) {
            if (times % 30 == 0)
                RCLCPP_ERROR(get_logger(), "The Firmware not reporting its version");
                robot->requestFirmwareVersion(); // TODO: Shouldn't this be in the if statement?
            robot->readInputs(0);
            rclcpp::sleep_for(mcbStatusSleepPeriodNs);
            times++;
        }
    }

    if (robot->firmware_version >= MIN_FW_FIRMWARE_DATE) {
        // If supported by firmware also request date code for this version
        RCLCPP_INFO(get_logger(), "Requesting Firmware daycode ");
        robot->requestFirmwareDate();
    }

    // Setup MCB parameters that are defined by host parameters in most cases
    RCLCPP_INFO(get_logger(), "Initializing MCB");
    initMcbParameters();
    RCLCPP_INFO(get_logger(), "Initialization of MCB completed.");

    if (robot->firmware_version >= MIN_FW_SYSTEM_EVENTS) {
        // Start out with zero for system events
        robot->setSystemEvents(0);  // Clear entire system events register
        robot->system_events = 0;
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Send out the refreshable firmware parameters, most are the PID terms
    // We must be sure num_fw_params is set to the modulo used in sendParams()
    for (int i = 0; i < robot->num_fw_params; i++) {
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
        robot->sendParams();
    }

    float expectedCycleTime = 1.0 / g_node_params.controller_loop_rate;
    rclcpp::Duration minCycleTime = rclcpp::Duration::from_seconds(0.75 * expectedCycleTime);
    rclcpp::Duration maxCycleTime = rclcpp::Duration::from_seconds(1.25 * expectedCycleTime);

    // Clear any commands the robot has at this time
    robot->clearCommands();

    double leftLastWheelPos   = 0.0;
    double rightLastWheelPos  = 0.0;
    double leftWheelPos  = 0.0;
    double rightWheelPos = 0.0;
    robot-> getWheelJointPositions(leftLastWheelPos, rightWheelPos);
    rclcpp::Duration zeroVelocityTime(0, 0);

    // Define a period where if wheels are under stress for this long we back off stress
    rclcpp::Duration wheelSlipNullingPeriod = rclcpp::Duration::from_seconds(2.0);

    int wheelSlipEvents = 0;

    RCLCPP_INFO(get_logger(), "Starting motor control node now");

    // Implement a speed reset while ESTOP is active and a delay after release
    double estopReleaseDeadtime = 0.8;
    double estopReleaseDelay    = 0.0;

    // Setup to be able to do periodic operations based on elapsed times
    rclcpp::Time current_time;
    // rclcpp::Duration sysMaintPeriod(60.0);     // A periodic MCB maintenance operation
    // rclcpp::Duration jointUpdatePeriod(0.25);  // A periodic time to update joint velocity

    auto sysMaintPeriod = rclcpp::Duration::from_seconds(60.0);       // A periodic MCB maintenance operation
    auto jointUpdatePeriod = rclcpp::Duration::from_seconds(0.25);    // A periodic time to update joint velocity


    rclcpp::Time last_loop_time = rclcpp::Clock().now();
    rclcpp::Duration elapsed_loop_time = rclcpp::Duration(0, 0);
    rclcpp::Time last_sys_maint_time = last_loop_time;
    rclcpp::Time last_joint_time = last_loop_time;
    ctrlLoopDelay.sleep();                  // Do delay to setup periodic loop delays
    uint32_t loopIdx = 0;

    while (rclcpp::ok()) {
        current_time = rclcpp::Clock().now();
        elapsed_loop_time = current_time - last_loop_time;
        last_loop_time = current_time;
        loopIdx += 1;

        // Speical handling if motor control is disabled.  skip the entire loop
        if (g_node_params.mcbControlEnabled == 0) {
            // Check for if we are just starting to go into idle mode and release MCB
            if (lastMcbEnabled == 1) {
                RCLCPP_WARN(get_logger(), "Motor controller going offline and closing MCB serial port");
                robot->closePort();
            }
            lastMcbEnabled = 0;
            ctrlLoopDelay.sleep();        // Allow controller to process command
            continue;
        }

        if (lastMcbEnabled == 0) {        // Were disabled but re-enabled so re-setup mcb
            bool portOpenStatus;
            lastMcbEnabled = 1;
            RCLCPP_WARN(get_logger(), "Motor controller went from offline to online!");
            portOpenStatus = robot->openPort();  // Must re-open serial port
            if (portOpenStatus == true) {
                robot->setSystemEvents(0);  // Clear entire system events register
                robot->system_events = 0;
                rclcpp::sleep_for(mcbStatusSleepPeriodNs);
              
                // Setup MCB parameters that are defined by host parameters in most cases
                initMcbParameters();
                RCLCPP_WARN(get_logger(), "Motor controller has been re-initialized as we go back online");
            } else {
                // We do not have recovery from this situation and it seems not possible
                RCLCPP_ERROR(get_logger(), "ERROR in re-opening of the Motor controller!");
            }
        }

        // Determine and set wheel velocities in rad/sec from hardware positions in rads
        rclcpp::Duration elapsed_time = current_time - last_joint_time;
        if (elapsed_time > jointUpdatePeriod) {
            last_joint_time = rclcpp::Clock().now();
            double leftWheelVel  = 0.0;
            double rightWheelVel = 0.0;
            robot-> getWheelJointPositions(leftWheelPos, rightWheelPos);
            leftWheelVel  = (leftWheelPos  - leftLastWheelPos)  / elapsed_time.seconds();
            rightWheelVel = (rightWheelPos - rightLastWheelPos) / elapsed_time.seconds();
            robot->setWheelJointVelocities(leftWheelVel, rightWheelVel); // rad/sec
            leftLastWheelPos  = leftWheelPos;
            rightLastWheelPos = rightWheelPos;

            // Publish motor state at this time
            robot->publishMotorState();

            // Implement static wheel slippage relief
            // Deal with auto-null of MCB wheel setpoints if wheel slip nulling is enabled
            // We null wheel torque if wheel speed has been very low for a long time
            // This would be even better if we only did this when over a certain current is heating the wheel
            if (g_wheel_slip_nulling != 0) {
                if (robot->areWheelSpeedsLower(WHEEL_SLIP_THRESHOLD) != 0) {
                    zeroVelocityTime += jointUpdatePeriod;   // add to time at zero velocity
                    if (zeroVelocityTime > wheelSlipNullingPeriod) {
                        // null wheel error if at zero velocity for the nulling check period
                        // OPTION: We could also just null wheels at high wheel power
                        RCLCPP_DEBUG(get_logger(), "Applying wheel slip relief now with slip period of %4.1f sec ",
                            wheelSlipNullingPeriod.seconds());
                        wheelSlipEvents += 1;
                        robot->nullWheelErrors();
                        zeroVelocityTime = rclcpp::Duration(0, 0);   // reset time we have been at zero velocity
                    }
                } else {
                    zeroVelocityTime = rclcpp::Duration(0, 0);   // reset time we have been at zero velocity
                }
            }
        }

        // Read in and process all serial packets that came from the MCB.
        // The MotorSerial class has been receiving and queing packets from the MCB
        robot->readInputs(loopIdx);

        // Here is the update call that ties our motor node to the standard ROS classes.
        // We will supply a brief overview of how this ROS robot interfaces to our hardware.
        // At the top is the controller_manager and just below it is hardware_interface
        // The ROS DiffDriveController works with one 'joint' for each wheel.
        // to know and publish /odom and generate the odom to base_footprint in /tf topic
        //
        // Refer to motor_hardware.cc code and study any interfaces to the joints_ array
        // where there is one 'joint' for each wheel. 
        // The joints_ used by ROS DiffDriveController have these 3 key members:
        //   position         - We incrementally update wheel position (from encoders)
        //   velocity         - We set desired wheel velocity in Rad/Sec
        //   velocity_command - We are told what speeds to set our MCB hardware with from this

        if ((minCycleTime < elapsed_loop_time) && (elapsed_loop_time < maxCycleTime)) {
            cm.update(current_time, elapsed_loop_time);  // Key update to controller_manager
        }
        else {
            RCLCPP_WARN(get_logger(), "Resetting controller due to time jump %f seconds",
                     elapsed_loop_time.seconds());
            cm.update(current_time, rclcpp::Duration::from_seconds(1.0 / g_node_params.controller_loop_rate)); // In ros1 there was "true" as  last param
            robot->clearCommands();
        }
        robot->setParams(g_firmware_params);
        robot->sendParams(); 

        // Periodically watch for MCB board having been reset which is an MCB system event
        // This is also a good place to refresh or show status that may have changed
        elapsed_time = current_time - last_sys_maint_time;
        if ((robot->firmware_version >= MIN_FW_SYSTEM_EVENTS) && (elapsed_time > sysMaintPeriod)) {
            robot->requestSystemEvents();
            rclcpp::sleep_for(mcbStatusSleepPeriodNs);
            last_sys_maint_time = rclcpp::Clock().now();

            // See if we are in a low battery voltage state
            std::string batStatus = "OK";
            if (robot->getBatteryVoltage() < g_firmware_params.battery_voltage_low_level) {
                batStatus = "LOW!";
            }

            // Post a status message for MCB state periodically. This may be nice to do more on as required
            RCLCPP_INFO(get_logger(), "Battery = %5.2f volts [%s], MCB system events 0x%x,  PidCtrl 0x%x, WheelType '%s' DriveType '%s' GearRatio %6.3f",
                robot->getBatteryVoltage(), batStatus.c_str(), robot->system_events, robot->getPidControlWord(),
                (robot->wheel_type == MotorMessage::OPT_WHEEL_TYPE_THIN) ? "thin" : "standard",
                g_node_params.drive_type.c_str(), robot->getWheelGearRatio());

            // If we detect a power-on of MCB we should re-initialize MCB
            if ((robot->system_events & MotorMessage::SYS_EVENT_POWERON) != 0) {
                RCLCPP_WARN(get_logger(), "Detected Motor controller PowerOn event!");
                robot->setSystemEvents(0);  // Clear entire system events register
                robot->system_events = 0;
                rclcpp::sleep_for(mcbStatusSleepPeriodNs);
              
                // Setup MCB parameters that are defined by host parameters in most cases
                initMcbParameters();
                RCLCPP_WARN(get_logger(), "Motor controller has been re-initialized");
            }

            // a periodic refresh of wheel type which is a safety net due to it's importance.
            // This can be removed when a solid message protocol is developed
            if (robot->firmware_version >= MIN_FW_WHEEL_TYPE_THIN) {
                // Refresh the wheel type setting
                robot->setWheelType(robot->wheel_type);
                rclcpp::sleep_for(mcbStatusSleepPeriodNs);
            }
            // a periodic refresh of drive type which is a safety net due to it's importance.
            if (robot->firmware_version >= MIN_FW_DRIVE_TYPE) {
                // Refresh the drive type setting
                robot->setDriveType(robot->drive_type);
                rclcpp::sleep_for(mcbStatusSleepPeriodNs);
            }
        }

        // Update motor controller speeds unless global disable is set, perhaps for colision safety
        if (robot->getEstopState()) {
            robot->writeSpeedsInRadians(0.0, 0.0);    // We send zero velocity when estop is active
            estopReleaseDelay = estopReleaseDeadtime;
        } else {
            if (estopReleaseDelay > 0.0) {
                // Implement a delay after estop release where velocity remains zero
                estopReleaseDelay -= (1.0/g_node_params.controller_loop_rate);
                robot->writeSpeedsInRadians(0.0, 0.0);
            } else {
                if (g_node_params.mcbSpeedEnabled != 0) {     // A global disable used for safety at node level
                    robot->writeSpeeds();         // Normal operation using current system speeds
                } else {
                    robot->writeSpeedsInRadians(0.0, 0.0);
                }
            }
        }

        robot->diag_updater.force_update();
        ctrlLoopDelay.sleep();        // Allow controller to process command
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorNode>();
    node->run();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
