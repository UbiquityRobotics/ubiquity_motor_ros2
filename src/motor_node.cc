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
#include <fstream> 
#include <string>
#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"



MotorNode::MotorNode() 
    : Node("motor_node"), robot(nullptr) {

    RCLCPP_INFO(get_logger(), "Motor node is starting.");

}

hardware_interface::HardwareInfo MotorNode::getHwInfo() {
    hardware_interface::HardwareInfo hardware_info;
    hardware_info.name = "ubiquity_motor_ros2/MotorHardware";
    hardware_info.hardware_plugin_name = "ubiquity_motor_ros2/MotorHardware";
    hardware_info.type = "system";

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

// // Dynamic reconfiguration callback for setting ROS parameters dynamically
// void MotorNode::PID_update_callback(const ubiquity_motor::PIDConfig& config,
//                          uint32_t level) {
//     if (level == 0xFFFFFFFF) {
//         return;
//     }

//     if (level == 1) {
//         firmware_params->pid_proportional = config.PID_P;
//     } else if (level == 2) {
//         firmware_params->pid_integral = config.PID_I;
//     } else if (level == 4) {
//         firmware_params->pid_derivative = config.PID_D;
//     } else if (level == 8) {
//         firmware_params->pid_denominator = config.PID_C;
//     } else if (level == 16) {
//         firmware_params->pid_moving_buffer_size = config.PID_W;
//     } else if (level == 32) {
//         firmware_params->pid_velocity = config.PID_V;
//     } else if (level == 64) {
//         firmware_params->max_pwm = config.MAX_PWM;
//     } else {
//         RCLCPP_ERROR(get_logger(), "Unsupported dynamic_reconfigure level %u", level);
//     }
// }



void MotorNode::run() {

    // RCLCPP_INFO(get_logger(), "Params initialized");

    rclcpp::Rate ctrlLoopDelay(10);

    // int lastMcbEnabled = 1;

    // Until we have a holdoff for MCB message overruns we do this delay to be cautious
    // Twice the period for status reports from MCB
    // rclcpp::Duration mcbStatusPeriodSec(0.02);

    // std::unique_ptr<MotorHardware> robot = nullptr;
    // Keep trying to open serial
    // {
    //     int times = 0;
    //     while (rclcpp::ok() && robot.get() == nullptr) {
    //         try {
    //             robot.reset(new MotorHardware());
    //         }
    //         catch (const serial::IOException& e) {
    //             if (times % 30 == 0)
    //                 RCLCPP_FATAL(get_logger(), "Error opening serial port, trying again");
    //         }
    //         ctrlLoopDelay.sleep();
    //         times++;
    //     }
    // }
    // RCLCPP_INFO(get_logger(), "MotorHardware constructed");

    // robot->init(shared_from_this());
    // RCLCPP_INFO(get_logger(), "MotorHardware inited");


    int controller_loop_rate = 10;

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    auto options = controller_manager::get_cm_node_options();

    options.arguments({
        "--ros-args",
        // "--remap", "controller_manager:__node:=motor_hardware_node",
        "--log-level", "debug", 
        "--param", "update_rate:=" + std::to_string(controller_loop_rate),
        "--params-file", "/home/ubuntu/ros2_ws/src/ubiquity_motor_ros2/cfg/test.yaml"
        });


    std::ifstream t("/home/ubuntu/ros2_ws/src/ubiquity_motor_ros2/robot_urdf.txt");
    std::stringstream buffer;
    buffer << t.rdbuf();


    // Create the ResourceManager and register the actuator interface
    auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(buffer.str(), get_node_clock_interface(), get_node_logging_interface(), true, controller_loop_rate);
    // auto resource_manager = std::make_unique<hardware_interface::ResourceManager>(get_node_clock_interface(), get_node_logging_interface());

    // resource_manager->import_component(std::make_unique<MotorHardware>(), getHwInfo());


    controller_manager::ControllerManager cm(std::move(resource_manager), executor, "controller_manager", get_namespace(), options);

    // cm.init_controller_manager();

    // // Subscribe to the topic with overall system control ability
    // auto sub = this->create_subscription<std_msgs::msg::String>(
    //         ROS_TOPIC_SYSTEM_CONTROL, 1000, std::bind(&MotorNode::SystemControlCallback, this, std::placeholders::_1));
    // }

    // ros::AsyncSpinner spinner(1);
    // spinner.start();

// TODO: dynamic reconfigure
    // dynamic_reconfigure::Server<ubiquity_motor::PIDConfig> server;
    // dynamic_reconfigure::Server<ubiquity_motor::PIDConfig>::CallbackType f;

    // f = boost::bind(&PID_update_callback, _1, _2);
    // server.setCallback(f);

    // executor->add_node(this);




//     std::string controller_name = "ubiquity_velocity_controller";
    
//     auto load_result = cm.load_controller(controller_name);
//     if (load_result == nullptr) {
//         RCLCPP_FATAL(get_logger(), "Failed to load controller: %s", controller_name.c_str());
//         rclcpp::shutdown();
//         return;
//     }

//     RCLCPP_INFO(get_logger(), "Controller %s loaded successfully", controller_name.c_str());



//     RCLCPP_INFO(get_logger(), "Activating ubiquity_velocity_controller after 1 sec");
//     rclcpp::sleep_for(rclcpp::Duration::from_seconds(1.0).to_chrono<std::chrono::nanoseconds>());


//     // Specify the controllers you want to activate and deactivate
//     std::vector<std::string> controllers_to_activate = {"ubiquity_velocity_controller"};
//     std::vector<std::string> controllers_to_deactivate = {};  // None in this example

//     // Set the strictness level (STRICT or BEST_EFFORT)
//     int strictness = controller_manager_msgs::srv::SwitchController::Request::STRICT;

//     // Perform the controller switch
//     controller_interface::return_type switch_result = cm.switch_controller(
//         controllers_to_activate,
//         controllers_to_deactivate,
//         strictness
//     );

//    if (switch_result == controller_interface::return_type::OK)
//     {
//         RCLCPP_INFO(get_logger(), "Controller %s activated successfully.", controllers_to_activate[0].c_str());
//     }
//     else
//     {
//         RCLCPP_ERROR(get_logger(), "Failed to activate controller %s.", controllers_to_activate[0].c_str());
//     }




    // Spin to handle callbacks and manage the control loop
    executor->spin();
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MotorNode>();
    node->run();

    // rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
