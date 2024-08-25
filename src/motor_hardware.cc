// #include "motor_hardware.h"
#include <ubiquity_motor_ros2/motor_hardware.h>

MotorHardware::MotorHardware(const std::shared_ptr<rclcpp::Node>& n, NodeParams node_params, CommsParams serial_params, FirmwareParams firmware_params)
     : logger(rclcpp::get_logger("MotorHardware")),
      diag_updater_(this)
{
    // Initialize publishers
    leftError = n->create_publisher<std_msgs::msg::Int32>("left_error", 10);
    rightError = n->create_publisher<std_msgs::msg::Int32>("right_error", 10);
    leftCurrent = n->create_publisher<std_msgs::msg::Float32>("left_current", 10);
    rightCurrent = n->create_publisher<std_msgs::msg::Float32>("right_current", 10);
    leftTickInterval = n->create_publisher<std_msgs::msg::Int32>("left_tick_interval", 10);
    rightTickInterval = n->create_publisher<std_msgs::msg::Int32>("right_tick_interval", 10);
    firmware_state = n->create_publisher<std_msgs::msg::String>("firmware_state", 10);
    battery_state = n->create_publisher<std_msgs::msg::Float32>("battery_state", 10);
    motor_power_active = n->create_publisher<std_msgs::msg::Bool>("motor_power_active", 10);
    motor_state = n->create_publisher<std_msgs::msg::String>("motor_state", 10);

    // Initialize other members
    motor_serial_ = nullptr; // Or actual initialization
    diag_updater_.setHardwareID("motor_hardware");

    // Diagnostic callback setup
    // diag_updater_.add("Motor Diagnostics", this, &MotorHardware::diagnosticCallback);
}

MotorHardware::~MotorHardware() {
    // Cleanup
    closePort();
}

// Implement the necessary methods...

hardware_interface::CallbackReturn MotorHardware::on_init(const hardware_interface::HardwareInfo &info) {
//    if (configure_default(info) != hardware_interface::CallbackReturn::SUCCESS) {
//         return hardware_interface::CallbackReturn::ERROR;
//     }

    // Initialize joint names and other variables
    for (const auto &joint : info.joints) {
        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(logger, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(logger, "Joint '%s' has %zu state interfaces found. 3 expected.", joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize joint state (position, velocity, effort)
        joints_.emplace_back();

        // Register the state interfaces
        hardware_interface::JointStateHandle state_handle(
            joint.name, &joints_.back().position, &joints_.back().velocity, &joints_.back().effort);
        state_interfaces_.emplace_back(state_handle);

        // Register the command interface
        hardware_interface::JointHandle cmd_handle(state_handle, &joints_.back().velocity_command);
        command_interfaces_.emplace_back(cmd_handle);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorHardware::export_state_interfaces() {
    // Implementation here
}

std::vector<hardware_interface::CommandInterface> MotorHardware::export_command_interfaces() {
    // Implementation here
}

// hardware_interface::return_type MotorHardware::start() {
//     // Implementation here
//     return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type MotorHardware::stop() {
//     // Implementation here
//     return hardware_interface::return_type::OK;
// }

hardware_interface::return_type MotorHardware::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // Implementation here
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorHardware::write(const rclcpp::Time & time, const rclcpp::Duration & period) {
    // Implementation here
    return hardware_interface::return_type::OK;
}

void MotorHardware::closePort() {
    // Implementation here
}

bool MotorHardware::openPort() {
    // Implementation here
    return true;
}

// Implement other methods similarly...

