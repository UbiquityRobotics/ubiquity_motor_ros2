#include <rclcpp/rclcpp.hpp>
#include <ubiquity_motor_ros2/motor_hardware.h>
#include <ubiquity_motor_ros2/motor_message.h>
#include <boost/assign.hpp>
#include <boost/math/special_functions/round.hpp>
#include <chrono>

// To access I2C we need some system includes
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#define  I2C_DEVICE  "/dev/i2c-1"     // This is specific to default Magni I2C port on host
const static uint8_t  I2C_PCF8574_8BIT_ADDR = 0x40; // I2C addresses are 7 bits but often shown as 8-bit

//#define SENSOR_DISTANCE 0.002478


// For experimental purposes users will see that the wheel encoders are three phases
// of very neaar 43 pulses per revolution or about 43*3 edges so we see very about 129 ticks per rev
// This leads to 129/(2*Pi)  or about 20.53 ticks per radian experimentally.
// 60 ticks per revolution of the motor (pre gearbox) and a gear ratio of 4.2941176:1 for 1st rev Magni wheels
// An unexplained constant from the past I leave here till explained is: 17.2328767

#define TICKS_PER_RAD_FROM_GEAR_RATIO   ((double)(4.774556)*(double)(2.0))  // Used to generate ticks per radian 
#define TICKS_PER_RADIAN_DEFAULT        41.004  // For runtime use  getWheelGearRatio() * TICKS_PER_RAD_FROM_GEAR_RATIO    
#define WHEEL_GEAR_RATIO_DEFAULT        WHEEL_GEAR_RATIO_1

// TODO: Make HIGH_SPEED_RADIANS, WHEEL_VELOCITY_NEAR_ZERO and ODOM_4WD_ROTATION_SCALE  all ROS params
#define HIGH_SPEED_RADIANS        (1.8)               // threshold to consider wheel turning 'very fast'
#define WHEEL_VELOCITY_NEAR_ZERO  ((double)(0.08))
#define ODOM_4WD_ROTATION_SCALE   ((double)(1.65))    // Used to correct for 4WD skid rotation error

#define MOTOR_AMPS_PER_ADC_COUNT   ((double)(0.0238)) // 0.1V/Amp  2.44V=1024 count so 41.97 cnt/amp

#define VELOCITY_READ_PER_SECOND   ((double)(10.0))   // read = ticks / (100 ms), so we scale of 10 for ticks/second
#define LOWEST_FIRMWARE_VERSION 28

// Debug verification use only
int32_t  g_odomLeft  = 0;
int32_t  g_odomRight = 0;
int32_t  g_odomEvent = 0;

//lead acid battery percentage levels for a single cell
const static float SLA_AGM[11] = {
    1.800, // 0
    1.837, // 10
    1.875, // 20
    1.912, // 30
    1.950, // 40
    2.010, // 50
    2.025, // 60
    2.062, // 70
    2.100, // 80
    2.137, // 90
    2.175, // 100
};


//li-ion battery percentage levels for a single cell
const static float LI_ION[11] = {
    3.3, // 0
    3.49, // 10
    3.53, // 20
    3.55, // 30
    3.60, // 40
    3.64, // 50
    3.70, // 60
    3.80, // 70
    3.85, // 80
    4.05, // 90
    4.20, // 100
};

// We sometimes need to know if we are rotating in place due to special ways of dealing with
// A 4wd robot must skid to turn. This factor approximates the actual rotation done vs what
// the wheel encoders have indicated.  This only applies if in 4WD mode
double   g_odom4wdRotationScale = ODOM_4WD_ROTATION_SCALE;

// 4WD robot chassis that has to use extensive torque to rotate in place and due to wheel slip has odom scale factor
double   g_radiansLeft  = 0.0;
double   g_radiansRight = 0.0;

// This utility opens and reads 1 or more bytes from a device on an I2C bus
// This method was taken on it's own from a big I2C class we may choose to use later
static int i2c_BufferRead(const char *i2cDevFile, uint8_t i2c8bitAddr,
                          uint8_t *pBuffer, int16_t chipRegAddr, uint16_t NumBytesToRead);


MotorHardware::MotorHardware(const std::shared_ptr<rclcpp::Node>& n, NodeParams node_params, CommsParams serial_params, FirmwareParams firmware_params)
     : diag_updater(n), node(n), logger(rclcpp::get_logger("MotorHardware"))
{
    // Insert a delay prior to serial port setup to avoid a race defect.
    // We see soemtimes the OS sets the port to 115200 baud just after we set it
    RCLCPP_INFO(logger, "Delay before MCB serial port initialization");
    rclcpp::sleep_for(std::chrono::milliseconds(5000)); 

    RCLCPP_INFO(logger, "Initialize MCB serial port '%s' for %d baud",
        serial_params.serial_port.c_str(), serial_params.baud_rate);

    motor_serial_ =
        new MotorSerial(serial_params.serial_port, serial_params.baud_rate);

     RCLCPP_INFO(logger, "MCB serial port initialized");

    // For motor tunning and other uses we publish details for each wheel
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

    sendPid_count = 0;
    num_fw_params = 8;     // number of params sent if any change

    estop_motor_power_off = false;  // Keeps state of ESTOP switch where true is in ESTOP state

    // Save default hardware encoder specifics for ticks in one radian of rotation of main wheel
    this->ticks_per_radian = TICKS_PER_RADIAN_DEFAULT; 
    this->wheel_gear_ratio = WHEEL_GEAR_RATIO_DEFAULT;

    fw_params = firmware_params;

    prev_fw_params.pid_proportional = -1;
    prev_fw_params.pid_integral = -1;
    prev_fw_params.pid_derivative = -1;
    prev_fw_params.pid_velocity = -1;
    prev_fw_params.pid_denominator = -1;
    prev_fw_params.pid_control = -1;
    prev_fw_params.pid_moving_buffer_size = -1;
    prev_fw_params.max_speed_fwd = -1;
    prev_fw_params.max_speed_rev = -1;
    prev_fw_params.deadman_timer = -1;
    prev_fw_params.deadzone_enable = -1;
    prev_fw_params.hw_options = -1;
    prev_fw_params.option_switch = -1;
    prev_fw_params.system_events = -1;
    prev_fw_params.controller_board_version = -1;
    prev_fw_params.estop_detection = -1;
    prev_fw_params.estop_pid_threshold = -1;
    prev_fw_params.max_speed_fwd = -1;
    prev_fw_params.max_speed_rev = -1;
    prev_fw_params.max_pwm = -1;

    hardware_version = 0;
    firmware_version = 0;
    firmware_date    = 0;

    diag_updater.setHardwareID("Motor Controller");
    diag_updater.add("Firmware", &motor_diag_, &MotorDiagnostics::firmware_status);
    diag_updater.add("Limits", &motor_diag_, &MotorDiagnostics::limit_status);
    diag_updater.add("Battery", &motor_diag_, &MotorDiagnostics::battery_status);
    diag_updater.add("MotorPower", &motor_diag_, &MotorDiagnostics::motor_power_status);
    diag_updater.add("PidParamP", &motor_diag_, &MotorDiagnostics::motor_pid_p_status);
    diag_updater.add("PidParamI", &motor_diag_, &MotorDiagnostics::motor_pid_i_status);
    diag_updater.add("PidParamD", &motor_diag_, &MotorDiagnostics::motor_pid_d_status);
    diag_updater.add("PidParamV", &motor_diag_, &MotorDiagnostics::motor_pid_v_status);
    diag_updater.add("PidMaxPWM", &motor_diag_, &MotorDiagnostics::motor_max_pwm_status);
    diag_updater.add("FirmwareOptions", &motor_diag_, &MotorDiagnostics::firmware_options_status);
    diag_updater.add("FirmwareDate", &motor_diag_, &MotorDiagnostics::firmware_date_status);
}

MotorHardware::~MotorHardware() {
    // Cleanup
    closePort();
}

// Implement the necessary methods...

hardware_interface::CallbackReturn MotorHardware::on_init(const hardware_interface::HardwareInfo &info) {
    // Initialize joint names and other variables
    if (info.joints.size() > sizeof(joints_) / sizeof(joints_[0])) {
        RCLCPP_FATAL(logger, "More joints found in configuration than expected.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    for (size_t i = 0; i < info.joints.size(); ++i) {
        const auto &joint = info.joints[i];

        size_t j;

        if (joint.name == "left_wheel_joint"){
            j = 0;
            RCLCPP_INFO(logger, "Setting up left_wheel_joint...");
        } else if (joint.name == "right_wheel_joint"){
            j = 1;
            RCLCPP_INFO(logger, "Setting up right_wheel_joint...");

        } else {
            RCLCPP_WARN(logger, "Joint %s not recognized by motor_node - skipping.", joint.name.c_str());
            continue;
        }


        if (joint.command_interfaces.size() != 1) {
            RCLCPP_FATAL(logger, "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(), joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (joint.state_interfaces.size() != 3) {
            RCLCPP_FATAL(logger, "Joint '%s' has %zu state interfaces found. 3 expected.", joint.name.c_str(), joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        state_interfaces_.emplace_back(joint.name, "position", &joints_[j].position);
        state_interfaces_.emplace_back(joint.name, "velocity", &joints_[j].velocity);
        state_interfaces_.emplace_back(joint.name, "effort", &joints_[j].effort);

        // Register the command interface directly
        command_interfaces_.emplace_back(joint.name, "velocity", &joints_[j].velocity_command);
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorHardware::export_state_interfaces() {
    return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> MotorHardware::export_command_interfaces() {
    return std::move(command_interfaces_);
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

