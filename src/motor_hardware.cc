#include <rclcpp/rclcpp.hpp>
#include <ubiquity_motor_ros2/motor_hardware.h>
#include <ubiquity_motor_ros2/motor_message.h>
#include <boost/assign.hpp>
#include <boost/math/special_functions/round.hpp>
#include <chrono>
#include "pluginlib/class_list_macros.hpp"


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

// MotorHardware::MotorHardware(const std::shared_ptr<rclcpp::Node>& n, NodeParams& node_params, CommsParams& serial_params, FirmwareParams& firmware_params)
//      : wheel_slip_nulling(0), diag_updater(n), node(n), node_params(node_params), serial_params(serial_params), fw_params(firmware_params), logger(rclcpp::get_logger("MotorHardware")),
//      ctrlLoopDelay(node_params->controller_loop_rate), zeroVelocityTime(0, 0), mcbStatusSleepPeriodNs(rclcpp::Duration::from_seconds(0.02).to_chrono<std::chrono::nanoseconds>()), 
//      sysMaintPeriod(rclcpp::Duration::from_seconds(60.0)), jointUpdatePeriod(rclcpp::Duration::from_seconds(0.25)), wheelSlipNullingPeriod(rclcpp::Duration::from_seconds(2.0))
// {
//     // For motor tunning and other uses we publish details for each wheel
//     leftError = n->create_publisher<std_msgs::msg::Int32>("left_error", 10);
//     rightError = n->create_publisher<std_msgs::msg::Int32>("right_error", 10);
//     leftCurrent = n->create_publisher<std_msgs::msg::Float32>("left_current", 10);
//     rightCurrent = n->create_publisher<std_msgs::msg::Float32>("right_current", 10);
//     leftTickInterval = n->create_publisher<std_msgs::msg::Int32>("left_tick_interval", 10);
//     rightTickInterval = n->create_publisher<std_msgs::msg::Int32>("right_tick_interval", 10);
//     firmware_state = n->create_publisher<std_msgs::msg::String>("firmware_state", 10);
//     battery_state = n->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
//     motor_power_active = n->create_publisher<std_msgs::msg::Bool>("motor_power_active", 10);
//     motor_state = n->create_publisher<ubiquity_motor_ros2_msgs::msg::MotorState>("motor_state", 10);

//     sendPid_count = 0;
//     num_fw_params = 8;     // number of params sent if any change

//     estop_motor_power_off = false;  // Keeps state of ESTOP switch where true is in ESTOP state

//     // Save default hardware encoder specifics for ticks in one radian of rotation of main wheel
//     this->ticks_per_radian = TICKS_PER_RADIAN_DEFAULT; 
//     this->wheel_gear_ratio = WHEEL_GEAR_RATIO_DEFAULT;

//     prev_fw_params.pid_proportional = -1;
//     prev_fw_params.pid_integral = -1;
//     prev_fw_params.pid_derivative = -1;
//     prev_fw_params.pid_velocity = -1;
//     prev_fw_params.pid_denominator = -1;
//     prev_fw_params.pid_control = -1;
//     prev_fw_params.pid_moving_buffer_size = -1;
//     prev_fw_params.max_speed_fwd = -1;
//     prev_fw_params.max_speed_rev = -1;
//     prev_fw_params.deadman_timer = -1;
//     prev_fw_params.deadzone_enable = -1;
//     prev_fw_params.hw_options = -1;
//     prev_fw_params.option_switch = -1;
//     prev_fw_params.system_events = -1;
//     prev_fw_params.controller_board_version = -1;
//     prev_fw_params.estop_detection = -1;
//     prev_fw_params.estop_pid_threshold = -1;
//     prev_fw_params.max_speed_fwd = -1;
//     prev_fw_params.max_speed_rev = -1;
//     prev_fw_params.max_pwm = -1;

//     hardware_version = 0;
//     firmware_version = 0;
//     firmware_date    = 0;

//     lastMcbEnabled = 1;
//     wheelSlipEvents = 0;

//     diag_updater->setHardwareID("Motor Controller");
//     diag_updater->add("Firmware", &motor_diag_, &MotorDiagnostics::firmware_status);
//     diag_updater->add("Limits", &motor_diag_, &MotorDiagnostics::limit_status);
//     diag_updater->add("Battery", &motor_diag_, &MotorDiagnostics::battery_status);
//     diag_updater->add("MotorPower", &motor_diag_, &MotorDiagnostics::motor_power_status);
//     diag_updater->add("PidParamP", &motor_diag_, &MotorDiagnostics::motor_pid_p_status);
//     diag_updater->add("PidParamI", &motor_diag_, &MotorDiagnostics::motor_pid_i_status);
//     diag_updater->add("PidParamD", &motor_diag_, &MotorDiagnostics::motor_pid_d_status);
//     diag_updater->add("PidParamV", &motor_diag_, &MotorDiagnostics::motor_pid_v_status);
//     diag_updater->add("PidMaxPWM", &motor_diag_, &MotorDiagnostics::motor_max_pwm_status);
//     diag_updater->add("FirmwareOptions", &motor_diag_, &MotorDiagnostics::firmware_options_status);
//     diag_updater->add("FirmwareDate", &motor_diag_, &MotorDiagnostics::firmware_date_status);


//     // last_loop_time = rclcpp::Clock().now();
//     last_sys_maint_time = rclcpp::Clock().now();
//     last_joint_time = last_sys_maint_time;
//     loopIdx = 0;

//     leftLastWheelPos   = 0.0;
//     rightLastWheelPos  = 0.0;
//     leftWheelPos  = 0.0;
//     rightWheelPos = 0.0;

//     estopReleaseDeadtime = 0.8;
//     estopReleaseDelay    = 0.0;
// }

MotorHardware::MotorHardware()
    : wheel_slip_nulling(0), diag_updater(nullptr), node(nullptr), node_params(nullptr), serial_params(nullptr), fw_params(nullptr), logger(rclcpp::get_logger("MotorHardware")), 
    zeroVelocityTime(0, 0), mcbStatusSleepPeriodNs(rclcpp::Duration::from_seconds(0.02).to_chrono<std::chrono::nanoseconds>()), 
    sysMaintPeriod(rclcpp::Duration::from_seconds(60.0)), jointUpdatePeriod(rclcpp::Duration::from_seconds(0.25)), wheelSlipNullingPeriod(rclcpp::Duration::from_seconds(2.0)) 
{
    sendPid_count = 0;
    num_fw_params = 8;     // number of params sent if any change

    estop_motor_power_off = false;  // Keeps state of ESTOP switch where true is in ESTOP state

    // Save default hardware encoder specifics for ticks in one radian of rotation of main wheel
    this->ticks_per_radian = TICKS_PER_RADIAN_DEFAULT; 
    this->wheel_gear_ratio = WHEEL_GEAR_RATIO_DEFAULT;

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

    lastMcbEnabled = 1;
    wheelSlipEvents = 0;
    
    last_sys_maint_time = rclcpp::Clock().now();
    last_joint_time = last_sys_maint_time;
    loopIdx = 0;

    leftLastWheelPos   = 0.0;
    rightLastWheelPos  = 0.0;
    leftWheelPos  = 0.0;
    rightWheelPos = 0.0;

    estopReleaseDeadtime = 0.8;
    estopReleaseDelay    = 0.0;

    // RCLCPP_INFO(logger, "MotorHardware constructed");

}

void MotorHardware::init(const std::shared_ptr<rclcpp::Node>& n, const std::shared_ptr<NodeParams>& node_params, const std::shared_ptr<CommsParams>& serial_params, const std::shared_ptr<FirmwareParams>& firmware_params){
   
   this->node_params = node_params;
   this->serial_params = serial_params;
   this->fw_params = firmware_params;
   
   
    // For motor tunning and other uses we publish details for each wheel
    leftError = n->create_publisher<std_msgs::msg::Int32>("left_error", 10);
    rightError = n->create_publisher<std_msgs::msg::Int32>("right_error", 10);
    leftCurrent = n->create_publisher<std_msgs::msg::Float32>("left_current", 10);
    rightCurrent = n->create_publisher<std_msgs::msg::Float32>("right_current", 10);
    leftTickInterval = n->create_publisher<std_msgs::msg::Int32>("left_tick_interval", 10);
    rightTickInterval = n->create_publisher<std_msgs::msg::Int32>("right_tick_interval", 10);
    firmware_state = n->create_publisher<std_msgs::msg::String>("firmware_state", 10);
    battery_state = n->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
    motor_power_active = n->create_publisher<std_msgs::msg::Bool>("motor_power_active", 10);
    motor_state = n->create_publisher<ubiquity_motor_ros2_msgs::msg::MotorState>("motor_state", 10);

    diag_updater.reset(new diagnostic_updater::Updater(n));

    diag_updater->setHardwareID("Motor Controller");
    diag_updater->add("Firmware", &motor_diag_, &MotorDiagnostics::firmware_status);
    diag_updater->add("Limits", &motor_diag_, &MotorDiagnostics::limit_status);
    diag_updater->add("Battery", &motor_diag_, &MotorDiagnostics::battery_status);
    diag_updater->add("MotorPower", &motor_diag_, &MotorDiagnostics::motor_power_status);
    diag_updater->add("PidParamP", &motor_diag_, &MotorDiagnostics::motor_pid_p_status);
    diag_updater->add("PidParamI", &motor_diag_, &MotorDiagnostics::motor_pid_i_status);
    diag_updater->add("PidParamD", &motor_diag_, &MotorDiagnostics::motor_pid_d_status);
    diag_updater->add("PidParamV", &motor_diag_, &MotorDiagnostics::motor_pid_v_status);
    diag_updater->add("PidMaxPWM", &motor_diag_, &MotorDiagnostics::motor_max_pwm_status);
    diag_updater->add("FirmwareOptions", &motor_diag_, &MotorDiagnostics::firmware_options_status);
    diag_updater->add("FirmwareDate", &motor_diag_, &MotorDiagnostics::firmware_date_status);
    
    // ctrlLoopDelay = rclcpp::Rate(node_params->controller_loop_rate);

    node = n;

    // Insert a delay prior to serial port setup to avoid a race defect.
    // We see soemtimes the OS sets the port to 115200 baud just after we set it
    RCLCPP_INFO(logger, "Delay before MCB serial port initialization");
    rclcpp::sleep_for(std::chrono::milliseconds(5000)); 

    RCLCPP_INFO(logger, "Initialize MCB serial port '%s' for %d baud",
        serial_params->serial_port.c_str(), serial_params->baud_rate);
    try{
        motor_serial_ =
            new MotorSerial(serial_params->serial_port, serial_params->baud_rate);
        RCLCPP_INFO(logger, "MCB serial port initialized");

    } catch(serial::IOException& e){
        RCLCPP_FATAL(logger, "MCB serial port failed to init: %s", e.what());
    }


    // RCLCPP_INFO(logger, "MotorHardware inited.");

}

MotorHardware::~MotorHardware() {
    // Cleanup
    closePort();
}

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

    RCLCPP_INFO(logger, "MotorHardware on_init done");


    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MotorHardware::export_state_interfaces() {
    return std::move(state_interfaces_);
}

std::vector<hardware_interface::CommandInterface> MotorHardware::export_command_interfaces() {
    return std::move(command_interfaces_);
}

hardware_interface::CallbackReturn MotorHardware::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(logger, "MotorHardware on_activate");

    // Activate the hardware, ensure the motors are ready to receive commands
    // setParams(fw_params); // Don't neeed this sice they are now passed as a reference
    requestFirmwareVersion();
    RCLCPP_INFO(logger, "MotorHardware on_activate - requested FirmwareVersion");

    rclcpp::sleep_for(mcbStatusSleepPeriodNs);

    // Make sure firmware is listening
    {
        diag_updater->broadcast(0, "Establishing communication with motors");
        // Start times counter at 1 to prevent false error print (0 % n = 0)
        int times = 1;
        while (rclcpp::ok() && firmware_version == 0) {
            if (times % 30 == 0){
                RCLCPP_ERROR(logger, "The Firmware not reporting its version");
                requestFirmwareVersion(); // TODO: Should this be out of the if statement? was like that before
            }
            readInputs(0);
            rclcpp::sleep_for(mcbStatusSleepPeriodNs);
            times++;
        }
    }

    if (firmware_version >= MIN_FW_FIRMWARE_DATE) {
        // If supported by firmware also request date code for this version
        RCLCPP_INFO(logger, "Requesting Firmware daycode ");
        requestFirmwareDate();
    }

    // Setup MCB parameters that are defined by host parameters in most cases
    RCLCPP_INFO(logger, "Initializing MCB");
    initMcbParameters();
    RCLCPP_INFO(logger, "Initialization of MCB completed.");


    if (firmware_version >= MIN_FW_SYSTEM_EVENTS) {
        // Start out with zero for system events
        setSystemEvents(0);  // Clear entire system events register
        system_events = 0;
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Send out the refreshable firmware parameters, most are the PID terms
    // We must be sure num_fw_params is set to the modulo used in sendParams()
    for (int i = 0; i < num_fw_params; i++) {
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
        sendParams();
    }


    // Clear any commands the robot has at this time
    clearCommands();

    getWheelJointPositions(leftLastWheelPos, rightWheelPos);

    RCLCPP_INFO(logger, "MotorHardware activated. Motor node starting now.");
    return hardware_interface::CallbackReturn::SUCCESS;
}


// hardware_interface::return_type MotorHardware::start() {
//     // Implementation here
//     return hardware_interface::return_type::OK;
// }

// hardware_interface::return_type MotorHardware::stop() {
//     // Implementation here
//     return hardware_interface::return_type::OK;
// }

hardware_interface::return_type MotorHardware::read(const rclcpp::Time& current_time, const rclcpp::Duration& elapsed_loop_time) {

    // current_time = rclcpp::Clock().now();
    // elapsed_loop_time = current_time - last_loop_time;
    // last_loop_time = current_time;

    RCLCPP_INFO(logger, "MotorHardware read");

    
    if(node == nullptr){
        // Not yet initialized
        return hardware_interface::return_type::OK;
    }

    loopIdx += 1;
    
    manageMotorControllerState();
    // If the motor control is disabled, skip reading
    if (node_params->mcbControlEnabled == 0) {
        return hardware_interface::return_type::OK;
    }

    readInputs(loopIdx);

    checkMcbReset();

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type MotorHardware::write(const rclcpp::Time& current_time, const rclcpp::Duration& elapsed_loop_time) {

    RCLCPP_INFO(logger, "MotorHardware write");


    if(node == nullptr){
        // Not yet initialized
        return hardware_interface::return_type::OK;
    }

    // If the motor control is disabled, skip reading
    if (node_params->mcbControlEnabled == 0) {
        return hardware_interface::return_type::OK;
    }

    setWheelVelocities(current_time, elapsed_loop_time);

    sendParams(); 

    writeMotorSpeeds();

    diag_updater->force_update();

    return hardware_interface::return_type::OK;
}

void MotorHardware::manageMotorControllerState() {
    // Special handling if motor control is disabled. Skip the entire loop
    if (node_params->mcbControlEnabled == 0) {
        // Check for if we are just starting to go into idle mode and release MCB
        if (lastMcbEnabled == 1) {
            RCLCPP_WARN(logger, "Motor controller going offline and closing MCB serial port");
            closePort();
        }
        lastMcbEnabled = 0;
    } else if (lastMcbEnabled == 0) {  // Were disabled but re-enabled, so re-setup MCB
        bool portOpenStatus;
        lastMcbEnabled = 1;
        RCLCPP_WARN(logger, "Motor controller went from offline to online!");
        portOpenStatus = openPort();  // Must re-open serial port
        if (portOpenStatus == true) {
            setSystemEvents(0);  // Clear entire system events register
            system_events = 0;
            rclcpp::sleep_for(mcbStatusSleepPeriodNs);

            // Setup MCB parameters that are defined by host parameters in most cases
            initMcbParameters();
            RCLCPP_WARN(logger, "Motor controller has been re-initialized as we go back online");
        } else {
            // We do not have recovery from this situation, and it seems not possible
            RCLCPP_ERROR(logger, "ERROR in re-opening of the Motor controller!");
        }
    }
}

void MotorHardware::setWheelVelocities(const rclcpp::Time& current_time, const rclcpp::Duration & elapsed_loop_time) {
    // Determine and set wheel velocities in rad/sec from hardware positions in rads
    // rclcpp::Duration elapsed_time = current_time - last_joint_time;
    if (elapsed_loop_time > jointUpdatePeriod) {
        last_joint_time = rclcpp::Clock().now();
        double leftWheelVel  = 0.0;
        double rightWheelVel = 0.0;
        getWheelJointPositions(leftWheelPos, rightWheelPos);
        leftWheelVel  = (leftWheelPos  - leftLastWheelPos)  / elapsed_loop_time.seconds();
        rightWheelVel = (rightWheelPos - rightLastWheelPos) / elapsed_loop_time.seconds();
        setWheelJointVelocities(leftWheelVel, rightWheelVel); // rad/sec
        leftLastWheelPos  = leftWheelPos;
        rightLastWheelPos = rightWheelPos;

        // Publish motor state at this time
        publishMotorState();

        // Implement static wheel slippage relief
        // Deal with auto-null of MCB wheel setpoints if wheel slip nulling is enabled
        // We null wheel torque if wheel speed has been very low for a long time
        // This would be even better if we only did this when over a certain current is heating the wheel
        if (wheel_slip_nulling != 0) {
            if (areWheelSpeedsLower(WHEEL_SLIP_THRESHOLD) != 0) {
                zeroVelocityTime += jointUpdatePeriod;   // add to time at zero velocity
                if (zeroVelocityTime > wheelSlipNullingPeriod) {
                    // null wheel error if at zero velocity for the nulling check period
                    // OPTION: We could also just null wheels at high wheel power
                    RCLCPP_DEBUG(logger, "Applying wheel slip relief now with slip period of %4.1f sec ",
                        wheelSlipNullingPeriod.seconds());
                    wheelSlipEvents += 1;
                    nullWheelErrors();
                    zeroVelocityTime = rclcpp::Duration(0, 0);   // reset time we have been at zero velocity
                }
            } else {
                zeroVelocityTime = rclcpp::Duration(0, 0);   // reset time we have been at zero velocity
            }
        }
    }

}

void MotorHardware::checkMcbReset() {
    // Periodically watch for MCB board having been reset which is an MCB system event
    // This is also a good place to refresh or show status that may have changed
    const rclcpp::Duration elapsed_time = rclcpp::Clock().now() - last_sys_maint_time;
    if ((firmware_version >= MIN_FW_SYSTEM_EVENTS) && (elapsed_time > sysMaintPeriod)) {
        requestSystemEvents();
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
        last_sys_maint_time = rclcpp::Clock().now();

        // See if we are in a low battery voltage state
        std::string batStatus = "OK";
        if (getBatteryVoltage() < fw_params->battery_voltage_low_level) {
            batStatus = "LOW!";
        }

        // Post a status message for MCB state periodically. This may be nice to do more on as required
        RCLCPP_INFO(logger, "Battery = %5.2f volts [%s], MCB system events 0x%x,  PidCtrl 0x%x, WheelType '%s' DriveType '%s' GearRatio %6.3f",
            getBatteryVoltage(), batStatus.c_str(), system_events, getPidControlWord(),
            (wheel_type == MotorMessage::OPT_WHEEL_TYPE_THIN) ? "thin" : "standard",
            node_params->drive_type.c_str(), getWheelGearRatio());

        // If we detect a power-on of MCB we should re-initialize MCB
        if ((system_events & MotorMessage::SYS_EVENT_POWERON) != 0) {
            RCLCPP_WARN(logger, "Detected Motor controller PowerOn event!");
            setSystemEvents(0);  // Clear entire system events register
            system_events = 0;
            rclcpp::sleep_for(mcbStatusSleepPeriodNs);
            
            // Setup MCB parameters that are defined by host parameters in most cases
            initMcbParameters();
            RCLCPP_WARN(logger, "Motor controller has been re-initialized");
        }

        // a periodic refresh of wheel type which is a safety net due to it's importance.
        // This can be removed when a solid message protocol is developed
        if (firmware_version >= MIN_FW_WHEEL_TYPE_THIN) {
            // Refresh the wheel type setting
            setWheelType(wheel_type);
            rclcpp::sleep_for(mcbStatusSleepPeriodNs);
        }
        // a periodic refresh of drive type which is a safety net due to it's importance.
        if (firmware_version >= MIN_FW_DRIVE_TYPE) {
            // Refresh the drive type setting
            setDriveType(drive_type);
            rclcpp::sleep_for(mcbStatusSleepPeriodNs);
        }
    }
}

void MotorHardware::writeMotorSpeeds(){
    // Update motor controller speeds unless global disable is set, perhaps for colision safety
    if (getEstopState()) {
        writeSpeedsInRadians(0.0, 0.0);    // We send zero velocity when estop is active
        estopReleaseDelay = estopReleaseDeadtime;
    } else {
        if (estopReleaseDelay > 0.0) {
            // Implement a delay after estop release where velocity remains zero
            estopReleaseDelay -= (1.0/node_params->controller_loop_rate);
            writeSpeedsInRadians(0.0, 0.0);
        } else {
            if (node_params->mcbSpeedEnabled != 0) {     // A global disable used for safety at node level
                writeSpeeds();         // Normal operation using current system speeds
            } else {
                writeSpeedsInRadians(0.0, 0.0);
            }
        }
    }
}

// Close of the serial port is used in a special case of suspending the motor controller
// so that another service can load firmware or do direct MCB diagnostics
void MotorHardware::closePort() {
    motor_serial_->closePort();
}

// After we have given up the MCB we open serial port again using current instance of Serial
bool MotorHardware::openPort() {
    return motor_serial_->openPort();
}

void MotorHardware::clearCommands() {
    for (size_t i = 0; i < sizeof(joints_) / sizeof(joints_[0]); i++) {
        joints_[i].velocity_command = 0;
    }
}

// Read the current wheel positions in radians both at once for a snapshot of position
void MotorHardware::getWheelJointPositions(double &leftWheelPosition, double &rightWheelPosition) {
    leftWheelPosition  = joints_[WheelJointLocation::Left].position;
    rightWheelPosition = joints_[WheelJointLocation::Right].position;
    return;
}

// Set the current wheel joing velocities in radians/sec both at once for a snapshot of velocity
// This interface is supplied because MotorHardware does not do a loop on it's own
void MotorHardware::setWheelJointVelocities(double leftWheelVelocity, double rightWheelVelocity) {
    joints_[WheelJointLocation::Left].velocity  = leftWheelVelocity;
    joints_[WheelJointLocation::Right].velocity = rightWheelVelocity;
    return;
}

// Publish motor state conditions
void MotorHardware::publishMotorState(void) {
    ubiquity_motor_ros2_msgs::msg::MotorState mstateMsg;

    mstateMsg.header.frame_id = "";   // Could be base_link.  We will use empty till required
    mstateMsg.header.stamp    = node->get_clock()->now();

    mstateMsg.left_position    = joints_[WheelJointLocation::Left].position;
    mstateMsg.right_position   = joints_[WheelJointLocation::Right].position;
    mstateMsg.left_rotate_rate  = joints_[WheelJointLocation::Left].velocity;
    mstateMsg.right_rotate_rate = joints_[WheelJointLocation::Right].velocity;
    mstateMsg.left_current     = motor_diag_.motorCurrentLeft;
    mstateMsg.right_current    = motor_diag_.motorCurrentRight;
    mstateMsg.left_pwm_drive    = motor_diag_.motorPwmDriveLeft;
    mstateMsg.right_pwm_drive   = motor_diag_.motorPwmDriveRight;
    motor_state->publish(mstateMsg);
    return;
}

// readInputs() will receive serial and act on the response from motor controller
//
// The motor controller sends unsolicited messages periodically so we must read the
// messages to update status in near realtime
//
void MotorHardware::readInputs(uint32_t index) {
    while (motor_serial_->commandAvailable()) {
        MotorMessage mm;
        mm = motor_serial_->receiveCommand();
        if (mm.getType() == MotorMessage::TYPE_RESPONSE) {
            switch (mm.getRegister()) {

                case MotorMessage::REG_SYSTEM_EVENTS:
                    if ((mm.getData() & MotorMessage::SYS_EVENT_POWERON) != 0) {
                        RCLCPP_WARN(logger, "Firmware System Event for PowerOn transition");
                        system_events = mm.getData();
                    }
                    break;
                case MotorMessage::REG_FIRMWARE_VERSION:
                    if (mm.getData() < LOWEST_FIRMWARE_VERSION) {
                        RCLCPP_FATAL(logger, "Firmware version %d, expect %d or above",
                                  mm.getData(), LOWEST_FIRMWARE_VERSION);
                        throw std::runtime_error("Firmware version too low");
                    } else {
                        RCLCPP_INFO_ONCE(logger, "Firmware version %d", mm.getData());
                        firmware_version = mm.getData();
                        motor_diag_.firmware_version = firmware_version;
                    }
                    publishFirmwareInfo();
                    break;

                case MotorMessage::REG_FIRMWARE_DATE:
                    // Firmware date is only supported as of fw version MIN_FW_FIRMWARE_DATE
                    RCLCPP_INFO_ONCE(logger, "Firmware date 0x%x (format 0xYYYYMMDD)", mm.getData());
                    firmware_date = mm.getData();
                    motor_diag_.firmware_date = firmware_date;

                    publishFirmwareInfo();
                    break;

                case MotorMessage::REG_BOTH_ODOM: {
                    /* 
                     * ODOM messages from the MCB tell us how far wheels have rotated
                     *
                     * It is here we keep track of wheel joint position 
                     * The odom counts from the MCB are the incremental number of ticks since last report
                     *  WARNING: IF WE LOOSE A MESSAGE WE DRIFT FROM REAL POSITION
                     */
                    int32_t odom = mm.getData();
                    // RCLCPP_ERROR(logger, "odom signed %d", odom);
                    int16_t odomLeft = (odom >> 16) & 0xffff;
                    int16_t odomRight = odom & 0xffff;

                    // Debug code to be used for verification
                    g_odomLeft  += odomLeft;
                    g_odomRight += odomRight;
                    g_odomEvent += 1;
                    //if ((g_odomEvent % 50) == 1) { RCLCPP_ERROR(logger, "leftOdom %d rightOdom %d", g_odomLeft, g_odomRight); }

                    // Due to extreme wheel slip that is required to turn a 4WD robot we are doing a scale factor.
                    // When doing a rotation on the 4WD robot that is in place where linear velocity is zero
                    // we will adjust the odom values for wheel joint rotation using the scale factor.
                    double odom4wdRotationScale = 1.0;

                    // Determine if we are rotating then set a scale to account for rotational wheel slip
                    double near0WheelVel = (double)(WHEEL_VELOCITY_NEAR_ZERO);
                    double leftWheelVel  =  g_radiansLeft;  // rotational speed of motor
                    double rightWheelVel =  g_radiansRight; // rotational speed of motor

                    int leftDir  = (leftWheelVel  >= (double)(0.0)) ? 1 : -1;
                    int rightDir = (rightWheelVel >= (double)(0.0)) ? 1 : -1;
                    int is4wdMode = (fw_params->hw_options & MotorMessage::OPT_DRIVE_TYPE_4WD);
                    if (
                        // Is this in 4wd robot mode
                        (is4wdMode != 0)

                        // Do the joints have Different rotational directions
                        && ((leftDir + rightDir) == 0)

                        // Are Both joints not near joint velocity of 0
                        && ((fabs(leftWheelVel)  > near0WheelVel) && (fabs(rightWheelVel) > near0WheelVel))

                        // Is the difference of the two absolute values of the joint velocities near zero
                        && ((fabs(leftWheelVel) - fabs(rightWheelVel)) < near0WheelVel) )  {

                        odom4wdRotationScale = g_odom4wdRotationScale;
                        if ((index % 16) == 1) {   // This throttles the messages for rotational torque enhancement
                            RCLCPP_INFO(logger, "ROTATIONAL_SCALING_ACTIVE: odom4wdRotationScale = %4.2f [%4.2f, %4.2f] [%d,%d] opt 0x%x 4wd=%d",
                                odom4wdRotationScale, leftWheelVel, rightWheelVel, leftDir, rightDir, fw_params->hw_options, is4wdMode);
                        }
                    } else {
                        if (fabs(leftWheelVel) > near0WheelVel) {
                            RCLCPP_DEBUG(logger, "odom4wdRotationScale = %4.2f [%4.2f, %4.2f] [%d,%d] opt 0x%x 4wd=%d",
                                odom4wdRotationScale, leftWheelVel, rightWheelVel, leftDir, rightDir, fw_params->hw_options, is4wdMode);
                        }
                    }

                    // Add or subtract from position in radians using the incremental odom value
                    joints_[WheelJointLocation::Left].position  +=
                        ((double)odomLeft  / (this->ticks_per_radian * odom4wdRotationScale));
                    joints_[WheelJointLocation::Right].position +=
                        ((double)odomRight / (this->ticks_per_radian * odom4wdRotationScale));

                    motor_diag_.odom_update_status.tick(); // Let diag know we got odom

                    break;
                }
                case MotorMessage::REG_BOTH_ERROR: {
                    std_msgs::msg::Int32 left;
                    std_msgs::msg::Int32 right;
                    int32_t speed = mm.getData();
                    int16_t leftSpeed = (speed >> 16) & 0xffff;
                    int16_t rightSpeed = speed & 0xffff;

                    left.data = leftSpeed;
                    right.data = rightSpeed;
                    leftError->publish(left);
                    rightError->publish(right);
                    break;
                }

                case MotorMessage::REG_PWM_BOTH_WHLS: {
                    int32_t bothPwm = mm.getData();
                    motor_diag_.motorPwmDriveLeft  = (bothPwm >> 16) & 0xffff;
                    motor_diag_.motorPwmDriveRight = bothPwm & 0xffff;
                    break;
                }

                case MotorMessage::REG_LEFT_CURRENT: {
                    // Motor current is an absolute value and goes up from a nominal count of near 1024
                    // So we subtract a nominal offset then multiply count * scale factor to get amps
                    int32_t data = mm.getData() & 0xffff;
                    motor_diag_.motorCurrentLeft =
                        (double)(data - motor_diag_.motorAmpsZeroAdcCount) * MOTOR_AMPS_PER_ADC_COUNT;
                    break;
                }
                case MotorMessage::REG_RIGHT_CURRENT: {
                    // Motor current is an absolute value and goes up from a nominal count of near 1024
                    // So we subtract a nominal offset then multiply count * scale factor to get amps
                    int32_t data = mm.getData() & 0xffff;
                    motor_diag_.motorCurrentRight =
                        (double)(data - motor_diag_.motorAmpsZeroAdcCount) * MOTOR_AMPS_PER_ADC_COUNT;
                    break;
                }

                case MotorMessage::REG_HW_OPTIONS: {
                    int32_t data = mm.getData();

                    // Enable or disable hardware options reported from firmware
                    motor_diag_.firmware_options = data;

                    // Set radians per encoder tic based on encoder specifics
                    if (data & MotorMessage::OPT_ENC_6_STATE) {
                        RCLCPP_WARN_ONCE(logger, "Encoder Resolution: 'Enhanced'");
                        fw_params->hw_options |= MotorMessage::OPT_ENC_6_STATE;
                        this->ticks_per_radian = this->getWheelTicksPerRadian();
                    } else {
                        RCLCPP_WARN_ONCE(logger, "Encoder Resolution: 'Standard'");
                        fw_params->hw_options &= ~MotorMessage::OPT_ENC_6_STATE;
                        this->ticks_per_radian  = this->getWheelTicksPerRadian() / (double)(2.0);
                    }

                    if (data & MotorMessage::OPT_WHEEL_TYPE_THIN) {
                        RCLCPP_WARN_ONCE(logger, "Wheel type is: 'thin'");
                        fw_params->hw_options |= MotorMessage::OPT_WHEEL_TYPE_THIN;
                    } else {
                        RCLCPP_WARN_ONCE(logger, "Wheel type is: 'standard'");
                        fw_params->hw_options &= ~MotorMessage::OPT_WHEEL_TYPE_THIN;
                    }

                    if (data & MotorMessage::OPT_DRIVE_TYPE_4WD) {
                        RCLCPP_WARN_ONCE(logger, "Drive type is: '4wd'");
                        fw_params->hw_options |= MotorMessage::OPT_DRIVE_TYPE_4WD;
                    } else {
                        RCLCPP_WARN_ONCE(logger, "Drive type is: '2wd'");
                        fw_params->hw_options &= ~MotorMessage::OPT_DRIVE_TYPE_4WD;
                    }

                    if (data & MotorMessage::OPT_WHEEL_DIR_REVERSE) {
                        RCLCPP_WARN_ONCE(logger, "Wheel direction is: 'reverse'");
                        fw_params->hw_options |= MotorMessage::OPT_WHEEL_DIR_REVERSE;
                    } else {
                        RCLCPP_WARN_ONCE(logger, "Wheel direction is: 'standard'");
                        fw_params->hw_options &= ~MotorMessage::OPT_WHEEL_DIR_REVERSE;
                    }
                    break;
                }

                case MotorMessage::REG_LIMIT_REACHED: {
                    int32_t data = mm.getData();

                    if (data & MotorMessage::LIM_M1_PWM) {
                        RCLCPP_WARN(logger, "left PWM limit reached");
                            motor_diag_.left_pwm_limit = true;
                    }
                    if (data & MotorMessage::LIM_M2_PWM) {
                        RCLCPP_WARN(logger, "right PWM limit reached");
                            motor_diag_.right_pwm_limit = true;
                    }
                    if (data & MotorMessage::LIM_M1_INTEGRAL) {
                        RCLCPP_DEBUG(logger, "left Integral limit reached");
                            motor_diag_.left_integral_limit = true;
                    }
                    if (data & MotorMessage::LIM_M2_INTEGRAL) {
                        RCLCPP_DEBUG(logger, "right Integral limit reached");
                            motor_diag_.right_integral_limit = true;
                    }
                    if (data & MotorMessage::LIM_M1_MAX_SPD) {
                        RCLCPP_WARN(logger, "left Maximum speed reached");
                            motor_diag_.left_max_speed_limit = true;
                    }
                    if (data & MotorMessage::LIM_M2_MAX_SPD) {
                        RCLCPP_WARN(logger, "right Maximum speed reached");
                            motor_diag_.right_max_speed_limit = true;
                    }
                    if (data & MotorMessage::LIM_PARAM_LIMIT) {
                        RCLCPP_WARN_ONCE(logger, "parameter limit in firmware");
                            motor_diag_.param_limit_in_firmware = true;
                    }
                    break;
                }
                case MotorMessage::REG_BATTERY_VOLTAGE: {
                    int32_t data = mm.getData();
                    sensor_msgs::msg::BatteryState bstate;
                    bstate.voltage = (float)data * fw_params->battery_voltage_multiplier +
                                   fw_params->battery_voltage_offset;
                    bstate.current = std::numeric_limits<float>::quiet_NaN();
                    bstate.charge = std::numeric_limits<float>::quiet_NaN();
                    bstate.capacity = std::numeric_limits<float>::quiet_NaN();
                    bstate.design_capacity = std::numeric_limits<float>::quiet_NaN();

                    // Hardcoded to a sealed lead acid 12S battery, but adjustable for future use
                    bstate.percentage = calculateBatteryPercentage(bstate.voltage, 12, SLA_AGM);
                    bstate.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
                    bstate.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
                    bstate.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
                    battery_state->publish(bstate);

                    motor_diag_.battery_voltage = bstate.voltage;
                    motor_diag_.battery_voltage_low_level = MotorHardware::fw_params->battery_voltage_low_level;
                    motor_diag_.battery_voltage_critical = MotorHardware::fw_params->battery_voltage_critical;
                    break;
                }
                case MotorMessage::REG_MOT_PWR_ACTIVE: {   // Starting with rev 5.0 board we can see power state
                    int32_t data = mm.getData();

                    if (data & MotorMessage::MOT_POW_ACTIVE) {
                        if (estop_motor_power_off == true) {
                            RCLCPP_WARN(logger, "Motor power has gone from inactive to active. Most likely from ESTOP switch");
                        }
                        estop_motor_power_off = false;
                    } else {
                        if (estop_motor_power_off == false) {
                            RCLCPP_WARN(logger, "Motor power has gone inactive. Most likely from ESTOP switch active");
                        }
                        estop_motor_power_off = true;
                    }
                    motor_diag_.estop_motor_power_off = estop_motor_power_off;  // A copy for diagnostics topic

                    std_msgs::msg::Bool estop_message;
                    estop_message.data = !estop_motor_power_off;
                    motor_power_active->publish(estop_message);
                    // TODO: Should this really fall through?
                }

                case MotorMessage::REG_TINT_BOTH_WHLS: {   // As of v41 show time between wheel enc edges
                    int32_t data = mm.getData();
                    uint16_t leftTickSpacing = (data >> 16) & 0xffff;
                    uint16_t rightTickSpacing = data & 0xffff;
                    uint16_t tickCap = 0;    // We can cap the max value if desired

                    if ((tickCap > 0) && (leftTickSpacing  > tickCap)) { leftTickSpacing  = tickCap; }
                    if ((tickCap > 0) && (rightTickSpacing > tickCap)) { rightTickSpacing = tickCap; }

                    // Publish the two wheel tic intervals
                    std_msgs::msg::Int32 leftInterval;
                    std_msgs::msg::Int32 rightInterval;

                    leftInterval.data  = leftTickSpacing;
                    rightInterval.data = rightTickSpacing;

                    // Only publish the tic intervals when wheels are moving
                    if (data > 1) {     // Optionally show the intervals for debug
                        leftTickInterval->publish(leftInterval);
                        rightTickInterval->publish(rightInterval);

                        RCLCPP_DEBUG(logger, "Tic Ints M1 %d [0x%x]  M2 %d [0x%x]",  
                            leftTickSpacing, leftTickSpacing, rightTickSpacing, rightTickSpacing);
                    }
                }
                default:
                    break;
            }
        }
    }
}


void MotorHardware::publishFirmwareInfo(){
    //publish the firmware version and optional date to ROS
    if(firmware_version > 0){

        std_msgs::msg::String fstate;
        fstate.data = "v"+std::to_string(firmware_version);

        if(firmware_date > 0){
            std::stringstream stream;
            stream << std::hex << firmware_date;
            std::string daycode(stream.str());

            fstate.data +=" "+daycode;
        }

        firmware_state->publish(fstate);
    }
}

// calculateBatteryPercentage() takes in battery voltage, number of cells, and type; returns approximate percentage
//
// A battery type is defined by an array of 11 values, each corresponding to one 10% sized step from 0% to 100%.
// If the values fall between the steps, they are linearly interpolated to give a more accurate reading.
//
float MotorHardware::calculateBatteryPercentage(float voltage, int cells, const float* type) {
    float onecell = voltage / (float)cells;

    if(onecell >= type[10])
        return 1.0;
    else if(onecell <= type[0])
        return 0.0;

    int upper = 0;
    int lower = 0;

    for(int i = 0; i < 11; i++){
        if(onecell > type[i]){
            lower = i;
        }else{
            upper = i;
            break;
        }
    }

    float deltavoltage = type[upper] - type[lower];
    float between_percent = (onecell - type[lower]) / deltavoltage;

    return (float)lower * 0.1 + between_percent * 0.1;
}

// writeSpeedsInRadians()  Take in radians per sec for wheels and send in message to controller
//
// A direct write speeds that allows caller setting speeds in radians
// This interface allows maintaining of system speed in state but override to zero
// which is of value for such a case as ESTOP implementation
//
void MotorHardware::writeSpeedsInRadians(double  left_radians, double  right_radians) {
    MotorMessage both;
    both.setRegister(MotorMessage::REG_BOTH_SPEED_SET);
    both.setType(MotorMessage::TYPE_WRITE);

    g_radiansLeft  = left_radians;
    g_radiansRight = right_radians;

    // We are going to implement a message when robot is moving very fast or rotating very fast
    if (((left_radians / VELOCITY_READ_PER_SECOND)  > HIGH_SPEED_RADIANS) || 
        ((right_radians / VELOCITY_READ_PER_SECOND) > HIGH_SPEED_RADIANS)) {
        RCLCPP_WARN(logger, "Wheel rotation at high radians per sec.  Left %f rad/s Right %f rad/s",
            left_radians, right_radians);
    }

    int16_t left_speed  = calculateSpeedFromRadians(left_radians);
    int16_t right_speed = calculateSpeedFromRadians(right_radians);

    // The masking with 0x0000ffff is necessary for handling -ve numbers
    int32_t data = (left_speed << 16) | (right_speed & 0x0000ffff);
    both.setData(data);

    std_msgs::msg::Int32 smsg;
    smsg.data = left_speed;

    motor_serial_->transmitCommand(both);

    // RCLCPP_ERROR(logger, "velocity_command %f rad/s %f rad/s",
    // joints_[WheelJointLocation::Left].velocity_command, joints_[WheelJointLocation::Right].velocity_command);
    // joints_[LEFT_WHEEL_JOINT].velocity_command, joints_[RIGHT_WHEEL_JOINT].velocity_command);
    // RCLCPP_ERROR(logger, "SPEEDS %d %d", left.getData(), right.getData());
}

// writeSpeeds()  Take in radians per sec for wheels and send in message to controller
//
// Legacy interface where no speed overrides are supported
//
void MotorHardware::writeSpeeds() {
    // This call pulls in speeds from the joints array maintained by other layers

    double  left_radians  = joints_[WheelJointLocation::Left].velocity_command;
    double  right_radians = joints_[WheelJointLocation::Right].velocity_command;

    writeSpeedsInRadians(left_radians, right_radians);
}

// areWheelSpeedsLower()  Determine if all wheel joint speeds are below given threshold
//
int MotorHardware::areWheelSpeedsLower(double wheelSpeedRadPerSec) {
    int retCode = 0;

    // This call pulls in speeds from the joints array maintained by other layers

    double  left_radians  = joints_[WheelJointLocation::Left].velocity_command;
    double  right_radians = joints_[WheelJointLocation::Right].velocity_command;

    if ((std::abs(left_radians)  < wheelSpeedRadPerSec) &&
        (std::abs(right_radians) < wheelSpeedRadPerSec)) {
        retCode = 1;
    }

    return retCode;
}

void MotorHardware::requestFirmwareVersion() {
    MotorMessage fw_version_msg;
    fw_version_msg.setRegister(MotorMessage::REG_FIRMWARE_VERSION);
    fw_version_msg.setType(MotorMessage::TYPE_READ);
    fw_version_msg.setData(0);
    motor_serial_->transmitCommand(fw_version_msg);
}

// Firmware date register implemented as of MIN_FW_FIRMWARE_DATE
void MotorHardware::requestFirmwareDate() {
    MotorMessage fw_date_msg;
    fw_date_msg.setRegister(MotorMessage::REG_FIRMWARE_DATE);
    fw_date_msg.setType(MotorMessage::TYPE_READ);
    fw_date_msg.setData(0);
    motor_serial_->transmitCommand(fw_date_msg);
}

// Request the MCB system event register
void MotorHardware::requestSystemEvents() {
    MotorMessage sys_event_msg;
    sys_event_msg.setRegister(MotorMessage::REG_SYSTEM_EVENTS);
    sys_event_msg.setType(MotorMessage::TYPE_READ);
    sys_event_msg.setData(0);
    motor_serial_->transmitCommand(sys_event_msg);
}

// Read the wheel currents in amps
void MotorHardware::getMotorCurrents(double &currentLeft, double &currentRight) {
    currentLeft  = motor_diag_.motorCurrentLeft;
    currentRight = motor_diag_.motorCurrentRight;
    return;
}


// Due to greatly limited pins on the firmware processor the host figures out the hardware rev and sends it to fw
// The hardware version is 0x0000MMmm  where MM is major rev like 4 and mm is minor rev like 9 for first units.
// The 1st firmware version this is set for is 32, before it was always 1
void MotorHardware::setHardwareVersion(int32_t hardware_version) {
    RCLCPP_INFO(logger, "setting hardware_version to %x", (int)hardware_version);
    this->hardware_version = hardware_version;
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_HARDWARE_VERSION);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(hardware_version);
    motor_serial_->transmitCommand(mm);
}

// Setup the controller board threshold to put into force estop protection on boards prior to rev 5.0 with hardware support
void MotorHardware::setEstopPidThreshold(int32_t estop_pid_threshold) {
    RCLCPP_INFO(logger, "setting Estop PID threshold to %d", (int)estop_pid_threshold);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_PID_MAX_ERROR);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(estop_pid_threshold);
    motor_serial_->transmitCommand(mm);
}

// Setup the controller board to have estop button state detection feature enabled or not
void MotorHardware::setEstopDetection(int32_t estop_detection) {
    RCLCPP_INFO(logger, "setting estop button detection to %x", (int)estop_detection);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_ESTOP_ENABLE);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(estop_detection);
    motor_serial_->transmitCommand(mm);
}

// Returns true if estop switch is active OR if motor power is off somehow off
bool MotorHardware::getEstopState(void) {
    return estop_motor_power_off;
}

// Setup the controller board maximum settable motor forward speed
void MotorHardware::setMaxFwdSpeed(int32_t max_speed_fwd) {
    RCLCPP_INFO(logger, "setting max motor forward speed to %d", (int)max_speed_fwd);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_MAX_SPEED_FWD);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(max_speed_fwd);
    motor_serial_->transmitCommand(mm);
}

// Setup the Wheel Type. Overrides mode in use on hardware
// This used to only be standard but THIN_WHEELS were added in Jun 2020
void MotorHardware::setWheelType(int32_t new_wheel_type) {

    MotorMessage ho;
    switch(new_wheel_type) {
        case MotorMessage::OPT_WHEEL_TYPE_STANDARD:
        case MotorMessage::OPT_WHEEL_TYPE_THIN:
            RCLCPP_INFO_ONCE(logger, "setting MCB wheel type %d", (int)new_wheel_type);
            wheel_type = new_wheel_type;
            ho.setRegister(MotorMessage::REG_WHEEL_TYPE);
            ho.setType(MotorMessage::TYPE_WRITE);
            ho.setData(wheel_type);
            motor_serial_->transmitCommand(ho);
            break;
        default:
            RCLCPP_ERROR(logger, "Illegal MCB wheel type 0x%x will not be set!", (int)new_wheel_type);
    }
}

// A simple fetch of the wheel_gear_ratio 
double MotorHardware::getWheelGearRatio(void) {
    return this->wheel_gear_ratio;
}

// A simple fetch of the encoder ticks per radian of wheel rotation
double MotorHardware::getWheelTicksPerRadian(void) {
    double result = this->getWheelGearRatio() * TICKS_PER_RAD_FROM_GEAR_RATIO;
    return result;
}

// Setup the local Wheel gear ratio so the motor hardware layer can adjust wheel odom reporting
// This gear ratio was introduced for a new version of the standard wheels in late 2021 production
// This is slightly more complex in that it is compounded with the now default 6 state encoder hw option
void MotorHardware::setWheelGearRatio(double new_wheel_gear_ratio) {
    // This gear ratio is not used by the firmware so it is a simple state element in this module
    this->wheel_gear_ratio = new_wheel_gear_ratio;
    this->ticks_per_radian  = this->getWheelTicksPerRadian();   // Need to also reset ticks_per_radian
    if ((fw_params->hw_options & MotorMessage::OPT_ENC_6_STATE) == 0) {
        this->ticks_per_radian = this->ticks_per_radian / (double)(2.0);   // 3 state was half
    }
    RCLCPP_INFO(logger, "Setting Wheel gear ratio to %6.4f and tics_per_radian to %6.4f",
        this->wheel_gear_ratio, this->ticks_per_radian);
}

// Setup the Drive Type. Overrides mode in use on hardware
// This used to only be 2WD and use of THIN_WHEELS set 4WD
// We are not trying to decouple wheel type from drive type
// This register always existed but was a do nothing till firmware v42
void MotorHardware::setDriveType(int32_t drive_type) {
    RCLCPP_INFO_ONCE(logger, "setting MCB drive type %d", (int)drive_type);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_DRIVE_TYPE);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(drive_type);
    motor_serial_->transmitCommand(mm);
}

// Setup the PID control options. Overrides modes in use on hardware
void MotorHardware::setPidControl(int32_t pid_control_word) {
    RCLCPP_INFO_ONCE(logger, "setting MCB pid control word to 0x%x", (int)pid_control_word);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_PID_CONTROL);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(pid_control_word);
    motor_serial_->transmitCommand(mm);
}

// Do a one time NULL of the wheel setpoint based on current position error
// This allows to relieve stress in static situation where wheels cannot slip to match setpoint
void MotorHardware::nullWheelErrors(void) {
    RCLCPP_DEBUG(logger, "Nulling MCB wheel errors using current wheel positions");
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_WHEEL_NULL_ERR);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(MotorOrWheelNumber::Motor_M1|MotorOrWheelNumber::Motor_M2);
    motor_serial_->transmitCommand(mm);
}

// Setup the Wheel direction. Overrides mode in use on hardware
// This allows for customer to install wheels on cutom robots as they like
void MotorHardware::setWheelDirection(int32_t wheel_direction) {
    RCLCPP_INFO(logger, "setting MCB wheel direction to %d", (int)wheel_direction);
    MotorMessage ho;
    ho.setRegister(MotorMessage::REG_WHEEL_DIR);
    ho.setType(MotorMessage::TYPE_WRITE);
    ho.setData(wheel_direction);
    motor_serial_->transmitCommand(ho);
}

// A simple fetch of the pid_control word from firmware params
int MotorHardware::getPidControlWord(void) {
    int pidControlWord;
    pidControlWord = motor_diag_.fw_pid_control;
    return pidControlWord;
}

// Read the controller board option switch itself that resides on the I2C bus but is on the MCB
// This call inverts the bits because a shorted option switch is a 0 where we want it as 1
// If return is negative something went wrong
int MotorHardware::getOptionSwitch(void) {
    uint8_t buf[16];
    int retBits = 0;
    RCLCPP_INFO(logger, "reading MCB option switch on the I2C bus");
    int retCount = i2c_BufferRead(I2C_DEVICE, I2C_PCF8574_8BIT_ADDR, &buf[0], -1, 1);
    if (retCount < 0) {
        RCLCPP_ERROR(logger, "Error %d in reading MCB option switch at 8bit Addr 0x%x",
            retCount, I2C_PCF8574_8BIT_ADDR);
        retBits = retCount;
    } else if (retCount != 1) {
        RCLCPP_ERROR(logger, "Cannot read byte from MCB option switch at 8bit Addr 0x%x", I2C_PCF8574_8BIT_ADDR);
        retBits = -1;
    } else {
        retBits = (0xff) & ~buf[0];
    }

    return retBits;
}

// Setup the controller board option switch register which comes from the I2C 8-bit IO chip on MCB
void MotorHardware::setOptionSwitchReg(int32_t option_switch_bits) {
    RCLCPP_INFO(logger, "setting MCB option switch register to 0x%x", (int)option_switch_bits);
    MotorMessage os;
    os.setRegister(MotorMessage::REG_OPTION_SWITCH);
    os.setType(MotorMessage::TYPE_WRITE);
    os.setData(option_switch_bits);
    motor_serial_->transmitCommand(os);
}

// Setup the controller board system event register or clear bits in the register
void MotorHardware::setSystemEvents(int32_t system_events) {
    RCLCPP_INFO(logger, "setting MCB system event register to %d", (int)system_events);
    MotorMessage se;
    se.setRegister(MotorMessage::REG_SYSTEM_EVENTS);
    se.setType(MotorMessage::TYPE_WRITE);
    se.setData(system_events);
    motor_serial_->transmitCommand(se);
}

// Setup the controller board maximum settable motor reverse speed
void MotorHardware::setMaxRevSpeed(int32_t max_speed_rev) {
    RCLCPP_INFO(logger, "setting max motor reverse speed to %d", (int)max_speed_rev);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_MAX_SPEED_REV);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(max_speed_rev);
    motor_serial_->transmitCommand(mm);
}

// Setup the controller board maximum PWM level allowed for a motor
void MotorHardware::setMaxPwm(int32_t max_pwm) {
    RCLCPP_INFO(logger, "setting max motor PWM to %x", (int)max_pwm);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_MAX_PWM);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(max_pwm);
    motor_serial_->transmitCommand(mm);
}

void MotorHardware::setDeadmanTimer(int32_t deadman_timer) {
    RCLCPP_ERROR(logger, "setting deadman to %d", (int)deadman_timer);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_DEADMAN);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(deadman_timer);
    motor_serial_->transmitCommand(mm);
}

void MotorHardware::setDeadzoneEnable(int32_t deadzone_enable) {
    RCLCPP_ERROR(logger, "setting deadzone enable to %d", (int)deadzone_enable);
    MotorMessage mm;
    mm.setRegister(MotorMessage::REG_DEADZONE);
    mm.setType(MotorMessage::TYPE_WRITE);
    mm.setData(deadman_timer);
    motor_serial_->transmitCommand(mm);
}

void MotorHardware::setParams(FirmwareParams fp) {
    fw_params->pid_proportional = fp.pid_proportional;
    fw_params->pid_integral = fp.pid_integral;
    fw_params->pid_derivative = fp.pid_derivative;
    fw_params->pid_velocity = fp.pid_velocity;
    fw_params->pid_denominator = fp.pid_denominator;
    fw_params->pid_moving_buffer_size = fp.pid_moving_buffer_size;
    fw_params->pid_denominator = fp.pid_denominator;
    fw_params->pid_control = fp.pid_control;
    fw_params->max_pwm = fp.max_pwm;
    fw_params->estop_pid_threshold = fp.estop_pid_threshold;
}

// Forces next calls to sendParams() to always update each parameter.
// KEEP THIS IN SYNC WITH CHANGES TO sendParams()
void MotorHardware::forcePidParamUpdates() {

    // Reset each of the flags that causes parameters to be  sent to MCB by sendParams()
    prev_fw_params.pid_proportional = -1;
    prev_fw_params.pid_integral = -1;
    prev_fw_params.pid_derivative = -1;
    prev_fw_params.pid_velocity = -1;
    prev_fw_params.pid_denominator = -1;
    prev_fw_params.pid_moving_buffer_size = -1;
    prev_fw_params.max_pwm = -1;
    prev_fw_params.pid_control = 1;
}

void MotorHardware::sendParams() {
    std::vector<MotorMessage> commands;

    // RCLCPP_ERROR(logger, "sending PID %d %d %d %d",
    //(int)p_value, (int)i_value, (int)d_value, (int)denominator_value);

    // Only send one register at a time to avoid overwhelming serial comms
    // SUPPORT NOTE!  Adjust modulo for total parameters in the cycle
    //                and be sure no duplicate modulos are used!
    int cycle = (sendPid_count++) % num_fw_params;     // MUST BE THE TOTAL NUMBER IN THIS HANDLING

    if (cycle == 0 &&
        fw_params->pid_proportional != prev_fw_params.pid_proportional) {
        RCLCPP_WARN(logger, "Setting PidParam P to %d", fw_params->pid_proportional);
        prev_fw_params.pid_proportional = fw_params->pid_proportional;
        motor_diag_.fw_pid_proportional = fw_params->pid_proportional;
        MotorMessage p;
        p.setRegister(MotorMessage::REG_PARAM_P);
        p.setType(MotorMessage::TYPE_WRITE);
        p.setData(fw_params->pid_proportional);
        commands.push_back(p);
    }

    if (cycle == 1 && fw_params->pid_integral != prev_fw_params.pid_integral) {
        RCLCPP_WARN(logger, "Setting PidParam I to %d", fw_params->pid_integral);
        prev_fw_params.pid_integral = fw_params->pid_integral;
        motor_diag_.fw_pid_integral = fw_params->pid_integral;
        MotorMessage i;
        i.setRegister(MotorMessage::REG_PARAM_I);
        i.setType(MotorMessage::TYPE_WRITE);
        i.setData(fw_params->pid_integral);
        commands.push_back(i);
    }

    if (cycle == 2 &&
        fw_params->pid_derivative != prev_fw_params.pid_derivative) {
        RCLCPP_WARN(logger, "Setting PidParam D to %d", fw_params->pid_derivative);
        prev_fw_params.pid_derivative = fw_params->pid_derivative;
        motor_diag_.fw_pid_derivative = fw_params->pid_derivative;
        MotorMessage d;
        d.setRegister(MotorMessage::REG_PARAM_D);
        d.setType(MotorMessage::TYPE_WRITE);
        d.setData(fw_params->pid_derivative);
        commands.push_back(d);
    }

    if (cycle == 3 && (motor_diag_.firmware_version >= MIN_FW_PID_V_TERM) &&
        fw_params->pid_velocity != prev_fw_params.pid_velocity) {
        RCLCPP_WARN(logger, "Setting PidParam V to %d", fw_params->pid_velocity);
        prev_fw_params.pid_velocity = fw_params->pid_velocity;
        motor_diag_.fw_pid_velocity = fw_params->pid_velocity;
        MotorMessage v;
        v.setRegister(MotorMessage::REG_PARAM_V);
        v.setType(MotorMessage::TYPE_WRITE);
        v.setData(fw_params->pid_velocity);
        commands.push_back(v);
    }

    if (cycle == 4 &&
        fw_params->pid_denominator != prev_fw_params.pid_denominator) {
        RCLCPP_WARN(logger, "Setting PidParam Denominator to %d", fw_params->pid_denominator);
        prev_fw_params.pid_denominator = fw_params->pid_denominator;
        motor_diag_.fw_pid_denominator = fw_params->pid_denominator;
        MotorMessage denominator;
        denominator.setRegister(MotorMessage::REG_PARAM_C);
        denominator.setType(MotorMessage::TYPE_WRITE);
        denominator.setData(fw_params->pid_denominator);
        commands.push_back(denominator);
    }

    if (cycle == 5 &&
        fw_params->pid_moving_buffer_size !=
            prev_fw_params.pid_moving_buffer_size) {
        RCLCPP_WARN(logger, "Setting PidParam D window to %d", fw_params->pid_moving_buffer_size);
        prev_fw_params.pid_moving_buffer_size =
            fw_params->pid_moving_buffer_size;
        motor_diag_.fw_pid_moving_buffer_size = fw_params->pid_moving_buffer_size;
        MotorMessage winsize;
        winsize.setRegister(MotorMessage::REG_MOVING_BUF_SIZE);
        winsize.setType(MotorMessage::TYPE_WRITE);
        winsize.setData(fw_params->pid_moving_buffer_size);
        commands.push_back(winsize);
    }

    if (cycle == 6 &&
        fw_params->max_pwm != prev_fw_params.max_pwm) {
        RCLCPP_WARN(logger, "Setting PidParam max_pwm to %d", fw_params->max_pwm);
        prev_fw_params.max_pwm = fw_params->max_pwm;
        motor_diag_.fw_max_pwm = fw_params->max_pwm;
        MotorMessage maxpwm;
        maxpwm.setRegister(MotorMessage::REG_MAX_PWM);
        maxpwm.setType(MotorMessage::TYPE_WRITE);
        maxpwm.setData(fw_params->max_pwm);
        commands.push_back(maxpwm);
    }

    if (cycle == 7 &&
        fw_params->pid_control != prev_fw_params.pid_control) {
        RCLCPP_WARN(logger, "Setting PidParam pid_control to %d", fw_params->pid_control);
        prev_fw_params.pid_control = fw_params->pid_control;
        motor_diag_.fw_pid_control = fw_params->pid_control;
        MotorMessage mmsg;
        mmsg.setRegister(MotorMessage::REG_PID_CONTROL);
        mmsg.setType(MotorMessage::TYPE_WRITE);
        mmsg.setData(fw_params->pid_control);
        commands.push_back(mmsg);
    }

    // SUPPORT NOTE!  Adjust max modulo for total parameters in the cycle, be sure no duplicates used!

    if (commands.size() != 0) {
        motor_serial_->transmitCommands(commands);
    }
}

// Get current battery voltage
float MotorHardware::getBatteryVoltage(void) {
    return motor_diag_.battery_voltage;   // We keep battery_voltage in diagnostic context
}

void MotorHardware::setDebugLeds(bool led_1, bool led_2) {
    std::vector<MotorMessage> commands;

    MotorMessage led1;
    led1.setRegister(MotorMessage::REG_LED_1);
    led1.setType(MotorMessage::TYPE_WRITE);
    if (led_1) {
        led1.setData(0x00000001);
    } else {
        led1.setData(0x00000000);
    }
    commands.push_back(led1);

    MotorMessage led2;
    led2.setRegister(MotorMessage::REG_LED_2);
    led2.setType(MotorMessage::TYPE_WRITE);
    if (led_2) {
        led2.setData(0x00000001);
    } else {
        led2.setData(0x00000000);
    }
    commands.push_back(led2);

    motor_serial_->transmitCommands(commands);
}

// calculate the binary speed value sent to motor controller board
// using an input expressed in radians.
// The firmware uses the same speed value no matter what type of encoder is used
int16_t MotorHardware::calculateSpeedFromRadians(double radians) {
    int16_t speed;
    double  encoderFactor = 1.0;
    double  speedFloat;

    // The firmware accepts same units for speed value
    // and will deal with it properly depending on encoder handling in use
    if (fw_params->hw_options & MotorMessage::OPT_ENC_6_STATE) {
        encoderFactor = (double)(0.5);
    }

    speedFloat = encoderFactor * radians * ((getWheelTicksPerRadian() * (double)(4.0)) / VELOCITY_READ_PER_SECOND);
    speed =  boost::math::iround(speedFloat);

    return speed;
}

double MotorHardware::calculateRadiansFromTicks(int16_t ticks) {
    double result;

    result =  ((double)ticks * VELOCITY_READ_PER_SECOND) / (getWheelTicksPerRadian() * (double)(4.0));
    return result;
}

// i2c_BufferRead()   A host OS system specific  utility to open, read, close from an I2C device
//
// The I2C address is the 8-bit address which is the 7-bit addr shifted left in some code
// If chipRegAddr is greater than 1 we write this out for the internal chip address for the following read(s)
//
// Returns number of bytes read where 0 or less implies some form of failure
//
// NOTE: The i2c8bitAddr will be shifted right one bit to use as 7-bit I2C addr
//
int MotorHardware::i2c_BufferRead(const char *i2cDevFile, uint8_t i2c8bitAddr,
                          uint8_t *pBuffer, int16_t chipRegAddr, uint16_t NumBytesToRead)
{
   int bytesRead = 0;
   int retCode   = 0;

    int fd;                                         // File descrition
    int  address   = i2c8bitAddr >> 1;              // Address of the I2C device
    uint8_t buf[8];                                 // Buffer for data being written to the i2c device

    if ((fd = open(i2cDevFile, O_RDWR)) < 0) {      // Open port for reading and writing
      retCode = -2;
      RCLCPP_ERROR(logger, "Cannot open I2C def of %s with error %s", i2cDevFile, strerror(errno));
      goto exitWithNoClose;
    }

    // The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
    if (ioctl(fd, I2C_SLAVE, address) != 0) {        // Set the port options and addr of the dev
      retCode = -3;
      RCLCPP_ERROR(logger, "Failed to get bus access to I2C device %s!  ERROR: %s", i2cDevFile, strerror(errno));
      goto exitWithFileClose;
    }

    if (chipRegAddr < 0) {     // Suppress reg address if negative value was used
      buf[0] = (uint8_t)(chipRegAddr);          // Internal chip register address
      if ((::write(fd, buf, 1)) != 1) {           // Write both bytes to the i2c port
        retCode = -4;
        goto exitWithFileClose;
      }
    }

    bytesRead = ::read(fd, pBuffer, NumBytesToRead);
    if (bytesRead != NumBytesToRead) {      // verify the number of bytes we requested were read
      retCode = -9;
      goto exitWithFileClose;
    }
    retCode = bytesRead;

  exitWithFileClose:
    close(fd);

  exitWithNoClose:

  return retCode;
}

void MotorHardware::initMcbParameters()
{
    // A full mcb initialization requires high level system overrides to be disabled
    node_params->mcbControlEnabled = 1;
    node_params->mcbSpeedEnabled   = 1;

    // Force future calls to sendParams() to update current pid parametes on the MCB
    forcePidParamUpdates();

    // Determine the wheel type to be used by the robot base 
    int32_t wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
    if (node_params->wheel_type == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        RCLCPP_INFO(logger, "Default wheel_type of 'standard' will be used.");
        wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
    } else {
        // Any other setting leads to host setting the wheel type
        if (node_params->wheel_type == "standard") {
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
            RCLCPP_INFO(logger, "Host is specifying wheel_type of '%s'", "standard");
        } else if (node_params->wheel_type == "thin"){
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_THIN;
            RCLCPP_INFO(logger, "Host is specifying wheel_type of '%s'", "thin");

            // If thin wheels and no drive_type is in yaml file we will use 4wd
            if (node_params->drive_type == "firmware_default") {
                RCLCPP_INFO(logger, "Default to drive_type of 4wd when THIN wheels unless option drive_type is set");
                node_params->drive_type = "4wd";
            }
        } else {
            RCLCPP_WARN(logger, "Invalid wheel_type of '%s' specified! Using wheel type of standard", 
                node_params->wheel_type.c_str());
            node_params->wheel_type = "standard";
            wheel_type = MotorMessage::OPT_WHEEL_TYPE_STANDARD;
        }
    }
    // Write out the wheel type setting to hardware layer
    setWheelType(wheel_type);
    wheel_type = wheel_type;
    rclcpp::sleep_for(mcbStatusSleepPeriodNs);


    // Determine the wheel gear ratio to be used by the robot base 
    // Firmware does not use this setting so no message to firmware is required
    // This gear ratio is contained in the hardware layer so if this node got new setting update hardware layer
    setWheelGearRatio(node_params->wheel_gear_ratio);
    RCLCPP_INFO(logger, "Wheel gear ratio of %5.3f will be used.", node_params->wheel_gear_ratio);

    // Determine the drive type to be used by the robot base
    int32_t drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
    if (node_params->drive_type == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        RCLCPP_INFO(logger, "Default drive_type of 'standard' will be used.");
        drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
    } else {
        // Any other setting leads to host setting the drive type
        if (node_params->drive_type == "standard") {
            drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
            RCLCPP_INFO(logger, "Host is specifying drive_type of '%s'", "standard");
        } else if (node_params->drive_type == "4wd"){
            drive_type = MotorMessage::OPT_DRIVE_TYPE_4WD;
            RCLCPP_INFO(logger, "Host is specifying drive_type of '%s'", "4wd");
        } else {
            RCLCPP_WARN(logger, "Invalid drive_type of '%s' specified! Using drive type of standard",
                node_params->drive_type.c_str());
            node_params->drive_type = "standard";
            drive_type = MotorMessage::OPT_DRIVE_TYPE_STANDARD;
        }
    }
    // Write out the drive type setting to hardware layer
    setDriveType(drive_type);
    drive_type = drive_type;
    rclcpp::sleep_for(mcbStatusSleepPeriodNs);

    int32_t wheel_direction = 0;
    if (node_params->wheel_direction == "firmware_default") {
        // Here there is no specification so the firmware default will be used
        RCLCPP_INFO(logger, "Firmware default wheel_direction will be used.");
    } else {
        // Any other setting leads to host setting the wheel type
        if (node_params->wheel_direction == "standard") {
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
            RCLCPP_INFO(logger, "Host is specifying wheel_direction of '%s'", "standard");
        } else if (node_params->wheel_direction == "reverse"){
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_REVERSE;
            RCLCPP_INFO(logger, "Host is specifying wheel_direction of '%s'", "reverse");
        } else {
            RCLCPP_WARN(logger, "Invalid wheel_direction of '%s' specified! Using wheel direction of standard", 
                node_params->wheel_direction.c_str());
            node_params->wheel_direction = "standard";
            wheel_direction = MotorMessage::OPT_WHEEL_DIR_STANDARD;
        }
        // Write out the wheel direction setting
        setWheelDirection(wheel_direction);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Tell the controller board firmware what version the hardware is at this time.
    // TODO: Read from I2C.   At this time we only allow setting the version from ros parameters
    if (firmware_version >= MIN_FW_HW_VERSION_SET) {
        RCLCPP_INFO_ONCE(logger, "Firmware is version %d. Setting Controller board version to %d", 
            firmware_version, fw_params->controller_board_version);
        setHardwareVersion(fw_params->controller_board_version);
        RCLCPP_DEBUG(logger, "Controller board version has been set to %d", 
            fw_params->controller_board_version);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Suggest to customer to have current firmware version
    if (firmware_version < MIN_FW_SUGGESTED) {
        RCLCPP_ERROR_ONCE(logger, "Firmware is version V%d. We strongly recommend minimum firmware version of at least V%d", 
            firmware_version, MIN_FW_SUGGESTED);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    } else {
        RCLCPP_INFO_ONCE(logger, "Firmware is version V%d. This meets the recommend minimum firmware versionof V%d", 
            firmware_version, MIN_FW_SUGGESTED);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Certain 4WD robots rely on wheels to skid to reach final positions.
    // For such robots when loaded down the wheels can get in a state where they cannot skid.
    // This leads to motor overheating.  This code below sacrifices accurate odometry which
    // is not achievable in such robots anyway to relieve high wattage drive power when zero velocity.
    wheel_slip_nulling = 0;
    if ((firmware_version >= MIN_FW_WHEEL_NULL_ERROR) && (node_params->drive_type == "4wd")) {
        wheel_slip_nulling = 1;
        RCLCPP_INFO(logger, "Wheel slip nulling will be enabled for this 4wd system when velocity remains at zero.");
    }

    // Tell the MCB board what the port that is on the Pi I2c says on it (the mcb cannot read it's own switchs!)
    // We could re-read periodically but perhaps only every 5-10 sec but should do it from main loop
    if (firmware_version >= MIN_FW_OPTION_SWITCH && hardware_version >= MIN_HW_OPTION_SWITCH) {
        fw_params->option_switch = getOptionSwitch();
        RCLCPP_INFO(logger, "Setting firmware option register to 0x%x.", fw_params->option_switch);
        setOptionSwitchReg(fw_params->option_switch);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }
    
    if (firmware_version >= MIN_FW_SYSTEM_EVENTS) {
        // Start out with zero for system events
        setSystemEvents(0);  // Clear entire system events register
        system_events = 0;
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    // Setup other firmware parameters that could come from ROS parameters
    if (firmware_version >= MIN_FW_ESTOP_SUPPORT) {
        setEstopPidThreshold(fw_params->estop_pid_threshold);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
        setEstopDetection(fw_params->estop_detection);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }

    if (firmware_version >= MIN_FW_MAX_SPEED_AND_PWM) {
        setMaxFwdSpeed(fw_params->max_speed_fwd);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
        setMaxRevSpeed(fw_params->max_speed_rev);
        rclcpp::sleep_for(mcbStatusSleepPeriodNs);
    }
        
    return;
}



// Diagnostics Status Updater Functions
using diagnostic_updater::DiagnosticStatusWrapper;
using diagnostic_msgs::msg::DiagnosticStatus;

void MotorDiagnostics::firmware_status(DiagnosticStatusWrapper &stat) {
    stat.add("Firmware Version", firmware_version);
    if (firmware_version == 0) {
        stat.summary(DiagnosticStatus::ERROR, "No firmware version reported. Power may be off.");
    }
    else if (firmware_version < MIN_FW_RECOMMENDED) {
        stat.summary(DiagnosticStatus::WARN, "Firmware is older than recommended! You must update firmware!");
    }
    else {
        stat.summary(DiagnosticStatus::OK, "Firmware version is OK");
    }
}

void MotorDiagnostics::firmware_date_status(DiagnosticStatusWrapper &stat) {

    // Only output status if the firmware daycode is supported
    if (firmware_version >= MIN_FW_FIRMWARE_DATE) {
        std::stringstream stream;
        stream << std::hex << firmware_date;
        std::string daycode(stream.str());

        stat.add("Firmware Date", daycode);
        stat.summary(DiagnosticStatus::OK, "Firmware daycode format is YYYYMMDD");
    }
}

// When a firmware limit condition is reported the diagnostic topic reports it.
// Once the report is made the condition is cleared till next time firmware reports that limit
void MotorDiagnostics::limit_status(DiagnosticStatusWrapper &stat) {
    stat.summary(DiagnosticStatus::OK, "Limits reached:");
    if (left_pwm_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::ERROR, " left pwm,");
        left_pwm_limit = false;
    }
    if (right_pwm_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::ERROR, " right pwm,");
        right_pwm_limit = false;
    }
    if (left_integral_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " left integral,");
        left_integral_limit = false;
    }
    if (right_integral_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " right integral,");
        right_integral_limit = false;
    }
    if (left_max_speed_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " left speed,");
        left_max_speed_limit = false;
    }
    if (right_max_speed_limit) {
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " right speed,");
        right_max_speed_limit = false;
    }
    if (param_limit_in_firmware) {
        // A parameter was sent to firmware that was out of limits for the firmware register
        stat.mergeSummary(DiagnosticStatusWrapper::WARN, " firmware limit,");
        param_limit_in_firmware = false;
    }
}

void MotorDiagnostics::battery_status(DiagnosticStatusWrapper &stat) {
    stat.add("Battery Voltage", battery_voltage);
    if (battery_voltage < battery_voltage_low_level) {
        stat.summary(DiagnosticStatusWrapper::WARN, "Battery low");
    }
    else if (battery_voltage < battery_voltage_critical) {
        stat.summary(DiagnosticStatusWrapper::ERROR, "Battery critical");
    }
    else {
        stat.summary(DiagnosticStatusWrapper::OK, "Battery OK");
    }
}

// PID parameters for motor control
void MotorDiagnostics::motor_pid_p_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam P", fw_pid_proportional);
    stat.summary(DiagnosticStatus::OK, "PID Parameter P");
}
void MotorDiagnostics::motor_pid_i_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam I", fw_pid_integral);
    stat.summary(DiagnosticStatus::OK, "PID Parameter I");
}
void MotorDiagnostics::motor_pid_d_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam D", fw_pid_derivative);
    stat.summary(DiagnosticStatus::OK, "PID Parameter D");
}
void MotorDiagnostics::motor_pid_v_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam V", fw_pid_velocity);
    stat.summary(DiagnosticStatus::OK, "PID Parameter V");
}
void MotorDiagnostics::motor_max_pwm_status(DiagnosticStatusWrapper &stat) {
    stat.add("PidParam MaxPWM", fw_max_pwm);
    stat.summary(DiagnosticStatus::OK, "PID Max PWM");
}


void MotorDiagnostics::motor_power_status(DiagnosticStatusWrapper &stat) {
    stat.add("Motor Power", !estop_motor_power_off);
    if (estop_motor_power_off == false) {
        stat.summary(DiagnosticStatusWrapper::OK, "Motor power on");
    }
    else {
        stat.summary(DiagnosticStatusWrapper::WARN, "Motor power off");
    }
}


// Show firmware options and give readable decoding of the meaning of the bits
void MotorDiagnostics::firmware_options_status(DiagnosticStatusWrapper &stat) {
    stat.add("Firmware Options", firmware_options);
    std::string option_descriptions("");
    if (firmware_options & MotorMessage::OPT_ENC_6_STATE) {
        option_descriptions += "High resolution encoders";
    } else {
        option_descriptions += "Standard resolution encoders";
    }
    if (firmware_options & MotorMessage::OPT_WHEEL_TYPE_THIN) {
        option_descriptions +=  ", Thin gearless wheels";
    } else {
        option_descriptions +=  ", Standard wheels";
    }
    if (firmware_options & MotorMessage::OPT_DRIVE_TYPE_4WD) {
        option_descriptions +=  ", 4 wheel drive";
    } else {
        option_descriptions +=  ", 2 wheel drive";
    }
    if (firmware_options & MotorMessage::OPT_WHEEL_DIR_REVERSE) {
        // Only indicate wheel reversal if that has been set as it is non-standard
        option_descriptions +=  ", Reverse polarity wheels";
    }
    stat.summary(DiagnosticStatusWrapper::OK, option_descriptions);
}


PLUGINLIB_EXPORT_CLASS(MotorHardware, hardware_interface::SystemInterface)
