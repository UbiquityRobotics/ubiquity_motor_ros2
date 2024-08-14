cmake_minimum_required(VERSION 3.5)
project(ubiquity_motor_ros2)

set(CMAKE_CXX_STANDARD 14) # use C++14

# Compiler flags for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control REQUIRED)
find_package(diagnostic_updater REQUIRED)

# ROS 2 message generation
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
)

# Add library
add_library(ubiquity_motor
  #src/motor_message.cc src/motor_serial.cc src/motor_hardware.cc
  src/motor_node.cc
)


# Include directories for library
include_directories(ubiquity_motor 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# target_include_directories(serial
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial>
#     # $<INSTALL_INTERFACE:include>
# )

ament_target_dependencies(ubiquity_motor
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "hardware_interface"
  "ros2_control"
  "diagnostic_updater"
)

# Add executable
add_executable(motor_node src/motor_node.cc)

# Include directories for executable
# target_include_directories(motor_node
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#     $<INSTALL_INTERFACE:include>
# )

ament_target_dependencies(motor_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "hardware_interface"
  "ros2_control"
  "diagnostic_updater"
)

# Install targets
install(TARGETS
  motor_node
  ubiquity_motor
  DESTINATION lib/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/upgrade_firmware.py
  scripts/test_motor_board.py
  scripts/test_pi_gpio.py
  DESTINATION lib/${PROJECT_NAME}
)

# Test dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()