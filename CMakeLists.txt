cmake_minimum_required(VERSION 3.8)
project(ubiquity_motor_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#find_package(message_generation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(diagnostic_updater REQUIRED)
#find_package(dynamic_reconfigure REQUIRED) - does not exist in ROS2

#find_package(Boost REQUIRED)
#find_package(thread REQUIRED)
#find_package(atomic REQUIRED)

add_library(ubiquity_motor
  src/motor_message.cc src/motor_serial.cc src/motor_hardware.cc
)

install(PROGRAMS
  scripts/upgrade_firmware.py
  scripts/test_motor_board.py
  scripts/test_pi_gpio.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
