cmake_minimum_required(VERSION 3.5)
project(ubiquity_motor_ros2)

set(CMAKE_CXX_STANDARD 14) # use C++14

# Compiler flags for warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
# find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(ros2_control REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ubiquity_motor_ros2_msgs REQUIRED)
# find_package(Boost REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# find_package(zenohc REQUIRED)

# add_compile_definitions(ZENOHCXX_ZENOHC) # Tells zenoh-cpp to use zenoh-c
# zenoh-c and zenoh-cpp should be in parent folder
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../zenoh-c ${CMAKE_BINARY_DIR}/zenohc)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../zenoh-cpp ${CMAKE_BINARY_DIR}/zenohcxx)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../rmw_zenoh/zenoh_c_vendor ${CMAKE_BINARY_DIR}/zenohcvendor)
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../rmw_zenoh/rmw_zenoh_cpp ${CMAKE_BINARY_DIR}/zenohrmw)
# add_subdirectory(../zenoh-cpp ${CMAKE_BINARY_DIR}/zenohcxx)

# # ROS 2 message generation
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/MotorState.msg"
#   DEPENDENCIES std_msgs # Add this line
# )

# Ensure the generated code is built before anything else that uses it
# ament_export_dependencies(rosidl_default_runtime)


set(serial_SRCS
    serial/src/serial.cc
    serial/include/serial/serial.h
    serial/include/serial/v8stdint.h
)
# If unix
list(APPEND serial_SRCS serial/src/impl/unix.cc)
list(APPEND serial_SRCS serial/src/impl/list_ports/list_ports_linux.cc)
add_library(ur_serial SHARED ${serial_SRCS})
# target_link_libraries(ur_serial /usr/local/lib/libpython3.8.so)

# Link libraries (add any additional libraries if needed)
target_link_libraries(ur_serial rt pthread)
##H HACK to contain all serial stuff in here
# list(APPEND catkin_LIBRARIES ur_serial)
# list(APPEND catkin_INCLUDE_DIRS serial/include)

# Include directories
target_include_directories(ur_serial
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial/include>
    $<INSTALL_INTERFACE:include>
)

# ## Mark executables and/or libraries for installation
# install(TARGETS ur_serial
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# Mark executables and/or libraries for installation
install(TARGETS ur_serial
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY serial/include/
  DESTINATION include
)

# Export package information
ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/serial/include)
ament_export_libraries(ur_serial)



# add_executable(zc_pub_turtle src/z_pub_turtle.cc)
# target_link_libraries(zc_pub_turtle PRIVATE zenohcxx::zenohc)
# set_property(TARGET zc_pub_turtle PROPERTY LANGUAGE CXX)
# set_property(TARGET zc_pub_turtle PROPERTY CXX_STANDARD 17)


# Add library
add_library(${PROJECT_NAME}_lib
  src/motor_message.cc 
  src/motor_serial.cc 
  src/motor_hardware.cc
)

# target_link_libraries(${PROJECT_NAME}_lib PUBLIC zenohcxx::zenohc::lib)

# Include directories for library
target_include_directories(${PROJECT_NAME}_lib 
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/serial/include  # Add this line to include the serial headers
    # ${CMAKE_CURRENT_SOURCE_DIR}/zenoh-c/include
    # ${CMAKE_CURRENT_SOURCE_DIR}/zenoh-cpp/include
)

# target_include_directories(serial
#   PUBLIC
#     $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/serial>
#     # $<INSTALL_INTERFACE:include>
# )


ament_target_dependencies(${PROJECT_NAME}_lib
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "diagnostic_msgs"
  "hardware_interface"
  "ros2_control"
  "diagnostic_updater"
  # "rosidl_default_runtime"
  "ubiquity_motor_ros2_msgs"
  # "${PROJECT_NAME}"  # This adds a dependency on the generated messages in the same package
)

# Add executable
add_executable(motor_node 
  src/motor_message.cc 
  src/motor_serial.cc 
  src/motor_hardware.cc
  src/motor_node.cc)

# Include directories for executable
target_include_directories(motor_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/serial/include
    ${controller_manager_INCLUDE_DIRS}
)

ament_target_dependencies(motor_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "hardware_interface"
  "ros2_control"
  "diagnostic_updater"
  "controller_manager"
  "ubiquity_motor_ros2_msgs"
)

target_link_libraries(motor_node Boost::thread ur_serial)

# Install targets
install(TARGETS
  motor_node
  ${PROJECT_NAME}_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/upgrade_firmware.py
  scripts/test_motor_board.py
  scripts/test_pi_gpio.py
  DESTINATION lib/${PROJECT_NAME}
)

# # Test dependencies
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()