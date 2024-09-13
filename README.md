# Ubiquity motor ROS 2

## Build 

Clone and build this repo: https://github.com/UbiquityRobotics/ROS2_humble

Run this in the ROS workspace:

`colcon build --base-paths src/ubiquity_motor_ros2 src/ubiquity_motor_ros2/ubiquity_motor_ros2_msgs --cmake-args --event-handlers console_direct+`

`source install/setup.bash`

## Run

`ros2 launch ubiquity_motor_ros2 motors.launch.py`

## Teleop 

`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true`
