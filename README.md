# Ubiquity motor ROS 2

## Build the motor node and its msgs
Run this in the ROS workspace:

`colcon build --base-paths src/ubiquity_motor_ros2 src/ubiquity_motor_ros2/ubiquity_motor_ros2_msgs --cmake-args --event-handlers console_direct+`

`source install/setup.bash`

## Run motor node

`ros2 launch ubiquity_motor_ros2 motors.launch.py`

## Teleop 

`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true`