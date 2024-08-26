# Ubiquity motor ROS 2

## Build the motor node and its msgs
colcon build --base-paths src/ubiquity_motor_ros2 src/ubiquity_motor_ros2/ubiquity_motor_ros2_msgs --cmake-args --event-handlers console_direct+