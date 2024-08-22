# Ubiquity motor ROS 2

## Build the motor node
colcon build --packages-select ubiquity_motor_ros2 --cmake-args -DCMAKE_PREFIX_PATH=/home/ubuntu/ros2_ws/install/zenohc --symlink-install --event-handlers console_direct+

## Build the examples
colcon build --packages-select zenohcxx --cmake-args -DCMAKE_PREFIX_PATH=/home/ubuntu/ros2_ws/install/zenohc --cmake-target examples --symlink-install --event-handlers console_direct+