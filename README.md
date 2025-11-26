# Ubiquity motor ROS 2

This node translates cmd_vel message to the data used by the motor firmware.

In the launch file, also magni_description is launched.

**TODO:** Launch magni_description in ezmap_bringup instead of here.

Gen6 only uses this node for launching magni_description, because firmware handles cmd_vel directly.
Gen5 uses both the motor node and magni_description.


## Build the motor node
Run this in the ROS workspace:

`colcon build --packages-up-to ubiquity_motor_ros2`

`source install/setup.bash`

## Run motor node

`ros2 launch ubiquity_motor_ros2 ubiquity_motor_ros2.launch.py`

## Drive robot with teleop using this node

In another terminal run:

`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true`