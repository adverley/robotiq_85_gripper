# robotiq_85_gripper
ROS2 packages for controlling the Robotiq 2F85 Gripper using RS485. 
Based on the [PickNik Drivers](https://github.com/PickNikRobotics/robotiq_85_gripper).

These packages do not use the [ros_control](http://control.ros.org/) stack, as it comes with 2 downsides:
- one has to write the HW interface in C++ (and the one I found from Picknick was written in python)
- The controller manager only takes in a single config file, so one has to duplicate launch files from the robot and gripper in order to bring up the combined system.

Nonetheless, the controllers that are created implement the interface of the `GripperController`: They expose a GripperCommand action. Because of this, the downstream users (Moveit e.g.) are completely agnostic of this.

## Packages
- `robotiq_85_description` : defines URDF description and contains a launch file to visualize the gripper in RVIZ (with a dummy link to attach the gripper to the world). This launch file also offers the possibility to use the `joint_publisher_gui` to publish the joint state of the gripper. `ros2 launch robotiq_85_description view_gripper.launch.py launch_joint_publisher_gui:=false`
- `robotiq_85_driver`: contains the PickNick driver and controller for the Robotiq 85. Additionally, this package contains a "fake_controller" to simulate a controller and driver for the gripper. This allows for testing other software packages etc.
- `robotiq_bringup`: contains a launch file that brings up either the driver and controller or the fake controller, depending on the `use_fake_controller` argument. Launch with: `ros2 launch robotiq_85_bringup robotiq_85_control.launch.py use_fake_controller:=true`

## Usage
Both the fake controller and real controller expose an action server at `/robotiq_gripper_controller/gripper_cmd`. This is the expected interface for Moveit grippers.

For example, to completely close the gripper send the following goal to the action server using the CLI:
`ros2 action send_goal /robotiq_gripper_controller/gripper_cmd control_msgs/action/GripperCommand "command: {position: 0.8}"`

