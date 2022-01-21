# ROS2 packages for controlling Robotiq 2F85 gripper with the ros2_control framework

test forward controller:
`ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.1]"`