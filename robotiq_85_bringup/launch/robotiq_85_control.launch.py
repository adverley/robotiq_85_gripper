from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_controller", default_value="false", description="Use fake controller to simulate gripper?"
        )
    )
    use_fake_controller = LaunchConfiguration("use_fake_controller")

    gripper_driver_node = Node(
        package="robotiq_85_driver",
        executable="robotiq_85_driver",
        name="robotiq_85_driver",
        condition=UnlessCondition(use_fake_controller),
        parameters=[{"num_grippers": 1}, {"comport": "/dev/ttyUSB0"}, {"baud": "115200"}],
        output="screen",
    )

    # TODO: start up the Controller

    fake_gripper_controller_node = Node(
        package="robotiq_85_driver",
        executable="robotiq_85_fake_action_server",
        name="robotiq_85_driver",
        condition=IfCondition(use_fake_controller),
        output="screen",
    )

    return LaunchDescription(declared_arguments + [fake_gripper_controller_node, gripper_driver_node])
