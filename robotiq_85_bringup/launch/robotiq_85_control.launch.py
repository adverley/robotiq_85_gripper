from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_controller", default_value="false", description="Use fake controller to simulate gripper?"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint_names, if changed also change the controller configuration.",
        )
    )

    prefix = LaunchConfiguration("prefix")
    use_fake_controller = LaunchConfiguration("use_fake_controller")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robotiq_85_description"), "urdf", "robotiq_85_gripper.urdf.xacro"]
            ),
            " ",
            "prefix:=",
            prefix,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    gripper_driver_node = Node(
        package="robotiq_85_driver",
        executable="robotiq_85_driver",
        name="robotiq_85_driver",
        condition=UnlessCondition(use_fake_controller),
        parameters=[{"num_grippers": 1}, {"comport": "/dev/ttyUSB0"}, {"baud": "115200"}],
        output="screen",
    )

    gripper_controller_node = Node(
        package="robotiq_85_driver",
        executable="single_robotiq_85_action_server",
        name="robotiq_85_controller",
        condition=UnlessCondition(use_fake_controller),
        output="screen",
    )
    fake_gripper_controller_node = Node(
        package="robotiq_85_driver",
        executable="robotiq_85_fake_action_server",
        name="robotiq_85_driver",
        condition=IfCondition(use_fake_controller),
        output="screen",
    )

    nodes_to_start = [robot_state_pub_node, fake_gripper_controller_node, gripper_controller_node, gripper_driver_node]

    return LaunchDescription(declared_arguments + nodes_to_start)
