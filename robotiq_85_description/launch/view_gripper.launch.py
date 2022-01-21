import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix of the joint_names, if changed also change the controller configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_joint_publisher_gui",
            default_value="true",
            description="Use Joint publisher GUI to manipulate joint pose.",
        )
    )

    prefix = LaunchConfiguration("prefix")
    launch_joint_publisher_gui = LaunchConfiguration("launch_joint_publisher_gui")
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
    # joint_state_pub_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher'
    # )
    print(robot_description)
    joint_state_pub_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(launch_joint_publisher_gui),
        parameters=[robot_description],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            os.path.join(get_package_share_directory("robotiq_85_description"), "config", "view_gripper.rviz"),
        ],
        parameters=[robot_description],
    )

    return LaunchDescription(declared_arguments + [rviz_node, robot_state_pub_node, joint_state_pub_gui_node])
