import os
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import Command,FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit

def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
    DeclareLaunchArgument(
    "prefix",
    default_value="",
    description="Prefix of the joint_names, if changed also change the controller configuration."
    )
    )

    prefix =  LaunchConfiguration("prefix")
    robot_description_content = Command(
    [
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("robotiq_85_description"), "urdf", "robotiq_85_gripper.urdf.xacro"]),
        " ",
        "prefix:=",
        prefix
    ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("robotiq_85_bringup"),
            "config",
            "robotiq_85_controllers.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters = [robot_description, robot_controllers]
    )

    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='log',
    arguments=['-d', os.path.join(get_package_share_directory("robotiq_85_description"), "config", "view_gripper.rviz")],
    parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["robotiq_85_joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )


    foward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["robotiq_85_forward_position_controller", "-c", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[foward_controller_spawner],
        )
    )
    nodes_to_start  = [
    control_node,
    robot_state_pub_node, 
    joint_state_broadcaster_spawner,
    delay_rviz_after_joint_state_broadcaster_spawner,
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]
    return LaunchDescription(declared_arguments  + nodes_to_start)

