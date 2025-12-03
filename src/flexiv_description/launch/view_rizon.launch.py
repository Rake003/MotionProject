from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("flexiv_description")

    # Default values so launching without arguments works
    robot_sn = LaunchConfiguration("robot_sn")
    rizon_type = LaunchConfiguration("rizon_type")
    load_gripper = LaunchConfiguration("load_gripper")
    gripper_name = LaunchConfiguration("gripper_name")
    load_mounted_ft_sensor = LaunchConfiguration("load_mounted_ft_sensor")
    gui = LaunchConfiguration("gui")
    rvizconfig = LaunchConfiguration("rvizconfig")

    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share, "rviz", "view_rizon.rviz"]
    )

    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                '"',
                PathJoinSubstitution(
                    [FindPackageShare("flexiv_description"), "urdf", "rizon.urdf.xacro"]
                ),
                '"',
                " ",
                "robot_sn:=", robot_sn, " ",
                "rizon_type:=", rizon_type, " ",
                "load_gripper:=", load_gripper, " ",
                "gripper_name:=", gripper_name, " ",
                "load_mounted_ft_sensor:=", load_mounted_ft_sensor,
            ]
        ),
        value_type=str,
    )

    # Nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(gui),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rvizconfig],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_sn",
                default_value="Rizon4s-000000",
                description="Serial number (default placeholder)",
            ),
            DeclareLaunchArgument(
                "rizon_type",
                default_value="Rizon4s",
                description="Rizon robot type",
                choices=["Rizon4", "Rizon4M", "Rizon4R", "Rizon4s", "Rizon10", "Rizon10s"],
            ),
            DeclareLaunchArgument(
                "load_gripper",
                default_value="False",
                description="Load Flexiv Grav gripper",
            ),
            DeclareLaunchArgument(
                "gripper_name",
                default_value="Flexiv-GN01",
                description="Name of gripper",
            ),
            DeclareLaunchArgument(
                "load_mounted_ft_sensor",
                default_value="False",
                description="Load mounted force-torque sensor",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="False",
                description="Enable joint_state_publisher_gui",
            ),
            DeclareLaunchArgument(
                "rvizconfig",
                default_value=default_rviz_config_path,
                description="RViz config path",
            ),
            robot_state_publisher_node,
            joint_state_publisher_node,
            joint_state_publisher_gui_node,
            rviz_node,
        ]
    )
