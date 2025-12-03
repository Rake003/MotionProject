from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []
    
    # Flexiv specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="rizon4",
            description="Type of Flexiv robot (rizon4, rizon10, etc.).",
            choices=["rizon4", "rizon10"],
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("flexiv_kinematics_ros"), "urdf", "flexiv_rizon4s_kinematics.urdf"]
            ),
            description="URDF file (absolute path) with the robot description.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("flexiv_kinematics_ros"), "rviz", "view_robot.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        )
    )

    # Initialize Arguments
    robot_type = LaunchConfiguration("robot_type")
    description_file = LaunchConfiguration("description_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Robot description - load directly from URDF file
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="cat")]),
            " ",
            description_file,
        ]
    )
    
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            robot_description,
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Interactive Kinematics Node (C++)
    interactive_kinematics_node = Node(
        package="flexiv_kinematics_ros",
        executable="interactive_kinematics_node",
        name="flexiv_interactive_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        prefix="xterm -e",  # Opens in separate terminal for user input
    )

    nodes_to_start = [
        robot_state_publisher_node,
        rviz_node,
        interactive_kinematics_node,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)