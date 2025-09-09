from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    robot_description_arg = DeclareLaunchArgument(
        name="robot_description", 
        default_value=PathJoinSubstitution([
                FindPackageShare("sayebot_description"), 
                "urdf", 
                "sayebot.urdf.xacro"
            ])
    )

    use_joint_publisher_arg = DeclareLaunchArgument(
        name="use_joint", 
        default_value="true"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("robot_description")]), value_type=str)
    use_joint_publisher = LaunchConfiguration("use_joint")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher", 
        parameters=[
            {"robot_description": robot_description}
        ]

    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui", 
        executable="joint_state_publisher_gui",
        condition=IfCondition(use_joint_publisher)
    )


    rviz2 = Node(
        package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        output="screen", 
        arguments=["-d", os.path.join(get_package_share_directory("sayebot_description"), "rviz", "display.rviz")]
    )

    return LaunchDescription([
        use_joint_publisher_arg,
        robot_description_arg,
        robot_state_publisher,
        joint_state_publisher_gui, 
        rviz2
    ])