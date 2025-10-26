from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="true"
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    joy_teleop = Node(
        package="joy_teleop", 
        executable="joy_teleop",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("sayebot_teleop"),
                "config",
                "joy_teleop.yaml"
            ]),
            {"use_sim_time": use_sim_time}
        ]
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("sayebot_teleop"),
                "config",
                "joy_config.yaml"
            ]),
            {"use_sim_time": use_sim_time}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        joy_teleop,
        joy_node
    ])