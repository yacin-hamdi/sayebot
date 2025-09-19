from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    gazebo = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("sayebot_description"), 
            "launch",
            "gazebo.launch.py"
        ])
    )

    rviz = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("sayebot_description"), 
            "launch", 
            "display.launch.py"
        ]), 
        launch_arguments={
            "use_joint": "false"
        }.items()
        
    )

    controller = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("sayebot_controller"),
            "launch", 
            "controller.launch.py"
        ])
    )

    return LaunchDescription([
        gazebo, 
        rviz,
        controller
    ])