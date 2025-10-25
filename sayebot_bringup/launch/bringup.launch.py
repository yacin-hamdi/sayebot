from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
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

    controller = TimerAction(
        period=10.0,

        actions=[IncludeLaunchDescription(
                    PathJoinSubstitution([
                        FindPackageShare("sayebot_controller"),
                        "launch", 
                        "controller.launch.py"
                    ])
                )]

    )

    

    joystick = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("sayebot_teleop"),
            "launch",
            "joystick.launch.py"
        ])
    )

    

    return LaunchDescription([
        gazebo, 
        # rviz,
        joystick, 
        controller
    ])