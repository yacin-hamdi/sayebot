from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os

def generate_launch_description():

    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="racetrack_day_empty"
    )
    world_name = LaunchConfiguration("world_name")

    world_path = PathJoinSubstitution([
        FindPackageShare("sayebot_description"),
        "worlds", 
        PythonExpression(["'", world_name, "'", " + '.world'"])
    ])

    
    

    resource_path = str(Path(get_package_share_directory("sayebot_description")).parent.resolve())
    resource_path += os.pathsep + os.path.join(get_package_share_directory("sayebot_description"), "models")
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH", 
        value=resource_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments=[("gz_args", PythonExpression(["' -v 4 -r ", world_path, "'"]))]
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim", 
        executable="create", 
        output="screen", 
        arguments=["-topic", "robot_description", 
                    "-name", "sayebot",
                    "-x", "-2.0",
                    "-y", "-15.0",
                    "-z", "0.5"
                   
                   
                   ]
    )

    

    gz_ros2_bridge = Node(
        package="ros_gz_bridge", 
        executable="parameter_bridge", 
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ]
    )
    return LaunchDescription([
        world_name_arg,
        gazebo_resource_path,
        gazebo, 
        gz_spawn_entity, 
        gz_ros2_bridge
    ])