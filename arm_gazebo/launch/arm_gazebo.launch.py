# arm_gazebo.launch.py

from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription

def generate_launch_description():
    # Declare any launch arguments if needed
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', 
                              description='Use simulation (Gazebo) clock if true.'),
        
        # Include the arm_world.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('arm_gazebo'), 'launch', 'arm_world.launch.py'])
            ),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Include the arm_control.launch.py to spawn controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare('arm_control'), 'launch', 'arm_control.launch.py'])
            ),
            condition=IfCondition(LaunchConfiguration('use_sim_time'))
        ),
        
        # Optionally log the launch
        LogInfo(
            condition=IfCondition(LaunchConfiguration('use_sim_time')),
            msg="Successfully launched Gazebo and Controllers."
        )
    ])