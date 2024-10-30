import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    # Get path to URDF file from the arm_description package
    urdf_file = os.path.join(get_package_share_directory('arm_description'), 'urdf', 'arm.urdf.xacro')

    # Argument declaration for using simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Node for loading robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])},
                    {"use_sim_time": use_sim_time}]
    )

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            'gz_args', 
            default_value='-r -v 1 empty.sdf',
            description='Arguments for gz_sim'
        )
    ] 
    # Node for starting Gazebo Ignition (gz sim)
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([get_package_share_directory('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )

    # Node for spawning the robot in Gazebo Ignition
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-entity', 'robot'
        ]
    )

    # Node for bridging the camera topics from Gazebo Ignition to ROS2
    bridge_camera_node = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ],
        output='screen'
    )


    # List of nodes to start
    nodes_to_start = [
        robot_state_publisher_node,
        gazebo_ignition,
        spawn_robot_node,
        bridge_camera_node
    ]
 
    return LaunchDescription(declared_arguments + nodes_to_start)