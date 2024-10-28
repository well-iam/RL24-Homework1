import os
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
 
# Importa get_package_share_directory
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():
    # Ottieni il percorso del file URDF dal pacchetto arm_description
    urdf_file = os.path.join(get_package_share_directory('arm_description'), 'urdf', 'arm.urdf')
 
    # Stampa il percorso del file URDF per debug
    print(f"Loading URDF from: {urdf_file}")

    # Dichiarazione dell'argomento per utilizzare il tempo di simulazione
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
 
    # Carica il robot URDF come parametro "robot_description" in ROS 2
    #robot_description = Command(['cat', urdf_file])
    
    # Nodo per caricare la descrizione del robot
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read(), 'use_sim_time': use_sim_time}] #prima era robot_description
    )

    # Nodo per avviare Gazebo Ignition (gz sim)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution(
                [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])])#,
        #launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()
    )
 
    # Nodo per spawnare il robot in Gazebo Ignition
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
 
    # Lista di nodi da avviare
    nodes_to_start = [
        gazebo,
        robot_state_publisher_node,
        spawn_robot_node
    ]
 
    return LaunchDescription(nodes_to_start)