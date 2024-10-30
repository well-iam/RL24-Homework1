import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Spawner node for joint_state_broadcaster
    joint_state_broadcaster_spawner = Node( 
        package="controller_manager", 
        executable="spawner", 
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"], 
        output="screen"
    )
    
    # Spawner node for position_controller
    position_controller_spawner = Node( 
        package="controller_manager", 
        executable="spawner", 
        arguments=["position_controller", "--controller-manager", "/controller_manager"], 
        output="screen"
    )

    # Spawner node for arm_controller_node
    arm_controller_node = Node( 
        package="arm_control", 
        executable="arm_controller_node", 
        name="arm_controller_node", 
        output="screen"
    )


    return LaunchDescription([
        joint_state_broadcaster_spawner,
        position_controller_spawner,
        arm_controller_node
    ])
