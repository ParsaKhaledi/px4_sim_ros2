from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    rsp_node =  Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='oak_state_publisher',
            parameters=[{'robot_description': Command(['cat x500_urdf.urdf']), 'use_sim_time': True}]
        )
    ld = LaunchDescription()
    ld.add_action(rsp_node)
    return ld
