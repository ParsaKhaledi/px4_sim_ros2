from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from ament_index_python.packages import get_package_share_directory
import os

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
