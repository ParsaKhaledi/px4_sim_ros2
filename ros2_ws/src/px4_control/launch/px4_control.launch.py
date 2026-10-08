"""Start the PX4 control node.

``use_sim_time`` defaults to true because the Docker stack bridges ``/clock``.
Pass ``use_sim_time:=false`` only when no simulator clock is running.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('px4_control')
    config = os.path.join(share, 'config', 'px4_control.yaml')
    estimation = os.environ.get('ESTIMATION_MODE', 'vision')
    yaw_rate = os.environ.get('PX4_MAX_YAW_RATE_DEG_S', '30.0')
    walls = os.environ.get('PX4_WALL_SEGMENTS_FILE', '')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('estimation_mode', default_value=estimation),
        DeclareLaunchArgument('max_yaw_rate_deg_s', default_value=yaw_rate),
        DeclareLaunchArgument('wall_segments_file', default_value=walls),
        Node(
            package='px4_control',
            executable='px4_control_node',
            name='px4_control',
            output='screen',
            parameters=[
                config,
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'estimation_mode': LaunchConfiguration('estimation_mode'),
                    'max_yaw_rate_deg_s': LaunchConfiguration('max_yaw_rate_deg_s'),
                    'wall_segments_file': LaunchConfiguration('wall_segments_file'),
                },
            ],
        ),
    ])
