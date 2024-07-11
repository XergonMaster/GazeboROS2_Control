import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ackerman_odometry',
            executable='cmd_vel2odom_node',
            name='cmd_vel2odom_node',
            output='screen',
            parameters=[{
                'wheel_base': 0.5  # Ajusta este valor según sea necesario
            }]
        ),
        Node(
            package='ackerman_odometry',
            executable='odom_to_tf_node',
            name='odom_to_tf_node',
            output='screen'
        )
    ])
