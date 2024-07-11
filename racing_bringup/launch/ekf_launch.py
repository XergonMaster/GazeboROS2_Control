from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener la ruta completa del archivo de configuración
    config_file = os.path.join(
        get_package_share_directory('racing_bringup'),
        'config',
        'ekf_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='ackerman_odometry',
            executable='cmd_vel2odom_node',
            name='cmd_vel2odom_node',
            output='screen',
            parameters=[{'wheel_base': 0.2275}]
        ),
        Node(
            package='ackerman_odometry',
            executable='odom_to_tf_node',
            name='odom_to_tf_node',
            output='screen'
        )
    ])
