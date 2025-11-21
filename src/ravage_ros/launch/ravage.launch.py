import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the installed config files
    config_dir = os.path.join(
        get_package_share_directory('ravage_ros'),
        'config'
    )

    return LaunchDescription([
        Node(
            package='ravage_ros',
            executable='attack_engine',
            name='ravage_attacker',
            output='screen',
            parameters=[
                {'config_path': config_dir},  # Pass the directory path
                {'attack_type': 'GPS'},       # Default parameter
                {'intensity': 5.0},
                {'duration': 30.0},
                {'software': 'ArduPilot'}
            ]
        )
    ])
