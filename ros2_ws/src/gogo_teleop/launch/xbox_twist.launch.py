from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('gogo_teleop'),
        'config',
        'xbox_twist_params.yaml'
    )

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js2'}]
        )
    
    xbox_twist_node = Node(
        package='gogo_teleop',
        executable='xbox_twist',
        name='xbox_twist',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        joy_node,
        xbox_twist_node
    ])
