from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        'config',
        default_value='xbox_twist_params.yaml',
        description='Xbox twist parameter file'
    )
    config_file = PathJoinSubstitution([
        get_package_share_directory('gogo_teleop'),
        'config',
        LaunchConfiguration('config')
    ])

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
        config_arg,
        joy_node,
        xbox_twist_node
    ])
