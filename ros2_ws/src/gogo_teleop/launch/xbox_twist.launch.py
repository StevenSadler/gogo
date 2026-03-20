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

    # Joystick device path
    # Default is set for Pi; on VM override with device:=/dev/input/js2
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/input/js0',  # Pi default
        description='Joystick device path (override for VM if needed)'
    )
    device_path = LaunchConfiguration('device')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': device_path}]
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
        device_arg,
        joy_node,
        xbox_twist_node
    ])
