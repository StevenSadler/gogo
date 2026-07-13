from launch import LaunchDescription
from launch_ros.actions import Node

from gogo_description.frames import BASE_LINK_FRAME, LASER_FRAME
from gogo_description.robot_constants import (
    LIDAR_X,
    LIDAR_Y,
    LIDAR_Z,
    LIDAR_ROLL,
    LIDAR_PITCH,
    LIDAR_YAW,
)

def generate_launch_description():

    lidar_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf',
        arguments=[
            str(LIDAR_X),
            str(LIDAR_Y),
            str(LIDAR_Z),
            str(LIDAR_ROLL),
            str(LIDAR_PITCH),
            str(LIDAR_YAW),
            BASE_LINK_FRAME,
            LASER_FRAME,
        ],
    )

    odometry_node = Node(
        package='gogo_localization',
        executable='odometry',
        name='odometry',
        output='screen',
    )

    cmd_vel_mux_node = Node(
        package='gogo_control',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
    )

    twist_serial_node = Node(
        package='gogo_control',
        executable='twist_serial',
        name='twist_serial',
        output='screen',
    )

    lidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar',
        output='screen'
    )

    return LaunchDescription([
        lidar_tf_node,
        odometry_node,
        cmd_vel_mux_node,
        twist_serial_node,
        lidar_node,
    ])
