from launch import LaunchDescription
from launch_ros.actions import Node


# Launch nodes required for joystick operation


def generate_launch_description():
    return LaunchDescription([
        Node(package='joy', executable='joy_node', output='screen'),
        Node(package='joy_control', executable='joy_control_main', output='screen'),
        Node(package='joy_filter', executable='joy_filter', output='screen'),
    ])
