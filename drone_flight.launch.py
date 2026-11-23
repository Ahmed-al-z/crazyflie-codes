from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_flight',
            executable='flight_server',
            output='screen',
            parameters=[{
                'cmd_topic': '/cmd_hover',
                'hover_height': 0.4,
                'max_flight_time': 60.0,
            }]),
    ])
