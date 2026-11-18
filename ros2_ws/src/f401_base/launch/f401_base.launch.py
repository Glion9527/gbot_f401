from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f401_base',
            executable='f401_base_node',
            name='f401_base',
            output='screen',
            parameters=[{
                'port': '/dev/f401',
                'baud': 115200,
                'cmd_timeout': 0.30,
                'cpr': [3960.0, 3960.0, 3960.0, 3960.0],
                'odom_frame': 'odom',
                'base_frame': 'base_link',
            }],
        )
    ])
