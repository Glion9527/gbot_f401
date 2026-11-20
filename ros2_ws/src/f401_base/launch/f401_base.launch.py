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
                'imu_frame': 'imu_link',
                # base_link 当前定义在四轮中心的地面投影点，x 前、y 左、z 上。
                # 2026-11-20 实测：IMU 中心在四轮中心后方 3.5cm、右侧 0.5cm、离地 6.5cm。
                'imu_x': -0.035,
                'imu_y': -0.005,
                'imu_z': 0.065,
                # 2026-11-20 重力/倾斜法实测
                'imu_roll': -0.0466,
                'imu_pitch': 0.0363,
                'imu_yaw': 0.1246,
            }],
        )
    ])
