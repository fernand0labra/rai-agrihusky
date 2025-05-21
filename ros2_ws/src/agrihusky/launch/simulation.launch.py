from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='agrihusky',
            executable='fake_gps_pub.py',
            name='fake_gps_pub',
            output='screen'
        ),
        Node(
            package='agrihusky',
            executable='fake_odometry_pub.py',
            name='fake_odometry_pub',
            output='screen'
        ),
        Node(
            package='agrihusky',
            executable='visualizer.py',
            name='visualizer',
            output='screen'
        ),
        Node(
            package='agrihusky',
            executable='controller.py',
            name='controller',
            output='screen'
        )
    ])
