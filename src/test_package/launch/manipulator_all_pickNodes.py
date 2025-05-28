from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='test_package',
            executable='point_publisher.py',
            name='point_publisher',
            output='screen'
        ),
        Node(
            package='test_package',
            executable='tf_listener.py',
            name='tf_listener',
            output='screen'
        ),
        Node(
            package='test_package',
            executable='go_to_pose.py',
            name='go_to_pose',
            output='screen'
        )
    ])

