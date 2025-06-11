from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mimic_controller',
            executable='mimic_publisher',
            name='mimic_publisher',
            output='screen'
        ),
        Node(
            package='mimic_controller',
            executable='mimic_to_moveit',
            name='mimic_follower',
            output='screen'
        )
    ])