from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='interaction_execution',
            executable='task_manager',
            name='task_manager',
            output='screen'
        )
    ])
