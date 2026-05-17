from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='interaction_execution',
            executable='ss3_task_coordinator',
	    name='ss3_task_coordinator',
            output='screen'
        )
    ])
