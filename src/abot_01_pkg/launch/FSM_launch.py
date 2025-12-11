from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='abot_01_pkg',
            namespace='turtlesim1',
            executable='moves_square',
            name='mover'
        ),        
        Node(
            package='abot_01_pkg',
            namespace='turtlesim1',
            executable='FSM',
            name='FSM'
        )
        
    ])
