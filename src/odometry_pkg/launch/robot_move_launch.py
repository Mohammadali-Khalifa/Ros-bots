from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odometry_helper',
            executable='wheel_dist_pub',
            name='wheel_dist_pub',
            output='screen',
        ),
        Node(
            package='odometry_helper',
            executable='odom_graph',
            name='odom_graph',
            output='screen',
        ),
        Node(
            package='odometry_pkg',
            executable='robot_move',
            name='robot_move',
            output='screen',
        ),
    ])
