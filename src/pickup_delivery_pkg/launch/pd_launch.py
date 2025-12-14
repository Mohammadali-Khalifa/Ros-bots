from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns')
    pickup_color = LaunchConfiguration('pickup_color')
    dropoff_color = LaunchConfiguration('dropoff_color')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='abot-01'),
        DeclareLaunchArgument('pickup_color', default_value='blue'),
        DeclareLaunchArgument('dropoff_color', default_value='pink'),

        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            namespace=robot_ns,
            output='screen',
        ),

        Node(
            package='pickup_delivery_pkg',
            executable='multi_color_detector_node',
            name='multi_color_detector_node',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'pickup_color': pickup_color,
                'dropoff_color': dropoff_color,
                'morph_iterations': 2,
            }],
        ),

        Node(
            package='pickup_delivery_pkg',
            executable='pickup_delivery_fsm_node',
            name='pickup_delivery_fsm_node',
            namespace=robot_ns,
            output='screen',
        ),

        Node(
            package='Ball_follow_pkg',
            executable='mode_switcher_node',
            name='mode_switcher_node',
            namespace=robot_ns,
            output='screen',
        ),

        Node(
            package='simple_abot_interface',
            executable='simple_abot_interface',
            name='simple_abot_interface',
            namespace=robot_ns,
            output='screen',
        ),
    ])
