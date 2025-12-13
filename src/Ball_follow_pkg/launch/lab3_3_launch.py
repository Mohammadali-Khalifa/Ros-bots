from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    target_color = LaunchConfiguration('target_color')
    morph_iterations = LaunchConfiguration('morph_iterations')
    robot_ns = 'abot-01'

    return LaunchDescription([
        DeclareLaunchArgument(
            'target_color',
            default_value='blue',
            description="Color to filter: 'blue', 'pink', or 'orange'",
        ),
        
        DeclareLaunchArgument(
            'morph_iterations',
            default_value='2',
            description='Number of erode/dilate iterations',
        ),

        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            namespace=robot_ns,
            output='screen',
        ),

        Node(
            package='color_filter_pkg',
            executable='filter',
            name='color_filter',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'target_color': target_color,
                'erode_dilate_iters': morph_iterations,
            }],
        ),

        Node(
            package='color_filter_pkg',
            executable='image_info',
            name='image_info',
            namespace=robot_ns,
            output='screen',
        ),

        Node(
            package='Ball_follow_pkg',
            executable='ball_follow_controller_node',
            name='ball_follow_controller_node',
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
            package='Ball_follow_pkg',
            executable='mode_input_node',
            name='mode_input_node',
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

        Node(
            package='image_view',
            executable='image_view',
            name='filtered_image_view',
            namespace=robot_ns,
            output='screen',
            remappings=[
                ('image', 'image_filtered'),
            ],
        ),
    ])
