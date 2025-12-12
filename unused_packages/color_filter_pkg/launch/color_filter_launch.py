from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    image_file   = LaunchConfiguration('image_file')
    target_color = LaunchConfiguration('target_color')

    return LaunchDescription([
        DeclareLaunchArgument(
            'image_file',
            default_value=(
                '/ros_ws/src/student_src/color_filter_pkg/images/'
                'blue_pingpong/image_blue_0.png'
            ),
        ),
        DeclareLaunchArgument(
            'target_color',
            default_value='blue',
        ),

        Node(
            package='image_publisher',
            executable='image_publisher_node',
            name='saved_pic',
            arguments=[image_file],
            remappings=[('image', '/image_raw')],
        ),

        Node(
            package='color_filter_pkg',
            executable='filter',
            name='color_filter',
            parameters=[{
                'target_color': target_color,
                'erode_dilate_iters': 2,
            }],
        ),

        Node(
            package='image_view',
            executable='image_view',
            name='viewer',
            remappings=[('image', '/image_filtered')],
        ),
    ])
