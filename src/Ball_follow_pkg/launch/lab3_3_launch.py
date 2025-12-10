from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    target_color = LaunchConfiguration('target_color')
    morph_iterations = LaunchConfiguration('morph_iterations')

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
            package='lab3_1_color_filter',
            executable='color_filter_node',
            name='color_filter_node',
            output='screen',
            parameters=[{
                'target_color': target_color,
                'morph_iterations': morph_iterations,
            }],
        ),

        Node(
            package='lab3_1_color_filter',
            executable='ball_measure_node',
            name='ball_measure_node',
            output='screen',
            parameters=[{
                'debug_ball': True,
            }],
        ),

        Node(
            package='lab3_3',
            executable='ball_follow_controller_node',
            name='ball_follow_controller_node',
            output='screen',
        ),

        Node(
            package='lab3_3',
            executable='mode_switcher_node',
            name='mode_switcher_node',
            output='screen',
        ),
        
        Node(
            package='image_view',
            executable='image_view',
            name='filtered_image_view',
            output='screen',
            remappings=[
                ('image', 'filtered_image'),
            ],
        ),
    ])
