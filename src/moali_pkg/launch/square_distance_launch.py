from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('moali_pkg'),
                    'move_square_launch.py'
                ])
            ])
        ),
        Node(
            package='moali_pkg',
            namespace='turtlesim1',
            executable='square_distance',
            name='distance'
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'topic',
                'echo',
                '/turtlesim1/turtle1/distance_traveled',
                'std_msgs/Float32'
            ],
            shell=True,
            output='screen'
        )
    ])