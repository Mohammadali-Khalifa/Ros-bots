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
            package='abot_01_pkg',
            namespace='turtlesim1',
            executable='square_distance',
            name='distance'
        ),
        Node(
            package='abot_01_pkg',
            namespace='turtlesim1',
            executable='convert',
            name='unit_convert',
            output='screen',
            parameters=[              
                {'user_input': 'smoot'},               
                {'input': 'turtle1/distance_traveled'},  
                {'output': 'converted_distance'},          
            ]
        ),
        ExecuteProcess(
            cmd=[
                'ros2',
                'topic',
                'echo',
                '/turtlesim1/converted_distance',
                'std_msgs/Float32'
            ],
            shell=True,
            output='screen'
        )
    ])

    #
#from launch import LaunchDescription
#from launch_ros.actions import Node
#from launch.actions import IncludeLaunchDescription
#from launch.actions import ExecuteProcess
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch.substitutions import PathJoinSubstitution, TextSubstitution
#from launch_ros.substitutions import FindPackageShare

#def generate_launch_description():
#    return LaunchDescription([
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource([
#                PathJoinSubstitution([
#                    FindPackageShare('<PACKAGE_NAME>'),
#                    '<LAB1.2_LAUNCH_FILE_NAME>'
#                ])
#            ])
#        ),
#        Node(
#            package='<PACKAGE_NAME>',
#            namespace='turtlesim1',
#            executable='<LAB1.3_NODE_NAME>',
#            name='<YOUR_CHOICE>'
#        ),
#        ExecuteProcess(
#            cmd=[
#                'ros2',
#                'topic',
#                'echo',
#                '<TOPIC_NAME>',
#                'std_msgs/Float32'
#            ],
#            shell=True,
#            output='screen'
#        )
#    ])
#    '''
