from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
   return LaunchDescription([
       Node(
           package='abot_01_pkg',
           executable='filter',
           name='color_filter',
           output='screen'
       ),
       Node(
           package='abot_01_pkg',
           executable='image_info',
           name='image_info',
           output='screen'
       ),
       Node(
           package='abot_01_pkg',
           executable='follow_ball_controller',
           name='follow_ball_controller',
           output='screen'
       ),
       Node(
           package='abot_01_pkg',
           executable='fsm',
           name='fsm',
           output='screen'
       ),
       Node(
           package='abot_01_pkg',
           executable='mode',
           name='mode',
           output='screen'
       ),
   ])
