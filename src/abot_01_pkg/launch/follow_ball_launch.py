from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
   return LaunchDescription([
       # Color filter from Lab 3.1 (make sure target_color == 'blue')
       Node(
           package='color_filter_pkg',
           executable='filter',
           name='color_filter',
           parameters=[{'target_color': 'blue'}],
           output='screen'
       ),
       # Image info from Lab 3.2
       Node(
           package='color_filter_pkg',
           executable='image_info',
           name='image_info',
           output='screen'
       ),
       # PID controller that turns image_info into /auto/cmd_vel
       Node(
           package='abot_01_pkg',
           executable='follow_ball_controller',
           name='follow_ball_controller',
           output='screen'
       ),
       # FSM that muxes teleop vs auto into /abot/cmd_vel
       Node(
           package='abot_01_pkg',
           executable='fsm',
           name='fsm',
           output='screen'
       ),
       # Mode node for fast mode switching (t/a/e)
       Node(
           package='abot_01_pkg',
           executable='mode',
           name='mode',
           output='screen'
       ),
   ])
