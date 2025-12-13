from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
       Node(
           package='abot_01_pkg',
           executable='filter',
           name='color_filter',
           output='screen',
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
           output='screen',
           parameters=[{
               'target_distance_m': 0.15,    
               'ball_diameter_m': 0.04,      
               'focal_px': 650.0,
               'frame_width': 640.0,
               'max_lin': 0.4,
               'max_ang': 1.2,
               'kp_ang': 0.004,
               'ki_ang': 0.0,
               'kd_ang': 0.0008,
               'kp_lin': 0.8,
               'ki_lin': 0.0,
               'kd_lin': 0.05,
           }]
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
