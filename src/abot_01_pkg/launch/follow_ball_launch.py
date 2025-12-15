from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   robot_ns = 'abot_01' 

   return LaunchDescription([
      Node(
          package='abot_01_pkg',
          executable='filter',
          name='color_filter',
          namespace=robot_ns,
          output='screen',
          remappings=[
              ('camera/image_raw', '/camera/image_raw'),
          ],
      ),
      Node(
           package='abot_01_pkg',
           executable='image_info',
           name='image_info',
           namespace=robot_ns,
           output='screen'
       ),
      Node(
          package='abot_01_pkg',
          executable='follow_ball_controller',
          name='follow_ball_controller',
          namespace=robot_ns,
          output='screen',
          parameters=[{
              'focal_px': 664.0,
              'target_distance_m': 0.15,
              'ball_diameter_m': 0.04,
              'frame_width': 640.0
          }]
      ),
       Node(
           package='abot_01_pkg',
           executable='fsm',
           name='fsm',
           namespace=robot_ns,
           output='screen'
       ),
      Node(
          package='abot_01_pkg',
          executable='mode',
          name='mode',
          namespace=robot_ns,
          output='screen'
      ),
   ])
