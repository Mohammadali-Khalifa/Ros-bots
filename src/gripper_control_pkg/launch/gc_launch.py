from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns')
    port = LaunchConfiguration('port')
    baud = LaunchConfiguration('baud')
    use_keyboard = LaunchConfiguration('use_keyboard')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='abot-01'),
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        DeclareLaunchArgument('use_keyboard', default_value='false'),

        Node(
            package='gripper_control_pkg',
            executable='gripper_serial_node',
            name='gripper_serial_node',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'port': port,
                'baud': baud,
            }],
        ),
    ])
