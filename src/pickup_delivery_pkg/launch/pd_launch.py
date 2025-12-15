from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_ns')
    pickup_color = LaunchConfiguration('pickup_color')
    dropoff_color = LaunchConfiguration('dropoff_color')
    image_topic = LaunchConfiguration('image_topic')

    use_gripper = LaunchConfiguration('use_gripper')
    gripper_port = LaunchConfiguration('gripper_port')
    gripper_baud = LaunchConfiguration('gripper_baud')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ns', default_value='abot-01'),
        DeclareLaunchArgument('pickup_color', default_value='blue'),
        DeclareLaunchArgument('dropoff_color', default_value='pink'),
        DeclareLaunchArgument('image_topic', default_value='camera/image_raw'),

        DeclareLaunchArgument('use_gripper', default_value='true'),
        DeclareLaunchArgument('gripper_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('gripper_baud', default_value='115200'),

        # Camera
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            namespace=robot_ns,
            output='screen',
        ),

        # HSV detector (publishes marker_measurements + image_filtered)
        Node(
            package='pickup_delivery_pkg',
            executable='hsv_detector_node',
            name='hsv_detector_node',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'pickup_color': pickup_color,
                'dropoff_color': dropoff_color,
                'min_area': 250,
            }],
        ),

        # FSM (publishes cmd_vel_auto + gripper_cmd + target_request + mode_request)
        Node(
            package='pickup_delivery_pkg',
            executable='pickup_delivery_fsm_node',
            name='pickup_delivery_fsm_node',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'pickup_color': pickup_color,
                'dropoff_color': dropoff_color,
                'force_autonomous': False,
                # Turn on the lift sequence only if the gripper needs it:
                'use_lift': False,
                'use_gripper': use_gripper,
            }],
        ),

        # Fast mode switching (a/t/s)
        Node(
            package='pickup_delivery_pkg',
            executable='mode_input_node',
            name='mode_input_node',
            namespace=robot_ns,
            output='screen',
        ),

        # Arduino gripper serial bridge (subscribes gripper_cmd)
        Node(
            package='pickup_delivery_pkg',
            executable='gripper_node',
            name='gripper_node',
            namespace=robot_ns,
            output='screen',
            parameters=[{
                'port': gripper_port,
                'baud': gripper_baud,
            }],
        ),

        # Hardware interface
        Node(
            package='simple_abot_interface',
            executable='simple_abot_interface',
            name='simple_abot_interface',
            namespace=robot_ns,
            output='screen',
        ),
    ])
