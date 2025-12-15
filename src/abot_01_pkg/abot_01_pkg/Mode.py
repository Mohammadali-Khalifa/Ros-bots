import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool

class Mode(Node):
    def __init__(self):
        super().__init__('Mode_node')
        self.pub = self.create_publisher(String, 'mode', 10)   # publishes a string which is the mode
    def mode_select(self):
        self.get_logger().info('Type: A for auto, T for teleop, E for estop')    # prints txt
        while rclpy.ok():
            user_message = input().strip().lower()  # saves user input into user_message
            if user_message == 'a':  # auto
                self.pub.publish(String(data='a'))
            elif user_message == 't':  # teleop
                self.pub.publish(String(data='t'))
            elif user_message == 'e':  # estop
                self.pub.publish(String(data='e'))
            else:
                self.get_logger().info('Type: A for auto, T for teleop, E for estop')
def main(args=None):
    rclpy.init(args=args)
    node = Mode()
    node.mode_select()   # run your input loop
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
