import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool

class Mode(Node):
    def __init__(self):
        super().__init__('Mode_node')
        self.pub = self.create_publisher(String, 'mode', 10)   # publishes a string which is the mode

    def mode_select(self):
        self.get_logger().info('Type: auto, teleop, ostop, estop')    # prints txt: Type: auto, teleop, ostop, estop
        while rclpy.ok():
            user_message = input().strip().lower()                    # saves user input into user_message

            if user_message == 'auto':                                # checks if user put in auto and then sends Autonomous to FSM
                self.pub.publish(String(data='a'))
            elif user_message == 'teleop':
                self.pub.publish(String(data='t'))                    # checks if user put in teleop and then sends teleoperated to FSM
            elif user_message == 'ostop':
                self.get_logger().info('ostop not implemented in FSM yet (use estop or add a mode)')  # keeps your option visible
            elif user_message == 'estop':
                self.pub.publish(String(data='e'))                    # checks if user put in estop and then sends ESTOP to FSM
            else:
                self.get_logger().info('Type: auto, teleop, ostop, estop')


def main(args=None):
    rclpy.init(args=args)
    node = Mode()
    node.mode_select()   # run your input loop
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
