import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool

class Mode(Node):
    def __init__(self):
        super().__init__('Mode')
        self.mode = self.create_publisher(String, '/mode', 10)   # publishes a string which is the mode
        self.timer = self.create_timer(0.1, self.mode_select)

    def mode_select(self):
        self.get_logger().info('Type: auto, teleop, ostop, estop')    # prints txt: Type: auto, teleop, ostop, estop
        while rclpy.ok():
            user_message = input()                                     # saves user input into user_message
            if user_message == 'auto':                                 # checks if user put in auto and then sends Autonomous to FSM
                self.mode.publish(String(data='auto'))
            elif user_message == 'teleop':
                self.mode.publish(String(data='teleop'))               # checks if user put in teleop and then sends teleoperated to FSM
            elif user_message == 'ostop':
                self.mode.publish(String(data='ostop'))                # checks if user put in ostop and then sends Obstical Stop to FSM
            elif user_message == 'estop':
                self.mode.publish(String(data='estop'))                # checks if user put in estop and then sends ESTOP to FSM
            else:
                self.get_logger().info('Type: auto, teleop, ostop, estop')


def main(args=None):
    rclpy.init(args=args)
    node = Mode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
