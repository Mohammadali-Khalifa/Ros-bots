import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class EStop(Node):
    def __init__(self):
        super().__init__('estop_reset')
        self.publisher_ = self.create_publisher(Bool, '/estop_reset', 10)  # publishes true when reset

    def loop(self):
        self.get_logger().info("Type 'reset' to RESET")
        while rclpy.ok():
            reset = input()                   #puts the user input into reset
            if reset == 'reset':               # if reste == reset, it will reset the bool and send the signal to unlach the ESTOP
                msg = Bool()
                msg.data = True               
                self.publisher_.publish(msg)
                self.get_logger().info('E-STOP reset') #prints to terminal that it was reset 

def main(args=None):
    rclpy.init(args=args)
    node = EStop()
    node.loop()             
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
