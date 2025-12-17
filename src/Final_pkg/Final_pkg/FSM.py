import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

#finite state machine node with modes: TELEOP, AUTO, ESTOP
class FSMCode(Node):

    def __init__(self):
        super().__init__('abot_fsm')
        self.current_mode = 'TELEOP' #current operating mode is TELEOP
        self.estop_active = False #ESTOP inactive
        self.teleop_cmd = Twist() #storing velocity commands
        self.auto_cmd = Twist()
        self.create_subscription(Twist, 'teleop/cmd_vel', self.teleop_cb, 10) #subscribe to teleop velocity commands
        self.create_subscription(Twist, 'auto/cmd_vel', self.auto_cb, 10) #subscribe to autonomous velocity commands
        self.create_subscription(String, 'mode', self.mode_cb, 10) #subscribe to mode selection
        self.create_subscription(Bool, 'estop', self.estop_cb, 10) #subscribe to estop signal
        self.cmd_pub = self.create_publisher(Twist, '/abot/cmd_vel', 10) #publish velocity that is sent to robot
        self.timer = self.create_timer(0.05, self.update) #timer callback to run every 0.05 seconds
        self.get_logger().info('FSM started in TELEOP mode') #startup message
    def teleop_cb(self, msg: Twist): #callback for TELEOP velocity commands
        self.teleop_cmd = msg
    def auto_cb(self, msg: Twist): #callback for AUTTO velocity commands
        self.auto_cmd = msg
    def mode_cb(self, msg: String): #mode selection ('t' is TELEOP, 'a' is AUTO, 'e' is ESTOP)
        key = msg.data.strip().lower()
        if key == 't':
            self.current_mode = 'TELEOP'
            self.get_logger().info('Mode -> TELEOP')
        elif key == 'a':
            self.current_mode = 'AUTO'
            self.get_logger().info('Mode -> AUTO')
        elif key == 'e':
            self.current_mode = 'ESTOP'
            self.get_logger().info('Mode -> ESTOP')
        else:
            self.get_logger().warn(f'Unknown mode key: "{msg.data}"') #warning for unknown key press
    def estop_cb(self, msg: Bool): #callback for ESTOP signal
        self.estop_active = msg.data
    def update(self):
        #determines which velocity command to publish based on selected operating mode or ESTOP signal
        cmd = Twist() 
        if self.estop_active or self.current_mode == 'ESTOP':
            pass  # keep zeros
        elif self.current_mode == 'TELEOP':
            cmd = self.teleop_cmd
        elif self.current_mode == 'AUTO':
            cmd = self.auto_cmd
        self.cmd_pub.publish(cmd)
def main(args=None):
    rclpy.init(args=args)
    node = FSMCode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
